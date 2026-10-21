#include "radar/io/config.hpp"

#include <cmath>
#include <fstream>
#include <ios>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>

// src/io/config.cpp — scenario JSON -> ScenarioConfig.
//
// Every access to the parsed json goes through the small typed helpers below, which
// turn a missing key, a wrong type, or a non-finite value into a ConfigError naming the
// path (ADR-0007). No nlohmann::json::exception escapes this file: parse() is wrapped,
// and the helpers never call bare operator[] / get<T>() on untyped values. This is the
// release-build boundary that backs motion's debug-only finite-input assert (issue #5).

namespace radar {

namespace {

using nlohmann::json;

// Build "<source>: <path>" message prefixes so every error names where it came from.
// source and path are both string_view; a swap only reorders the two halves of a
// diagnostic string (never silent corruption) and is caught by ErrorMessageNamesTheSource.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] std::string where(std::string_view source, std::string_view path) {
  std::string msg(source);
  msg += ": ";
  msg += path;
  return msg;
}

[[nodiscard]] const json& require(const json& obj, const char* key, std::string_view source,
                                  std::string_view path) {
  if (!obj.is_object()) {
    throw ConfigError(where(source, path) + " must be a JSON object");
  }
  const auto it = obj.find(key);
  if (it == obj.end()) {
    throw ConfigError(where(source, std::string(path) + "." + key) + " is required but missing");
  }
  return *it;
}

// A finite double at `obj[key]`. Rejects wrong type and non-finite (NaN/inf). In practice
// nlohmann already rejects NaN/Infinity tokens (parse_error) and overflowing literals like
// 1e400 (out_of_range) at parse time, so this isfinite guard is defense-in-depth — cheap
// insurance against any future non-finite path reaching a field, since a single inf in a
// target state would spread to every component through motion's F*x.
[[nodiscard]] double req_finite(const json& obj, const char* key, std::string_view source,
                                std::string_view path) {
  const json& v = require(obj, key, source, path);
  if (!v.is_number()) {
    throw ConfigError(where(source, std::string(path) + "." + key) + " must be a number");
  }
  const double d = v.get<double>();
  if (!std::isfinite(d)) {
    throw ConfigError(where(source, std::string(path) + "." + key) + " must be finite");
  }
  return d;
}

// Optional finite double with a default (for M2 fields that may be absent in M1 files).
[[nodiscard]] double opt_finite(const json& obj, const char* key, double fallback,
                                std::string_view source, std::string_view path) {
  if (!obj.is_object() || obj.find(key) == obj.end()) {
    return fallback;
  }
  return req_finite(obj, key, source, path);
}

[[nodiscard]] int req_int(const json& obj, const char* key, std::string_view source,
                          std::string_view path) {
  const json& v = require(obj, key, source, path);
  if (!v.is_number_integer()) {
    throw ConfigError(where(source, std::string(path) + "." + key) + " must be an integer");
  }
  return v.get<int>();
}

[[nodiscard]] int opt_int(const json& obj, const char* key, int fallback, std::string_view source,
                          std::string_view path) {
  if (!obj.is_object() || obj.find(key) == obj.end()) {
    return fallback;
  }
  return req_int(obj, key, source, path);
}

void require_positive(double value, std::string_view source, std::string_view path) {
  if (value <= 0.0) {
    throw ConfigError(where(source, path) + " must be positive");
  }
}

SensorSpec parse_sensor(const json& root, std::string_view source) {
  SensorSpec s;
  const auto it = root.find("sensor");
  if (it == root.end()) {
    return s;  // all defaults; sensor block is optional at M1
  }
  const json& j = *it;
  s.sigma_m = opt_finite(j, "sigma_m", s.sigma_m, source, "sensor");
  // Issue #5 (sensor extension): a negative/NaN sigma_m would silently disable noise.
  // NaN already rejected by opt_finite; reject negatives here.
  if (s.sigma_m < 0.0) {
    throw ConfigError(where(source, "sensor.sigma_m") + " must be non-negative");
  }
  s.p_detect = opt_finite(j, "p_detect", s.p_detect, source, "sensor");       // M2, carried
  s.clutter_per_scan = opt_finite(j, "clutter_per_scan", s.clutter_per_scan,  // M2, carried
                                  source, "sensor");
  return s;
}

TrackerSpec parse_tracker(const json& root, std::string_view source) {
  TrackerSpec t;
  const auto it = root.find("tracker");
  if (it == root.end()) {
    return t;  // all defaults
  }
  const json& j = *it;
  t.sigma_a = opt_finite(j, "sigma_a", t.sigma_a, source, "tracker");
  if (t.sigma_a < 0.0) {
    throw ConfigError(where(source, "tracker.sigma_a") + " must be non-negative");
  }
  t.gate_chi2 = opt_finite(j, "gate_chi2", t.gate_chi2, source, "tracker");
  t.confirm_m = opt_int(j, "confirm_m", t.confirm_m, source, "tracker");
  t.confirm_n = opt_int(j, "confirm_n", t.confirm_n, source, "tracker");
  t.delete_after_misses =
      opt_int(j, "delete_after_misses", t.delete_after_misses, source, "tracker");
  return t;
}

TargetSpec parse_target(const json& j, std::size_t index, std::string_view source) {
  const std::string path = "targets[" + std::to_string(index) + "]";
  TargetSpec t;
  t.id = req_int(j, "id", source, path);
  // Issue #5 (sensor extension): id = -1 collides with kClutterOrigin — a real target
  // would be scored as clutter. Reject any negative id.
  if (t.id < 0) {
    throw ConfigError(where(source, path + ".id") +
                      " must be non-negative (-1 is the clutter tag)");
  }
  t.x0 = req_finite(j, "x0", source, path);
  t.y0 = req_finite(j, "y0", source, path);
  t.vx0 = req_finite(j, "vx0", source, path);
  t.vy0 = req_finite(j, "vy0", source, path);

  // Maneuvers: parsed but must be empty in M1 (CT deferred, ADR-0006).
  const auto man = j.find("maneuvers");
  if (man != j.end()) {
    if (!man->is_array()) {
      throw ConfigError(where(source, path + ".maneuvers") + " must be an array");
    }
    if (!man->empty()) {
      throw ConfigError(where(source, path + ".maneuvers") +
                        " is not supported in M1 (the coordinated-turn model is deferred); "
                        "it must be empty or absent");
    }
  }
  return t;
}

ScenarioConfig build(const json& root, std::string_view source) {
  if (!root.is_object()) {
    throw ConfigError(where(source, "<root>") + " must be a JSON object");
  }

  ScenarioConfig cfg;
  cfg.dt_s = req_finite(root, "dt_s", source, "");
  require_positive(cfg.dt_s, source, "dt_s");  // non-positive dt poisons propagation

  cfg.duration_s = opt_finite(root, "duration_s", cfg.duration_s, source, "");
  require_positive(cfg.duration_s, source, "duration_s");

  cfg.area_half_width_m = opt_finite(root, "area_half_width_m", cfg.area_half_width_m, source, "");
  require_positive(cfg.area_half_width_m, source, "area_half_width_m");

  // seed: optional non-negative integer.
  if (const auto it = root.find("seed"); it != root.end()) {
    if (!it->is_number_integer() && !it->is_number_unsigned()) {
      throw ConfigError(where(source, "seed") + " must be an integer");
    }
    const auto s = it->get<long long>();
    if (s < 0) {
      throw ConfigError(where(source, "seed") + " must be non-negative");
    }
    cfg.seed = static_cast<std::uint64_t>(s);
  }

  cfg.sensor = parse_sensor(root, source);
  cfg.tracker = parse_tracker(root, source);

  const json& targets = require(root, "targets", source, "");
  if (!targets.is_array()) {
    throw ConfigError(where(source, "targets") + " must be an array");
  }
  std::set<int> seen_ids;
  cfg.targets.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    TargetSpec t = parse_target(targets[i], i, source);
    // Issue #5 (sensor extension): duplicate ids break origin-based scoring.
    if (!seen_ids.insert(t.id).second) {
      throw ConfigError(where(source, "targets[" + std::to_string(i) + "].id") +
                        " duplicates an earlier target id (" + std::to_string(t.id) + ")");
    }
    cfg.targets.push_back(std::move(t));
  }
  return cfg;
}

}  // namespace

// The parameter order (text, then source-name) is the public signature in config.hpp and
// cannot be reordered; source_name defaults, so most callers pass only json_text. A swap
// is caught immediately (the "JSON" would be a file path and fail to parse).
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ScenarioConfig parse_scenario(std::string_view json_text, std::string_view source_name) {
  json root;
  try {
    root = json::parse(json_text);
  } catch (const json::exception& e) {
    // Covers syntax errors (parse_error) AND number overflow like 1e400 (out_of_range
    // 406) — nlohmann throws the latter from parse(), not just malformed syntax. Either
    // way it becomes a ConfigError naming the source; no json exception escapes io.
    throw ConfigError(std::string(source_name) + ": invalid JSON (" + e.what() + ")");
  }
  return build(root, source_name);
}

ScenarioConfig load_scenario(const std::filesystem::path& file) {
  // in is not const: extracting its rdbuf below updates the stream's state.
  // NOLINTNEXTLINE(misc-const-correctness)
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    throw ConfigError(file.string() + ": cannot open scenario file");
  }
  // Slurp via the stream buffer rather than a pair of istreambuf_iterators: the iterator
  // idiom makes GCC 14 emit a false-positive -Wnull-dereference inside libstdc++'s inlined
  // streambuf under -Werror. rdbuf() is equivalent, clearer, and warning-clean.
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return parse_scenario(buffer.str(), file.string());
}

}  // namespace radar
