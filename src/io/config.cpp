#include "radar/io/config.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <ios>
#include <limits>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>

#include "radar/types.hpp"  // kClutterOrigin

// src/io/config.cpp — scenario JSON -> ScenarioConfig.
//
// Every access to the parsed json goes through the small typed helpers below, which turn
// a missing key, a wrong type, an out-of-range value, or a non-finite value into a
// ConfigError naming the path (ADR-0007). No nlohmann::json::exception escapes this file:
// parse() is wrapped, and the helpers never call bare operator[] / get<T>() on untyped
// values. This is the release-build boundary that backs motion's debug-only finite-input
// assert (issue #5).

namespace radar {

namespace {

using nlohmann::json;

// Join a parent path and a key into a dotted path WITHOUT a leading dot for top-level
// keys: join_path("", "dt_s") -> "dt_s"; join_path("sensor", "sigma_m") -> "sensor.sigma_m".
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] std::string join_path(std::string_view parent, std::string_view key) {
  std::string out(parent);
  if (!out.empty()) {
    out += '.';
  }
  out += key;
  return out;
}

// "<source>: <path>" prefix so every error names where it came from. For the root a path
// of "" would read "source: ", so callers pass "<root>" explicitly for that case. A swap
// only reorders two message fragments and is caught by ErrorMessageNamesTheSource.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] std::string at(std::string_view source, std::string_view path) {
  std::string msg(source);
  msg += ": ";
  msg += path;
  return msg;
}

[[nodiscard]] const json& require(const json& obj, const char* key, std::string_view source,
                                  std::string_view path) {
  if (!obj.is_object()) {
    throw ConfigError(at(source, path.empty() ? "<root>" : path) + " must be a JSON object");
  }
  const auto it = obj.find(key);
  if (it == obj.end()) {
    throw ConfigError(at(source, join_path(path, key)) + " is required but missing");
  }
  return *it;
}

// A finite double at obj[key]. Rejects wrong type and non-finite (NaN/inf). nlohmann
// already rejects NaN/Infinity tokens and overflowing literals (1e400) at parse time, so
// the isfinite guard is documented defense-in-depth against any future non-finite path.
[[nodiscard]] double req_finite(const json& obj, const char* key, std::string_view source,
                                std::string_view path) {
  const json& v = require(obj, key, source, path);
  const std::string field = join_path(path, key);
  if (!v.is_number()) {
    throw ConfigError(at(source, field) + " must be a number");
  }
  const double d = v.get<double>();
  if (!std::isfinite(d)) {
    throw ConfigError(at(source, field) + " must be finite");
  }
  return d;
}

[[nodiscard]] double opt_finite(const json& obj, const char* key, double fallback,
                                std::string_view source, std::string_view path) {
  if (obj.find(key) == obj.end()) {
    return fallback;
  }
  return req_finite(obj, key, source, path);
}

// A JSON integer that fits in `int`. Range-checks BEFORE narrowing so a too-large value is
// rejected rather than silently wrapped. nlohmann stores an integer above INT64_MAX as
// number_unsigned, where get<int64_t>() would cast it negative (e.g. 18446744073709551615
// -> -1, which then sneaks past an int-range check with a misleading message); so unsigned
// values are read as uint64_t and compared against INT_MAX directly (PR #14 re-review #2),
// mirroring the seed path.
[[nodiscard]] int req_int(const json& obj, const char* key, std::string_view source,
                          std::string_view path) {
  const json& v = require(obj, key, source, path);
  const std::string field = join_path(path, key);
  if (!v.is_number_integer()) {
    throw ConfigError(at(source, field) + " must be an integer");
  }
  if (v.is_number_unsigned()) {
    const std::uint64_t u = v.get<std::uint64_t>();
    if (u > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      throw ConfigError(at(source, field) + " is out of range for a 32-bit integer");
    }
    return static_cast<int>(u);
  }
  const std::int64_t n = v.get<std::int64_t>();
  if (n < std::numeric_limits<int>::min() || n > std::numeric_limits<int>::max()) {
    throw ConfigError(at(source, field) + " is out of range for a 32-bit integer");
  }
  return static_cast<int>(n);
}

[[nodiscard]] int opt_int(const json& obj, const char* key, int fallback, std::string_view source,
                          std::string_view path) {
  if (obj.find(key) == obj.end()) {
    return fallback;
  }
  return req_int(obj, key, source, path);
}

void require_positive(double value, std::string_view source, std::string_view path) {
  if (value <= 0.0) {
    throw ConfigError(at(source, path) + " must be positive");
  }
}

// The value at root[block] as an object, or throw. An absent block is allowed (returns
// nullptr -> caller keeps defaults); a present-but-wrong-typed block is rejected (review
// #3): "sensor": 0 / "x" / [ ... ] no longer silently loads with defaults.
[[nodiscard]] const json* optional_object(const json& root, const char* block,
                                          std::string_view source) {
  const auto it = root.find(block);
  if (it == root.end()) {
    return nullptr;
  }
  if (!it->is_object()) {
    throw ConfigError(at(source, block) + " must be a JSON object");
  }
  return &*it;
}

SensorSpec parse_sensor(const json& root, std::string_view source) {
  SensorSpec s;
  const json* j = optional_object(root, "sensor", source);
  if (j == nullptr) {
    return s;  // all defaults; sensor block is optional at M1
  }
  s.sigma_m = opt_finite(*j, "sigma_m", s.sigma_m, source, "sensor");
  // Issue #5 (sensor extension): a negative sigma_m would silently disable noise.
  if (s.sigma_m < 0.0) {
    throw ConfigError(at(source, "sensor.sigma_m") + " must be non-negative");
  }
  // M2 fields, carried but validated now so a bad scenario fails at load (review #4):
  s.p_detect = opt_finite(*j, "p_detect", s.p_detect, source, "sensor");
  if (s.p_detect < 0.0 || s.p_detect > 1.0) {
    throw ConfigError(at(source, "sensor.p_detect") + " must be in [0, 1]");
  }
  s.clutter_per_scan = opt_finite(*j, "clutter_per_scan", s.clutter_per_scan, source, "sensor");
  if (s.clutter_per_scan < 0.0) {
    throw ConfigError(at(source, "sensor.clutter_per_scan") + " must be non-negative");
  }
  return s;
}

TrackerSpec parse_tracker(const json& root, std::string_view source) {
  TrackerSpec t;
  const json* j = optional_object(root, "tracker", source);
  if (j == nullptr) {
    return t;  // all defaults
  }
  t.sigma_a = opt_finite(*j, "sigma_a", t.sigma_a, source, "tracker");
  if (t.sigma_a < 0.0) {
    throw ConfigError(at(source, "tracker.sigma_a") + " must be non-negative");
  }
  t.gate_chi2 = opt_finite(*j, "gate_chi2", t.gate_chi2, source, "tracker");
  if (t.gate_chi2 <= 0.0) {
    throw ConfigError(at(source, "tracker.gate_chi2") + " must be positive");
  }
  t.confirm_m = opt_int(*j, "confirm_m", t.confirm_m, source, "tracker");
  t.confirm_n = opt_int(*j, "confirm_n", t.confirm_n, source, "tracker");
  t.delete_after_misses =
      opt_int(*j, "delete_after_misses", t.delete_after_misses, source, "tracker");
  // Review #4: M-of-N must be sensible (1 <= m <= n); deletion count at least 1.
  if (t.confirm_m < 1 || t.confirm_m > t.confirm_n) {
    throw ConfigError(at(source, "tracker.confirm_m") +
                      " must satisfy 1 <= confirm_m <= confirm_n");
  }
  if (t.delete_after_misses < 1) {
    throw ConfigError(at(source, "tracker.delete_after_misses") + " must be at least 1");
  }
  return t;
}

TargetSpec parse_target(const json& j, std::size_t index, std::string_view source) {
  const std::string path = "targets[" + std::to_string(index) + "]";
  if (!j.is_object()) {
    throw ConfigError(at(source, path) + " must be a JSON object");
  }
  TargetSpec t;
  t.id = req_int(j, "id", source, path);
  // Issue #5 (sensor extension): a negative id would collide with the clutter sentinel
  // (kClutterOrigin), so a real target would be scored as clutter.
  if (t.id < 0) {
    throw ConfigError(at(source, path + ".id") + " must be non-negative (" +
                      std::to_string(kClutterOrigin) + " is the clutter tag)");
  }
  t.x0 = req_finite(j, "x0", source, path);
  t.y0 = req_finite(j, "y0", source, path);
  t.vx0 = req_finite(j, "vx0", source, path);
  t.vy0 = req_finite(j, "vy0", source, path);

  // Maneuvers: parsed but must be empty in M1 (CT deferred, ADR-0006).
  const auto man = j.find("maneuvers");
  if (man != j.end()) {
    if (!man->is_array()) {
      throw ConfigError(at(source, path + ".maneuvers") + " must be an array");
    }
    if (!man->empty()) {
      throw ConfigError(at(source, path + ".maneuvers") +
                        " is not supported in M1 (the coordinated-turn model is deferred); "
                        "it must be empty or absent");
    }
  }
  return t;
}

ScenarioConfig build(const json& root, std::string_view source) {
  if (!root.is_object()) {
    throw ConfigError(at(source, "<root>") + " must be a JSON object");
  }

  ScenarioConfig cfg;
  cfg.dt_s = req_finite(root, "dt_s", source, "");
  require_positive(cfg.dt_s, source, "dt_s");  // non-positive dt poisons propagation

  cfg.duration_s = opt_finite(root, "duration_s", cfg.duration_s, source, "");
  require_positive(cfg.duration_s, source, "duration_s");

  cfg.area_half_width_m = opt_finite(root, "area_half_width_m", cfg.area_half_width_m, source, "");
  require_positive(cfg.area_half_width_m, source, "area_half_width_m");

  // seed: optional unsigned integer. ScenarioConfig::seed is uint64, so accept the full
  // unsigned range (review #2: 18446744073709551615 is a legal seed, not "negative").
  if (const auto it = root.find("seed"); it != root.end()) {
    if (!it->is_number_integer()) {  // true for both signed and unsigned JSON integers
      throw ConfigError(at(source, "seed") + " must be an integer");
    }
    if (it->is_number_unsigned()) {
      cfg.seed = it->get<std::uint64_t>();
    } else {
      const std::int64_t s = it->get<std::int64_t>();
      if (s < 0) {
        throw ConfigError(at(source, "seed") + " must be non-negative");
      }
      cfg.seed = static_cast<std::uint64_t>(s);
    }
  }

  cfg.sensor = parse_sensor(root, source);
  cfg.tracker = parse_tracker(root, source);

  const json& targets = require(root, "targets", source, "");
  if (!targets.is_array()) {
    throw ConfigError(at(source, "targets") + " must be an array");
  }
  std::set<int> seen_ids;
  cfg.targets.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    TargetSpec t = parse_target(targets[i], i, source);
    // Issue #5 (sensor extension): duplicate ids break origin-based scoring.
    if (!seen_ids.insert(t.id).second) {
      throw ConfigError(at(source, "targets[" + std::to_string(i) + "].id") +
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
  // Review #8: reject a directory (or other non-regular path) up front. On Linux,
  // std::ifstream happily "opens" a directory and then `<< rdbuf()` yields an EMPTY
  // buffer with NO stream error (good() stays true), so without this check the user would
  // see a misleading "invalid JSON (empty input)" instead of "cannot read". Checking
  // is_directory before opening is the portable way to distinguish the two.
  std::error_code ec;
  if (std::filesystem::is_directory(file, ec)) {
    throw ConfigError(file.string() + ": is a directory, not a scenario file");
  }

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
  if (in.bad()) {  // a genuine read error (not EOF) after a successful open
    throw ConfigError(file.string() + ": cannot read scenario file");
  }
  return parse_scenario(buffer.str(), file.string());
}

}  // namespace radar
