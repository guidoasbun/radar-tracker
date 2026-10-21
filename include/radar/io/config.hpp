#ifndef RADAR_IO_CONFIG_HPP
#define RADAR_IO_CONFIG_HPP

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace radar {

/// Error thrown for an invalid scenario: a syntax error, a missing/wrong-typed key, or
/// a failed validation (issue #5). The message names the source and the offending JSON
/// path, e.g. "single.json: targets[1].vx0 is not finite". Dedicated (not
/// std::invalid_argument) so apps/* can map a bad scenario to exit code 1 without
/// swallowing a genuine library bug — see ADR-0007. Derives from std::runtime_error, so
/// a caller that only catches std::exception still works.
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// A maneuver segment (coordinated turn). Parsed from the scenario, but in M1 the list
/// MUST be empty: the CT motion model is deferred (ADR-0006), so a non-empty maneuver
/// list is rejected at load. The fields are carried so M2 needs no re-plumbing.
struct ManeuverSpec {
  double t_start_s{0.0};
  double t_end_s{0.0};
  double turn_rate_deg_s{0.0};
};

/// One scenario target: a stable id and its initial CV state. `maneuvers` is empty in M1.
struct TargetSpec {
  int id{0};
  double x0{0.0};
  double y0{0.0};
  double vx0{0.0};
  double vy0{0.0};
  std::vector<ManeuverSpec> maneuvers;
};

/// Sensor parameters. `sigma_m` is the only field M1 uses; `p_detect` and
/// `clutter_per_scan` are parsed (if present) and carried for M2's full sensor model,
/// but unused by the noise-only M1 sensor.
struct SensorSpec {
  double sigma_m{50.0};
  double p_detect{0.9};
  double clutter_per_scan{2.0};
};

/// Tracker parameters. Carried for M2 (association/track/tracker); of these, only
/// `sigma_a` is consumed in M1 (it feeds the Kalman filter's process-noise config).
struct TrackerSpec {
  double sigma_a{5.0};
  double gate_chi2{9.21};
  int confirm_m{3};
  int confirm_n{5};
  int delete_after_misses{5};
};

/// A fully-parsed, validated scenario. Produced by load_scenario/parse_scenario; the
/// single source of truth for a run (the tracker block travels with the scenario).
struct ScenarioConfig {
  double dt_s{0.1};
  double duration_s{60.0};
  std::uint64_t seed{42};
  double area_half_width_m{20000.0};
  SensorSpec sensor;
  TrackerSpec tracker;
  std::vector<TargetSpec> targets;
};

/// Load and validate a scenario from a JSON file. Throws ConfigError if the file is
/// missing, is not valid JSON, is missing a required key, has a wrong-typed key, or
/// fails validation (issue #5: non-finite/non-positive dt_s, non-finite target states,
/// non-finite/negative sigma_m, negative or duplicate target ids, a non-empty maneuver
/// list). The message names the file and the offending path.
[[nodiscard]] ScenarioConfig load_scenario(const std::filesystem::path& file);

/// Parse and validate a scenario from an in-memory JSON string. `source_name` is used
/// only in error messages (so tests can drive validation without temp files). Same
/// throwing contract as load_scenario.
[[nodiscard]] ScenarioConfig parse_scenario(std::string_view json_text,
                                            std::string_view source_name = "<memory>");

}  // namespace radar

#endif  // RADAR_IO_CONFIG_HPP
