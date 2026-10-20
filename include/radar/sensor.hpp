#ifndef RADAR_SENSOR_HPP
#define RADAR_SENSOR_HPP

#include <Eigen/Core>
#include <random>
#include <vector>

#include "radar/types.hpp"

namespace radar {

/// Sensor parameters. M1 uses only the measurement-noise std-dev; the detection
/// probability, clutter rate, and surveillance half-width arrive with the full
/// model in M2 (see the deferred note below).
struct SensorConfig {
  double sigma_m{50.0};  ///< measurement-noise std-dev per axis (m); spec default
};

/// A true target the sensor can see on this scan: a stable id and its true
/// position. The id becomes the measurement's hidden origin. Velocity is
/// irrelevant to a position measurement, so it is not carried here.
struct TrueTarget {
  int id{0};                                          ///< >= 0; becomes the hidden origin
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};  ///< true (x east, y north) in meters
};

/// Noise-only sensor (M1 slice): emit exactly one LabeledMeasurement per target,
/// z = p + n with n ~ N(0, sigma_m^2 I) (independent axes), origin = target id.
/// P_D = 1 (every target detected), no clutter. The returned list is **shuffled**
/// so its order carries no information about origin (the `Scan` contract); the
/// hidden origin travels on each LabeledMeasurement, never on list position.
///
/// Randomness is caller-owned and explicit: `rng` is the single scenario
/// std::mt19937_64 (seeded from the scenario, default 42), passed by reference and
/// threaded through the whole run so results reproduce exactly. The sensor holds
/// no RNG state of its own. (A 0/1-element shuffle draws nothing from `rng`, so
/// single-target scenarios reproduce bit-for-bit.)
///
/// Returns LabeledMeasurement because the sensor is the one place that knows a
/// detection's origin; the simulator keeps these for truth/scoring and hands the
/// tracker only `labeled.measurement` (a plain Measurement, no origin — the
/// type-system isolation enforced in types.hpp).
///
/// Validates its inputs and throws `std::invalid_argument` if `cfg.sigma_m` is not
/// finite or is negative, or if any target has `id < 0` (would collide with the
/// clutter sentinel −1) or a non-finite position. A runtime check, not an assert,
/// because a bad `sigma_m` in a release build otherwise fails *open* to a silent
/// noise-free sensor. All inputs are validated **before any noise is drawn**, so a
/// throw leaves `rng` untouched — a caller that catches and retries stays on the
/// same random stream ("same seed, same output"). `io` should also reject these at
/// the scenario boundary (issue #5). `sense` runs once per scan, not a hot path.
[[nodiscard]] std::vector<LabeledMeasurement> sense(const std::vector<TrueTarget>& targets,
                                                    const SensorConfig& cfg, std::mt19937_64& rng);

// ---------------------------------------------------------------------------
// DEFERRED (M2): the full per-scan sensor model.
//
// M1 implements Gaussian-noise detection of in-view targets with P_D = 1, the
// hidden origin tag, and the order shuffle. The remaining spec steps land in M2,
// alongside the multi-target / clutter scenarios that exercise them:
//   * P_D < 1  — per target, a Bernoulli(P_D) trial; emit only on success.
//   * clutter  — draw Poisson(lambda) false alarms, placed uniformly in the
//                surveillance square (+/- L on each axis), interleaved with the
//                detections before the shuffle.
//   * culling  — targets outside the surveillance square produce no detection.
// Defaults for those knobs: P_D = 0.9, lambda = 2, L = 20000 m. They will extend
// SensorConfig; the `sense` signature (targets, cfg, rng&) is forward-compatible.
// ---------------------------------------------------------------------------

}  // namespace radar

#endif  // RADAR_SENSOR_HPP
