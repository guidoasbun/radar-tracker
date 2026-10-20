#include "radar/sensor.hpp"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <random>
#include <vector>

#include "radar/types.hpp"

namespace radar {

std::vector<LabeledMeasurement> sense(const std::vector<TrueTarget>& targets,
                                      const SensorConfig& cfg, std::mt19937_64& rng) {
  assert(std::isfinite(cfg.sigma_m) && cfg.sigma_m >= 0.0 &&
         "sense: sigma_m must be finite and non-negative");

  // normal_distribution's second argument is a STANDARD DEVIATION (not variance).
  // sigma_m == 0 is the no-noise baseline: we must NOT construct
  // normal_distribution(0, 0) because libstdc++ asserts stddev > 0 — instead we
  // emit z == p directly and consume no randomness. For sigma_m > 0, one
  // distribution serves the whole call (the random state lives in rng, so reusing
  // the object does not affect reproducibility).
  const bool noisy = cfg.sigma_m > 0.0;
  std::normal_distribution<double> noise(0.0, noisy ? cfg.sigma_m : 1.0);

  std::vector<LabeledMeasurement> out;
  out.reserve(targets.size());
  for (const TrueTarget& t : targets) {
    assert(t.position.allFinite() && "sense: target position must be finite");

    // Two independent draws => n ~ N(0, sigma_m^2 I). Order (x then y) is fixed so
    // the consumed RNG sequence is deterministic for a given generator state.
    const double nx = noisy ? noise(rng) : 0.0;
    const double ny = noisy ? noise(rng) : 0.0;

    LabeledMeasurement lm;
    lm.measurement.position = t.position + Eigen::Vector2d{nx, ny};
    lm.origin = t.id;  // P_D = 1: every target produces exactly one measurement
    out.push_back(lm);
  }
  return out;
}

}  // namespace radar
