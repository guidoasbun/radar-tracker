#include "radar/sensor.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include "radar/types.hpp"

namespace radar {

std::vector<LabeledMeasurement> sense(const std::vector<TrueTarget>& targets,
                                      const SensorConfig& cfg, std::mt19937_64& rng) {
  // Runtime validation, not assert: asserts are compiled out in release/ci
  // (-DNDEBUG), and a bad sigma_m there fails OPEN — sigma_m < 0 or NaN would make
  // `noisy` false and silently emit a perfect, noise-free sensor (plausible-looking
  // measurements, inflating every tracking result). sense() runs once per scan, not
  // in a hot loop, so a real check is free. It is not noexcept; io should also
  // reject these at the scenario boundary (see issue #5).
  if (!std::isfinite(cfg.sigma_m) || cfg.sigma_m < 0.0) {
    throw std::invalid_argument("sense: sigma_m must be finite and non-negative");
  }

  // sigma_m == 0 is the no-noise baseline: we must NOT construct
  // normal_distribution(0, 0) (the standard requires stddev > 0), so we emit z == p
  // directly and consume no randomness. For sigma_m > 0, one distribution serves the
  // whole call (the random state lives in rng). normal_distribution's second argument
  // is a STANDARD DEVIATION, not a variance.
  const bool noisy = cfg.sigma_m > 0.0;
  std::normal_distribution<double> noise(0.0, noisy ? cfg.sigma_m : 1.0);

  std::vector<LabeledMeasurement> out;
  out.reserve(targets.size());
  for (const TrueTarget& t : targets) {
    if (t.id < 0) {
      // A negative id collides with kClutterOrigin (-1): the scorer would count a
      // real target's track as a false (clutter) track. Reject it.
      throw std::invalid_argument("sense: target id must be non-negative");
    }
    if (!t.position.allFinite()) {
      throw std::invalid_argument("sense: target position must be finite");
    }

    // Two independent draws => n ~ N(0, sigma_m^2 I). Fixed order (x then y) keeps
    // the consumed RNG sequence deterministic for a given generator state.
    const double nx = noisy ? noise(rng) : 0.0;
    const double ny = noisy ? noise(rng) : 0.0;

    LabeledMeasurement lm;
    lm.measurement.position = t.position + Eigen::Vector2d{nx, ny};
    lm.origin = t.id;  // P_D = 1: every target produces exactly one measurement
    out.push_back(lm);
  }

  // Shuffle so list order carries no information about origin (the Scan contract:
  // "already shuffled"). With 0 or 1 element std::shuffle draws nothing from rng, so
  // single-target scenarios (single.json) reproduce bit-for-bit as before; the
  // hidden origin travels on each LabeledMeasurement, not on its position in the list.
  std::shuffle(out.begin(), out.end(), rng);
  return out;
}

}  // namespace radar
