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
  // Validate EVERY input before drawing any noise, so a throw consumes no
  // randomness: otherwise an invalid target N would throw only after targets
  // 0..N-1 had already advanced `rng`, and a caller that catches and retries would
  // be on a different random stream — silently breaking "same seed, same output".
  if (!std::isfinite(cfg.sigma_m) || cfg.sigma_m < 0.0) {
    throw std::invalid_argument("sense: sigma_m must be finite and non-negative");
  }
  for (const TrueTarget& t : targets) {
    if (t.id < 0) {
      // A negative id collides with kClutterOrigin (-1): the scorer would count a
      // real target's track as a false (clutter) track. Reject it.
      throw std::invalid_argument("sense: target id must be non-negative");
    }
    if (!t.position.allFinite()) {
      throw std::invalid_argument("sense: target position must be finite");
    }
  }

  // sigma_m == 0 is the no-noise baseline: we must NOT construct
  // normal_distribution(0, 0) (the standard requires stddev > 0), so we emit z == p
  // directly and consume no randomness. For sigma_m > 0, one distribution serves the
  // whole call; output is a pure function of rng's state because the distribution is
  // constructed per call (libstdc++'s normal_distribution caches the second of each
  // generated pair inside the object, so reuse across calls would shift output).
  // normal_distribution's second argument is a STANDARD DEVIATION, not a variance.
  const bool noisy = cfg.sigma_m > 0.0;
  std::normal_distribution<double> noise(0.0, noisy ? cfg.sigma_m : 1.0);

  std::vector<LabeledMeasurement> out;
  out.reserve(targets.size());
  for (const TrueTarget& t : targets) {
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
