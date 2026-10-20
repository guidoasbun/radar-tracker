// tests/test_sensor.cpp — unit tests for the noise-only (M1) sensor.
//
// Randomness makes correctness partly statistical, so there are two kinds of
// check: exact determinism (same generator state => identical output, no
// tolerance) and distribution shape (mean/std/RMSE/covariance over a large
// fixed-seed sample, with tolerances). The fixed seed keeps the statistical test
// deterministic in CI. Note: std::normal_distribution is NOT identical across
// libstdc++/libc++, so determinism is asserted within one binary and the
// statistical checks use tolerances, never cross-implementation byte-equality.
//
// Input validation is a runtime throw (not an assert), so the EXPECT_THROW cases
// run in every build — ci/ci-clang included — not only where asserts are live.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "radar/sensor.hpp"
#include "radar/types.hpp"

namespace {

constexpr std::uint64_t kSeed = 42;  // spec default scenario seed

radar::TrueTarget target(int id, double x, double y) {
  return radar::TrueTarget{.id = id, .position = Eigen::Vector2d{x, y}};
}

// A deterministic generator for the tests. A constant seed is the POINT here —
// reproducible, non-flaky statistical and determinism checks — so the cert PRNG
// "predictable seed" warning is deliberately suppressed at this single factory.
std::mt19937_64 seeded(std::uint64_t s) {
  return std::mt19937_64{s};  // NOLINT(cert-msc32-c,cert-msc51-cpp): fixed test seed on purpose
}

// Find the measurement carrying `origin` (order is shuffled, so never index by
// position). Returns nullptr if absent.
const radar::LabeledMeasurement* by_origin(const std::vector<radar::LabeledMeasurement>& v,
                                           int origin) {
  const auto it = std::ranges::find_if(
      v, [origin](const radar::LabeledMeasurement& m) { return m.origin == origin; });
  return it == v.end() ? nullptr : &*it;
}

}  // namespace

// ---- Structure: one measurement per target, origin preserved (per element) --

TEST(Sensor, OneMeasurementPerTargetWithOrigin) {
  const std::vector<radar::TrueTarget> targets{target(7, 1000.0, -500.0), target(3, 0.0, 2000.0)};
  const radar::SensorConfig cfg{.sigma_m = 50.0};
  std::mt19937_64 rng = seeded(kSeed);

  const std::vector<radar::LabeledMeasurement> out = radar::sense(targets, cfg, rng);

  ASSERT_EQ(out.size(), targets.size());
  // Order is shuffled, so look up by origin rather than by position: both targets
  // are present exactly once, and the tracker-facing value is a plain Measurement.
  EXPECT_NE(by_origin(out, 7), nullptr);
  EXPECT_NE(by_origin(out, 3), nullptr);
  static_assert(std::is_same_v<decltype(out[0].measurement), radar::Measurement>);
}

TEST(Sensor, TwoTargetsNearOriginStillBothReturned) {
  // Both targets at the same point with zero noise: only the origin tag tells them
  // apart, which is the whole point of not leaking origin through list position.
  const std::vector<radar::TrueTarget> targets{target(0, 10.0, 10.0), target(1, 10.0, 10.0)};
  const radar::SensorConfig cfg{.sigma_m = 0.0};
  std::mt19937_64 rng = seeded(kSeed);

  const auto out = radar::sense(targets, cfg, rng);
  ASSERT_EQ(out.size(), 2U);
  const radar::LabeledMeasurement* m0 = by_origin(out, 0);
  const radar::LabeledMeasurement* m1 = by_origin(out, 1);
  ASSERT_NE(m0, nullptr);
  ASSERT_NE(m1, nullptr);
  EXPECT_TRUE(m0->measurement.position.isApprox(Eigen::Vector2d{10.0, 10.0}));
}

// ---- Exact determinism: same seed => identical; different seed => differs ----

TEST(Sensor, SameSeedProducesIdenticalMeasurements) {
  const std::vector<radar::TrueTarget> targets{target(1, 10.0, 20.0)};
  const radar::SensorConfig cfg{.sigma_m = 50.0};

  std::mt19937_64 rng_a = seeded(kSeed);
  std::mt19937_64 rng_b = seeded(kSeed);
  const auto a = radar::sense(targets, cfg, rng_a);
  const auto b = radar::sense(targets, cfg, rng_b);

  ASSERT_EQ(a.size(), 1U);
  ASSERT_EQ(b.size(), 1U);
  EXPECT_TRUE((a[0].measurement.position.array() == b[0].measurement.position.array()).all());
}

TEST(Sensor, DifferentSeedProducesDifferentMeasurements) {
  const std::vector<radar::TrueTarget> targets{target(1, 10.0, 20.0)};
  const radar::SensorConfig cfg{.sigma_m = 50.0};

  std::mt19937_64 rng_a = seeded(kSeed);
  std::mt19937_64 rng_b = seeded(kSeed + 1);
  const auto a = radar::sense(targets, cfg, rng_a);
  const auto b = radar::sense(targets, cfg, rng_b);

  EXPECT_FALSE(a[0].measurement.position.isApprox(b[0].measurement.position));
}

// ---- Degenerate: zero noise => z == p exactly ------------------------------

TEST(Sensor, ZeroSigmaYieldsExactTruth) {
  const Eigen::Vector2d p{1234.5, -6789.0};
  const std::vector<radar::TrueTarget> targets{radar::TrueTarget{.id = 5, .position = p}};
  const radar::SensorConfig cfg{.sigma_m = 0.0};
  std::mt19937_64 rng = seeded(kSeed);

  const auto out = radar::sense(targets, cfg, rng);
  EXPECT_TRUE((out[0].measurement.position.array() == p.array()).all());
}

// ---- Statistical: mean, std, RMSE, and x/y independence (covariance ~ 0) ----

TEST(Sensor, NoiseMatchesGaussianStatistics) {
  constexpr double kSigma = 50.0;
  constexpr int kN = 200000;
  const Eigen::Vector2d p{100.0, -200.0};
  const radar::SensorConfig cfg{.sigma_m = kSigma};
  std::mt19937_64 rng = seeded(kSeed);

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sumsq_x = 0.0;
  double sumsq_y = 0.0;
  double sum_xy = 0.0;  // for the x/y covariance (independence) check
  double sumsq_r = 0.0;
  const std::vector<radar::TrueTarget> one{radar::TrueTarget{.id = 0, .position = p}};
  for (int i = 0; i < kN; ++i) {
    const Eigen::Vector2d e = radar::sense(one, cfg, rng)[0].measurement.position - p;
    sum_x += e.x();
    sum_y += e.y();
    sumsq_x += e.x() * e.x();
    sumsq_y += e.y() * e.y();
    sum_xy += e.x() * e.y();
    sumsq_r += e.squaredNorm();
  }

  EXPECT_NEAR(sum_x / kN, 0.0, 1.0);
  EXPECT_NEAR(sum_y / kN, 0.0, 1.0);
  EXPECT_NEAR(std::sqrt(sumsq_x / kN), kSigma, 1.0);
  EXPECT_NEAR(std::sqrt(sumsq_y / kN), kSigma, 1.0);
  EXPECT_NEAR(std::sqrt(sumsq_r / kN), kSigma * std::numbers::sqrt2, 1.5);

  // Independence: Cov(x, y) must be ~ 0. A correlated-noise bug (e.g. ny = nx)
  // leaves mean/std/RMSE unchanged but drives this toward sigma^2, so this is the
  // check that actually tests AC-1's "independent axes". SE ~ sigma^2/sqrt(N) ~ 5.6;
  // tolerance 0.02*sigma^2 = 50 is ~9 SE.
  EXPECT_NEAR(sum_xy / kN, 0.0, 0.02 * kSigma * kSigma);
}

// ---- Input validation throws (runs in EVERY build, not just asserts) --------

TEST(Sensor, NegativeSigmaThrows) {
  std::mt19937_64 rng = seeded(kSeed);
  EXPECT_THROW((void)radar::sense({target(0, 0.0, 0.0)}, radar::SensorConfig{.sigma_m = -1.0}, rng),
               std::invalid_argument);
}

TEST(Sensor, NonFiniteSigmaThrows) {
  std::mt19937_64 rng = seeded(kSeed);
  const radar::SensorConfig nan_cfg{.sigma_m = std::numeric_limits<double>::quiet_NaN()};
  const radar::SensorConfig inf_cfg{.sigma_m = std::numeric_limits<double>::infinity()};
  EXPECT_THROW((void)radar::sense({target(0, 0.0, 0.0)}, nan_cfg, rng), std::invalid_argument);
  EXPECT_THROW((void)radar::sense({target(0, 0.0, 0.0)}, inf_cfg, rng), std::invalid_argument);
}

TEST(Sensor, NegativeTargetIdThrows) {
  // id == -1 would collide with kClutterOrigin, labeling a real target as clutter.
  std::mt19937_64 rng = seeded(kSeed);
  EXPECT_THROW(
      (void)radar::sense({target(-1, 0.0, 0.0)}, radar::SensorConfig{.sigma_m = 50.0}, rng),
      std::invalid_argument);
}

TEST(Sensor, NonFinitePositionThrows) {
  std::mt19937_64 rng = seeded(kSeed);
  const auto nan_t = target(0, std::numeric_limits<double>::quiet_NaN(), 0.0);
  const auto inf_t = target(0, std::numeric_limits<double>::infinity(), 0.0);
  EXPECT_THROW((void)radar::sense({nan_t}, radar::SensorConfig{.sigma_m = 50.0}, rng),
               std::invalid_argument);
  EXPECT_THROW((void)radar::sense({inf_t}, radar::SensorConfig{.sigma_m = 50.0}, rng),
               std::invalid_argument);
}
