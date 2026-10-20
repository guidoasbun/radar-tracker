// tests/test_sensor.cpp — unit tests for the noise-only (M1) sensor.
//
// Randomness makes correctness partly statistical, so there are two kinds of
// check: exact determinism (same generator state => identical output, no
// tolerance) and distribution shape (mean/std/RMSE over a large fixed-seed
// sample, with tolerances). The fixed seed keeps the statistical test
// deterministic in CI. Note: std::normal_distribution is NOT identical across
// libstdc++/libc++, so determinism is asserted within one binary and the
// statistical checks use tolerances, never cross-implementation byte-equality.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
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

}  // namespace

// ---- Structure: one measurement per target, origin preserved ---------------

TEST(Sensor, OneMeasurementPerTargetWithOrigin) {
  const std::vector<radar::TrueTarget> targets{target(7, 1000.0, -500.0), target(3, 0.0, 2000.0)};
  const radar::SensorConfig cfg{.sigma_m = 50.0};
  std::mt19937_64 rng = seeded(kSeed);

  const std::vector<radar::LabeledMeasurement> out = radar::sense(targets, cfg, rng);

  ASSERT_EQ(out.size(), targets.size());
  // P_D = 1, no shuffle: order follows the input and origin == target id.
  EXPECT_EQ(out[0].origin, 7);
  EXPECT_EQ(out[1].origin, 3);
  // The tracker-facing value is a plain Measurement (position only).
  static_assert(std::is_same_v<decltype(out[0].measurement), radar::Measurement>);
}

// ---- Exact determinism: same seed => identical; different seed => differs ---

TEST(Sensor, SameSeedProducesIdenticalMeasurements) {
  const std::vector<radar::TrueTarget> targets{target(1, 10.0, 20.0)};
  const radar::SensorConfig cfg{.sigma_m = 50.0};

  std::mt19937_64 rng_a = seeded(kSeed);
  std::mt19937_64 rng_b = seeded(kSeed);
  const auto a = radar::sense(targets, cfg, rng_a);
  const auto b = radar::sense(targets, cfg, rng_b);

  ASSERT_EQ(a.size(), 1U);
  ASSERT_EQ(b.size(), 1U);
  // Bit-for-bit: a determinism claim should not use a tolerance.
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

// ---- Statistical: mean ~ 0, std ~ sigma_m, RMSE ~ sigma_m*sqrt(2) ----------

TEST(Sensor, NoiseMatchesGaussianStatistics) {
  constexpr double kSigma = 50.0;
  constexpr int kN = 200000;
  const Eigen::Vector2d p{100.0, -200.0};
  const radar::SensorConfig cfg{.sigma_m = kSigma};
  std::mt19937_64 rng = seeded(kSeed);

  // Accumulate per-axis error and squared error, plus squared range for RMSE.
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sumsq_x = 0.0;
  double sumsq_y = 0.0;
  double sumsq_r = 0.0;  // ||z - p||^2
  const std::vector<radar::TrueTarget> one{radar::TrueTarget{.id = 0, .position = p}};
  for (int i = 0; i < kN; ++i) {
    const Eigen::Vector2d e = radar::sense(one, cfg, rng)[0].measurement.position - p;
    sum_x += e.x();
    sum_y += e.y();
    sumsq_x += e.x() * e.x();
    sumsq_y += e.y() * e.y();
    sumsq_r += e.squaredNorm();
  }

  const double mean_x = sum_x / kN;
  const double mean_y = sum_y / kN;
  const double std_x = std::sqrt(sumsq_x / kN);
  const double std_y = std::sqrt(sumsq_y / kN);
  const double rmse = std::sqrt(sumsq_r / kN);

  // Tolerances: loose enough for this fixed seed's sample, tight enough to catch
  // a wrong mean or a std/variance mix-up.
  EXPECT_NEAR(mean_x, 0.0, 1.0);
  EXPECT_NEAR(mean_y, 0.0, 1.0);
  EXPECT_NEAR(std_x, kSigma, 1.0);
  EXPECT_NEAR(std_y, kSigma, 1.0);
  // Spec: raw measurement RMSE ~ sigma_m*sqrt(2) ~ 70.7 m at sigma_m = 50.
  EXPECT_NEAR(rmse, kSigma * std::numbers::sqrt2, 1.5);
}

// ---- Finite-input precondition (only where asserts are live) ---------------
#ifndef NDEBUG
TEST(SensorDeathTest, NonFinitePositionAborts) {
  const std::vector<radar::TrueTarget> bad{radar::TrueTarget{
      .id = 0, .position = Eigen::Vector2d{std::numeric_limits<double>::quiet_NaN(), 0.0}}};
  const radar::SensorConfig cfg{.sigma_m = 50.0};
  std::mt19937_64 rng = seeded(kSeed);
  EXPECT_DEATH((void)radar::sense(bad, cfg, rng), "position must be finite");
}

TEST(SensorDeathTest, NegativeSigmaAborts) {
  const std::vector<radar::TrueTarget> targets{radar::TrueTarget{.id = 0}};
  const radar::SensorConfig cfg{.sigma_m = -1.0};
  std::mt19937_64 rng = seeded(kSeed);
  EXPECT_DEATH((void)radar::sense(targets, cfg, rng), "sigma_m must be finite and non-negative");
}
#endif  // NDEBUG
