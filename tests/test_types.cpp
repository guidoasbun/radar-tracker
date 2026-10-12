// tests/test_types.cpp — unit tests for the shared data structures.
//
// The `types` module has no runtime logic, so the weight here is on value
// semantics and on the guarantees that encode the spec's anti-leakage
// requirement: the Measurement handed to the tracker must not be able to reveal
// a detection's origin.
//
// The authoritative origin-isolation asserts live in `radar/types.hpp` so they
// hold in every build, not only when tests are compiled. This file keeps one
// extra, test-only structured-binding guard (it counts members, catching an
// origin field inserted anywhere) plus the runtime behaviour checks.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "radar/types.hpp"

namespace {

// ---- Compile-time guarantees (fail the build, not just the test) -----------

// Measurement must be exactly {position}: a single member. If someone later
// adds a field *anywhere* (front, middle, or back), this structured binding
// stops compiling. It complements the type/size asserts in types.hpp, which
// catch widening `position` or swapping it for an origin-smuggling struct.
// (Not constexpr: an Eigen member's default ctor is not a constant expression,
// so this lives in an ordinary function whose body the compiler must check.)
[[maybe_unused]] inline void measurement_shape_guard() {
  const radar::Measurement m{};
  const auto& [p] = m;  // exactly one binding <=> exactly one member
  (void)p;
}

// Value-semantics guards. Eigen fixed-size members are NOT trivially copyable
// (their copy constructor is user-provided), so we assert the achievable
// guarantee: these remain ordinary copyable value types.
static_assert(std::is_copy_constructible_v<radar::Measurement>);
static_assert(std::is_copy_assignable_v<radar::Measurement>);
static_assert(std::is_copy_constructible_v<radar::TrackState>);
static_assert(std::is_copy_assignable_v<radar::TrackState>);
static_assert(std::is_copy_constructible_v<radar::LabeledMeasurement>);

}  // namespace

// ---- Defaults --------------------------------------------------------------

TEST(Types, MeasurementDefaults) {
  const radar::Measurement m{};
  EXPECT_TRUE(m.position.isApprox(Eigen::Vector2d::Zero()));
}

TEST(Types, LabeledMeasurementDefaultsToClutter) {
  const radar::LabeledMeasurement lm{};
  EXPECT_EQ(lm.origin, radar::kClutterOrigin);
  EXPECT_EQ(radar::kClutterOrigin, -1);
}

TEST(Types, ScanDefaults) {
  const radar::Scan s{};
  EXPECT_EQ(s.index, 0);
  EXPECT_DOUBLE_EQ(s.time_s, 0.0);
  EXPECT_TRUE(s.measurements.empty());
}

TEST(Types, TrackStateDefaultsToZeroStateAndNanCovariance) {
  const radar::TrackState ts{};
  EXPECT_TRUE(ts.x.isApprox(Eigen::Vector4d::Zero()));
  // P defaults to NaN so an uninitialised covariance fails loudly rather than
  // masquerading as a valid 1 m^2 identity (see types.hpp rationale).
  EXPECT_FALSE(ts.P.allFinite());
  for (int r = 0; r < ts.P.rows(); ++r) {
    for (int c = 0; c < ts.P.cols(); ++c) {
      EXPECT_TRUE(std::isnan(ts.P(r, c)));
    }
  }
}

// ---- TrackStatus wire/CSV values are pinned --------------------------------

TEST(Types, TrackStatusHasPinnedValues) {
  EXPECT_EQ(static_cast<std::uint8_t>(radar::TrackStatus::Tentative), 0);
  EXPECT_EQ(static_cast<std::uint8_t>(radar::TrackStatus::Confirmed), 1);
  EXPECT_EQ(static_cast<std::uint8_t>(radar::TrackStatus::Coasting), 2);
  EXPECT_EQ(static_cast<std::uint8_t>(radar::TrackStatus::Deleted), 3);
}

// ---- Construction / designated initializers --------------------------------

TEST(Types, MeasurementDesignatedInit) {
  const radar::Measurement m{.position = Eigen::Vector2d{10.0, -20.0}};
  EXPECT_DOUBLE_EQ(m.position.x(), 10.0);
  EXPECT_DOUBLE_EQ(m.position.y(), -20.0);
}

// ---- State-vector index constants ------------------------------------------

TEST(Types, StateIndexConstantsMapToLayout) {
  EXPECT_EQ(radar::kPx, 0);
  EXPECT_EQ(radar::kPy, 1);
  EXPECT_EQ(radar::kVx, 2);
  EXPECT_EQ(radar::kVy, 3);

  radar::TrackState ts{};
  ts.x(radar::kPx) = 100.0;
  ts.x(radar::kPy) = 200.0;
  ts.x(radar::kVx) = 30.0;
  ts.x(radar::kVy) = -40.0;

  // Reading back through the named indices lands in the intended slots,
  // in [px, py, vx, vy] order.
  EXPECT_DOUBLE_EQ(ts.x(0), 100.0);
  EXPECT_DOUBLE_EQ(ts.x(1), 200.0);
  EXPECT_DOUBLE_EQ(ts.x(2), 30.0);
  EXPECT_DOUBLE_EQ(ts.x(3), -40.0);
}

// ---- LabeledMeasurement handoff --------------------------------------------

TEST(Types, LabeledMeasurementExtractsPlainMeasurement) {
  radar::LabeledMeasurement lm{};
  lm.measurement.position = Eigen::Vector2d{5.0, 6.0};
  lm.origin = 7;  // a target id

  // The tracker-facing value is the plain Measurement, with no tag reachable.
  const radar::Measurement handed = lm.measurement;
  EXPECT_TRUE(handed.position.isApprox(Eigen::Vector2d{5.0, 6.0}));
}

// ---- Scan holds plain (unlabeled) measurements -----------------------------

TEST(Types, ScanCarriesPlainMeasurements) {
  radar::Scan s{};
  s.index = 3;
  s.time_s = 0.3;
  s.measurements.push_back(radar::Measurement{.position = Eigen::Vector2d{1.0, 2.0}});
  s.measurements.push_back(radar::Measurement{.position = Eigen::Vector2d{3.0, 4.0}});

  EXPECT_EQ(s.index, 3);
  EXPECT_EQ(s.measurements.size(), 2U);
  EXPECT_TRUE(s.measurements[1].position.isApprox(Eigen::Vector2d{3.0, 4.0}));
  // Compile-time: Scan's element type is Measurement, not LabeledMeasurement.
  static_assert(std::is_same_v<decltype(s.measurements)::value_type, radar::Measurement>);
}
