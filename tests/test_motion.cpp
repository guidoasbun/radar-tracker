// tests/test_motion.cpp — unit tests for the constant-velocity truth model.
//
// motion is deterministic and noise-free, so these are exact checks: the
// transition matrix F (compared against a literal matrix written from the spec,
// independently of the implementation's recipe), the CV law p += v*dt with
// velocity held, the equivalence propagate_cv(x, dt) == F*x, degenerate dt == 0,
// a multi-step closed-form straight-line trajectory with a direct path-length
// check, negative-dt round-trip, and (where asserts are live) the finite-input
// precondition.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <limits>

#include "radar/motion.hpp"
#include "radar/types.hpp"

namespace {

constexpr double kDt = 0.1;  // spec scan interval (s)

// M3: motion propagates a plain state vector by design (truth has no covariance);
// there must be no TrackState overload that would propagate P. Enforce it at
// compile time so the signature contract can't silently drift. A concept is used
// (not is_invocable_v<decltype(propagate_cv), ...>) because decltype of an
// overloaded function is ill-formed: the moment a second overload exists the
// assertion would fail to compile with "cannot resolve address of overloaded
// function" instead of reporting its message. The concept keeps working with
// overloads and matches the detail::HasOrigin pattern in types.hpp.
template <class S>
concept CvPropagatable = requires(const S& s) { radar::propagate_cv(s, 0.0); };

static_assert(CvPropagatable<Eigen::Vector4d>, "propagate_cv must accept a plain state vector");
static_assert(!CvPropagatable<radar::TrackState>,
              "propagate_cv must not accept a TrackState (P-propagation is kalman's job)");

// Propagate `x0` by `steps` CV steps of `kDt`. Factored out so the test bodies
// stay simple (and under the clang-tidy cognitive-complexity threshold).
Eigen::Vector4d step_n(const Eigen::Vector4d& x0, int steps) {
  Eigen::Vector4d x = x0;
  for (int i = 0; i < steps; ++i) {
    x = radar::propagate_cv(x, kDt);
  }
  return x;
}

}  // namespace

// ---- Transition matrix (independent literal from the spec) -----------------

TEST(Motion, CvTransitionMatrixEqualsSpecLiteral) {
  // Written out from the spec, NOT via Identity-plus-two-entries, so the test
  // checks the implementation rather than repeating its recipe.
  Eigen::Matrix4d expected;
  // clang-format off
  expected << 1.0, 0.0, kDt, 0.0,
              0.0, 1.0, 0.0, kDt,
              0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 1.0;
  // clang-format on
  // Exact: every entry of F is assigned exactly (1.0, 0.0, or dt), so demand ==.
  EXPECT_TRUE((radar::cv_transition_matrix(kDt).array() == expected.array()).all());
}

TEST(Motion, CvTransitionMatrixAtZeroIsIdentity) {
  EXPECT_TRUE(radar::cv_transition_matrix(0.0).isApprox(Eigen::Matrix4d::Identity()));
}

// ---- CV propagation law ----------------------------------------------------

TEST(Motion, PropagateCvAdvancesPositionLeavesVelocity) {
  // Aircraft at (1000, -500) m moving (250, 0) m/s (spec-range speed).
  const Eigen::Vector4d x0{1000.0, -500.0, 250.0, 0.0};
  const Eigen::Vector4d x1 = radar::propagate_cv(x0, kDt);

  EXPECT_DOUBLE_EQ(x1(radar::kPx), 1000.0 + (250.0 * kDt));  // 1025.0
  EXPECT_DOUBLE_EQ(x1(radar::kPy), -500.0);                  // v_y = 0, unchanged
  EXPECT_DOUBLE_EQ(x1(radar::kVx), 250.0);                   // velocity held
  EXPECT_DOUBLE_EQ(x1(radar::kVy), 0.0);
}

TEST(Motion, ZeroVelocityTargetDoesNotMove) {
  const Eigen::Vector4d x0{42.0, -7.0, 0.0, 0.0};
  // Exact: p + 0*v == p for finite inputs.
  EXPECT_TRUE((radar::propagate_cv(x0, kDt).array() == x0.array()).all());
}

TEST(Motion, PropagateCvEqualsMatrixTimesState) {
  const Eigen::Vector4d x0{3.0, 4.0, -12.0, 7.5};
  const Eigen::Vector4d via_fn = radar::propagate_cv(x0, kDt);
  const Eigen::Vector4d via_mat = radar::cv_transition_matrix(kDt) * x0;
  EXPECT_TRUE(via_fn.isApprox(via_mat));
}

TEST(Motion, ZeroDtIsNoOp) {
  const Eigen::Vector4d x0{3.0, 4.0, -12.0, 7.5};
  // Exact: F(0) is the identity, so this is bit-for-bit the input.
  EXPECT_TRUE((radar::propagate_cv(x0, 0.0).array() == x0.array()).all());
}

// ---- Multi-step, closed form, and direct path length -----------------------

TEST(Motion, MultiStepMatchesClosedFormAndPathLength) {
  const Eigen::Vector2d p0{0.0, 0.0};
  const Eigen::Vector2d v{150.0, -200.0};  // 250 m/s magnitude
  const Eigen::Vector4d x0{p0.x(), p0.y(), v.x(), v.y()};

  constexpr int kSteps = 600;  // 60 s run at dt = 0.1 s
  const double total_t = kSteps * kDt;
  const Eigen::Vector4d x = step_n(x0, kSteps);

  // Closed form: p = p0 + v * (N*dt); velocity unchanged throughout.
  EXPECT_NEAR(x(radar::kPx), p0.x() + (v.x() * total_t), 1e-6);
  EXPECT_NEAR(x(radar::kPy), p0.y() + (v.y() * total_t), 1e-6);
  EXPECT_DOUBLE_EQ(x(radar::kVx), v.x());
  EXPECT_DOUBLE_EQ(x(radar::kVy), v.y());

  // Spec contract: CV path length equals speed * time (direct check).
  const Eigen::Vector2d p_n{x(radar::kPx), x(radar::kPy)};
  EXPECT_NEAR((p_n - p0).norm(), v.norm() * total_t, 1e-6);
}

TEST(Motion, NStepsEqualOneBigStep) {
  const Eigen::Vector4d x0{0.0, 0.0, 150.0, -200.0};
  constexpr int kSteps = 600;
  const Eigen::Vector4d many = step_n(x0, kSteps);
  const Eigen::Vector4d one = radar::propagate_cv(x0, kSteps * kDt);
  EXPECT_NEAR(many(radar::kPx), one(radar::kPx), 1e-6);
  EXPECT_NEAR(many(radar::kPy), one(radar::kPy), 1e-6);
}

// ---- Negative dt is supported: round-trip returns to start -----------------

TEST(Motion, NegativeDtRoundTripReturnsToStart) {
  const Eigen::Vector4d x0{1000.0, 2000.0, 250.0, -100.0};
  const Eigen::Vector4d forward = radar::propagate_cv(x0, kDt);
  const Eigen::Vector4d back = radar::propagate_cv(forward, -kDt);
  // p + v*dt - v*dt == p; the back-step uses the same (unchanged) velocity.
  EXPECT_TRUE(back.isApprox(x0));
}

// ---- Determinism (exact) ---------------------------------------------------

TEST(Motion, DeterministicExact) {
  const Eigen::Vector4d x0{10.0, 20.0, 30.0, 40.0};
  const Eigen::Vector4d a = radar::propagate_cv(x0, kDt);
  const Eigen::Vector4d b = radar::propagate_cv(x0, kDt);
  // A determinism claim should be exact, not within a tolerance.
  EXPECT_TRUE((a.array() == b.array()).all());
}

// ---- Finite-input precondition (only where asserts are live) ---------------
//
// release/ci/ci-clang compile with -DNDEBUG (asserts off); debug/asan do not.
// Guard the death tests so they run exactly where the assert is active and are
// skipped where it is compiled out.
#ifndef NDEBUG
TEST(MotionDeathTest, NonFiniteDtAborts) {
  const Eigen::Vector4d x0{1.0, 2.0, 0.0, 0.0};
  EXPECT_DEATH((void)radar::propagate_cv(x0, std::numeric_limits<double>::quiet_NaN()),
               "dt must be finite");
  // isfinite rejects infinity as well as NaN.
  EXPECT_DEATH((void)radar::propagate_cv(x0, std::numeric_limits<double>::infinity()),
               "dt must be finite");
}

TEST(MotionDeathTest, NonFiniteStateAborts) {
  const Eigen::Vector4d nan_state{1.0, 2.0, std::numeric_limits<double>::quiet_NaN(), 0.0};
  EXPECT_DEATH((void)radar::propagate_cv(nan_state, kDt), "x must be finite");
  const Eigen::Vector4d inf_state{1.0, 2.0, std::numeric_limits<double>::infinity(), 0.0};
  EXPECT_DEATH((void)radar::propagate_cv(inf_state, kDt), "x must be finite");
}
#endif  // NDEBUG
