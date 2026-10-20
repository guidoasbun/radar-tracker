// tests/test_kalman.cpp — unit tests for the CV linear Kalman filter.
//
// This is the keystone module, so the weight is on two things:
//   * an INDEPENDENT check of predict (hand-computed, never calling
//     cv_transition_matrix) so a shared-F bug cannot hide (issue #6 / ADR-0006);
//   * the Joseph update staying symmetric and positive semi-definite.
// The exhaustive statistical battery (NEES/NIS) is Phase K; AC-6 here is a smoke test.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

#include "radar/kalman.hpp"
#include "radar/motion.hpp"
#include "radar/sensor.hpp"
#include "radar/types.hpp"

namespace {

radar::TrackState state_of(const Eigen::Vector4d& x, const Eigen::Matrix4d& p) {
  radar::TrackState s;
  s.x = x;
  s.P = p;
  return s;
}

radar::Measurement meas(double x, double y) {
  return radar::Measurement{.position = Eigen::Vector2d{x, y}};
}

bool is_psd(const Eigen::Matrix4d& m) {
  // Symmetric eigensolver; smallest eigenvalue >= 0 (within rounding) => PSD.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(m);
  return es.eigenvalues().minCoeff() > -1e-9;
}

}  // namespace

// ---- Model matrices vs a LITERAL spec (independent of the implementation) ---

TEST(Kalman, ProcessNoiseMatchesSpecLiteral) {
  constexpr double kDt = 0.5;
  constexpr double kSa = 2.0;
  const Eigen::Matrix4d q = radar::process_noise(kDt, kSa);

  // Written out from the spec formula with dt=0.5, sigma_a=2 (so sigma_a^2 = 4):
  //   dt^2=0.25, dt^3=0.125, dt^4=0.0625; entries scaled by 4.
  const double a = 4.0 * (0.0625 / 4.0);  // dt^4/4  -> 0.0625
  const double b = 4.0 * (0.125 / 2.0);   // dt^3/2  -> 0.25
  const double c = 4.0 * 0.25;            // dt^2    -> 1.0
  Eigen::Matrix4d expected;
  // clang-format off
  expected << a,   0.0, b,   0.0,
              0.0, a,   0.0, b,
              b,   0.0, c,   0.0,
              0.0, b,   0.0, c;
  // clang-format on
  EXPECT_TRUE(q.isApprox(expected));
  EXPECT_TRUE(q.isApprox(q.transpose()));  // symmetric
}

TEST(Kalman, MeasurementMatrixAndNoise) {
  const radar::MeasurementMatrix h = radar::measurement_matrix();
  radar::MeasurementMatrix h_expected;
  h_expected << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  EXPECT_TRUE(h.isApprox(h_expected));

  const Eigen::Matrix2d r = radar::measurement_noise(50.0);
  EXPECT_DOUBLE_EQ(r(0, 0), 2500.0);
  EXPECT_DOUBLE_EQ(r(1, 1), 2500.0);
  EXPECT_DOUBLE_EQ(r(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(r(1, 0), 0.0);
}

// ---- Predict, HAND-COMPUTED (issue #6: no cv_transition_matrix call) --------

TEST(Kalman, PredictAdvancesStateByHandComputedValues) {
  const radar::KalmanConfig cfg{.dt = 0.5, .sigma_a = 0.0, .sigma_m = 50.0};
  // sigma_a = 0 => Q = 0, so P_pred = F P F^T exactly; lets us check the mean cleanly.
  const Eigen::Vector4d x0{100.0, 200.0, 10.0, -20.0};
  const radar::TrackState s = state_of(x0, Eigen::Matrix4d::Identity());

  const radar::TrackState out = radar::predict(s, cfg);

  // Hand-computed from p' = p + v*dt, v' = v (dt = 0.5), NOT via cv_transition_matrix:
  //   px = 100 + 10*0.5 = 105;  py = 200 + (-20)*0.5 = 190;  vx = 10;  vy = -20.
  EXPECT_DOUBLE_EQ(out.x(radar::kPx), 105.0);
  EXPECT_DOUBLE_EQ(out.x(radar::kPy), 190.0);
  EXPECT_DOUBLE_EQ(out.x(radar::kVx), 10.0);
  EXPECT_DOUBLE_EQ(out.x(radar::kVy), -20.0);
}

TEST(Kalman, PredictGrowsCovarianceByProcessNoise) {
  const radar::KalmanConfig cfg{.dt = 0.1, .sigma_a = 3.0, .sigma_m = 50.0};
  const radar::TrackState s = state_of(Eigen::Vector4d::Zero(), Eigen::Matrix4d::Identity());

  const radar::TrackState out = radar::predict(s, cfg);
  EXPECT_TRUE(out.P.isApprox(out.P.transpose()));  // symmetric
  // Process noise strictly increases the velocity-block variance.
  EXPECT_GT(out.P(radar::kVx, radar::kVx), s.P(radar::kVx, radar::kVx));
  EXPECT_GT(out.P(radar::kVy, radar::kVy), s.P(radar::kVy, radar::kVy));
}

// ---- Track initialization --------------------------------------------------

TEST(Kalman, InitiateTrackExactStateAndCovariance) {
  const radar::KalmanConfig cfg{.dt = 0.1, .sigma_a = 1.0, .sigma_m = 50.0};
  const radar::TrackState s = radar::initiate_track(meas(1000.0, -2000.0), cfg);

  EXPECT_DOUBLE_EQ(s.x(radar::kPx), 1000.0);
  EXPECT_DOUBLE_EQ(s.x(radar::kPy), -2000.0);
  EXPECT_DOUBLE_EQ(s.x(radar::kVx), 0.0);
  EXPECT_DOUBLE_EQ(s.x(radar::kVy), 0.0);
  EXPECT_DOUBLE_EQ(s.P(radar::kPx, radar::kPx), 2500.0);  // sigma_m^2
  EXPECT_DOUBLE_EQ(s.P(radar::kPy, radar::kPy), 2500.0);
  EXPECT_DOUBLE_EQ(s.P(radar::kVx, radar::kVx), 150.0 * 150.0);  // kSigmaV0^2
  EXPECT_DOUBLE_EQ(s.P(radar::kVy, radar::kVy), 150.0 * 150.0);
}

// ---- Innovation, S, gate distance ------------------------------------------

TEST(Kalman, InnovationIsMeasurementMinusPredictedPosition) {
  const radar::TrackState s =
      state_of(Eigen::Vector4d{10.0, 20.0, 1.0, 2.0}, Eigen::Matrix4d::Identity());
  const Eigen::Vector2d y = radar::innovation(s, meas(13.0, 24.0));
  EXPECT_DOUBLE_EQ(y.x(), 3.0);  // 13 - 10
  EXPECT_DOUBLE_EQ(y.y(), 4.0);  // 24 - 20
}

TEST(Kalman, InnovationCovarianceIsPositionBlockPlusR) {
  const radar::KalmanConfig cfg{.dt = 0.1, .sigma_a = 1.0, .sigma_m = 10.0};
  Eigen::Matrix4d p = Eigen::Matrix4d::Zero();
  p(radar::kPx, radar::kPx) = 5.0;
  p(radar::kPy, radar::kPy) = 7.0;
  const radar::TrackState s = state_of(Eigen::Vector4d::Zero(), p);

  const Eigen::Matrix2d scov = radar::innovation_covariance(s, cfg);
  EXPECT_DOUBLE_EQ(scov(0, 0), 5.0 + 100.0);  // Ppos + sigma_m^2
  EXPECT_DOUBLE_EQ(scov(1, 1), 7.0 + 100.0);
}

TEST(Kalman, GateDistanceKnownValues) {
  // Zero innovation => zero distance.
  EXPECT_DOUBLE_EQ(radar::gate_distance(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity()), 0.0);
  // S = sigma^2 I => d^2 = ||y||^2 / sigma^2. y=(3,4), sigma^2=4 => 25/4 = 6.25.
  const Eigen::Vector2d y{3.0, 4.0};
  const Eigen::Matrix2d s = 4.0 * Eigen::Matrix2d::Identity();
  EXPECT_DOUBLE_EQ(radar::gate_distance(y, s), 6.25);
}

// ---- Joseph update: fixed point, symmetry, PSD, trace drop -----------------

TEST(Kalman, UpdateWithZeroInnovationLeavesMeanAndShrinksCovariance) {
  const radar::KalmanConfig cfg{.dt = 0.1, .sigma_a = 1.0, .sigma_m = 50.0};
  // Prediction exactly at the measurement => innovation 0 => mean unchanged.
  const Eigen::Vector4d x0{500.0, -300.0, 0.0, 0.0};
  const Eigen::Matrix4d p = Eigen::Matrix4d::Identity() * 1.0e4;
  const radar::TrackState prior = state_of(x0, p);

  const radar::TrackState post = radar::update(prior, meas(500.0, -300.0), cfg);

  EXPECT_TRUE(post.x.isApprox(x0));                  // zero innovation leaves the mean put
  EXPECT_TRUE(post.P.isApprox(post.P.transpose()));  // symmetric (symmetrized)
  EXPECT_TRUE(is_psd(post.P));
  // An update cannot increase position uncertainty.
  const double tr_prior = prior.P.topLeftCorner<2, 2>().trace();
  const double tr_post = post.P.topLeftCorner<2, 2>().trace();
  EXPECT_LT(tr_post, tr_prior);
}

TEST(Kalman, JosephMatchesTextbookFormOnBenignCase) {
  const radar::KalmanConfig cfg{.dt = 0.1, .sigma_a = 1.0, .sigma_m = 20.0};
  Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
  p(radar::kPx, radar::kPx) = 400.0;
  p(radar::kPy, radar::kPy) = 400.0;
  const radar::TrackState prior = state_of(Eigen::Vector4d{10.0, 20.0, 1.0, -1.0}, p);

  const radar::TrackState post = radar::update(prior, meas(12.0, 19.0), cfg);

  // Recompute the textbook (I - K H) P form independently and compare.
  const radar::MeasurementMatrix h = radar::measurement_matrix();
  const Eigen::Matrix2d r = radar::measurement_noise(cfg.sigma_m);
  const Eigen::Matrix2d s = prior.P.topLeftCorner<2, 2>() + r;
  const Eigen::Matrix<double, 4, 2> k = prior.P * h.transpose() * s.inverse();
  const Eigen::Matrix4d p_textbook = (Eigen::Matrix4d::Identity() - k * h) * prior.P;
  EXPECT_TRUE(post.P.isApprox(p_textbook, 1e-9));
}

// ---- Convergence smoke (AC-6): the filter actually tracks ------------------

TEST(Kalman, ConvergesOnStraightLineTruth) {
  const radar::KalmanConfig cfg{.dt = 0.1, .sigma_a = 1.0, .sigma_m = 50.0};
  const radar::SensorConfig scfg{.sigma_m = 50.0};
  std::mt19937_64 rng{42};  // NOLINT(cert-msc32-c,cert-msc51-cpp): fixed test seed

  // Truth: CV target crossing the field.
  Eigen::Vector4d truth{0.0, 0.0, 200.0, -100.0};
  const radar::TrueTarget t0{.id = 0, .position = truth.head<2>()};
  radar::TrackState est = radar::initiate_track(radar::sense({t0}, scfg, rng)[0].measurement, cfg);

  double last_err = 0.0;
  for (int k = 0; k < 60; ++k) {
    truth = radar::propagate_cv(truth, cfg.dt);
    const radar::TrueTarget t{.id = 0, .position = truth.head<2>()};
    const radar::Measurement z = radar::sense({t}, scfg, rng)[0].measurement;
    est = radar::update(radar::predict(est, cfg), z, cfg);
    last_err = (est.x.head<2>() - truth.head<2>()).norm();
  }
  // After convergence the estimate should sit well inside the raw noise (~71 m RMS),
  // comfortably under a loose smoke bound. (Phase K does the rigorous statistics.)
  EXPECT_LT(last_err, 100.0);
}

// ---- Validation throws (every build) ---------------------------------------

TEST(Kalman, InvalidConfigThrows) {
  const radar::TrackState s = state_of(Eigen::Vector4d::Zero(), Eigen::Matrix4d::Identity());
  EXPECT_THROW((void)radar::predict(s, {.dt = 0.0, .sigma_a = 1.0, .sigma_m = 50.0}),
               std::invalid_argument);
  EXPECT_THROW(
      (void)radar::predict(
          s, {.dt = std::numeric_limits<double>::quiet_NaN(), .sigma_a = 1.0, .sigma_m = 50.0}),
      std::invalid_argument);
  EXPECT_THROW((void)radar::process_noise(0.1, -1.0), std::invalid_argument);
  EXPECT_THROW((void)radar::measurement_noise(-1.0), std::invalid_argument);
}

TEST(Kalman, NonFiniteMeasurementThrows) {
  const radar::KalmanConfig cfg{.dt = 0.1, .sigma_a = 1.0, .sigma_m = 50.0};
  const radar::TrackState s = state_of(Eigen::Vector4d::Zero(), Eigen::Matrix4d::Identity());
  EXPECT_THROW((void)radar::update(s, meas(std::numeric_limits<double>::infinity(), 0.0), cfg),
               std::invalid_argument);
}

TEST(Kalman, DegenerateGateCovarianceThrows) {
  const Eigen::Vector2d y{1.0, 1.0};
  EXPECT_THROW((void)radar::gate_distance(y, Eigen::Matrix2d::Zero()), std::invalid_argument);
}
