#include "radar/kalman.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

#include "radar/linalg.hpp"
#include "radar/motion.hpp"
#include "radar/types.hpp"

namespace radar {

namespace {

// Validation helpers. Invalid filter input is a runtime throw, not an assert: a NaN
// dt or measurement silently produces NaN states that poison every track and the
// Phase-K NEES/NIS battery. These run once per call, not on a hot path at M1 scale.

void require_dt(double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("kalman: dt must be finite and positive");
  }
}

void require_sigma(double sigma, const char* what) {
  if (!std::isfinite(sigma) || sigma < 0.0) {
    throw std::invalid_argument(what);
  }
}

void require_finite_state(const TrackState& s) {
  if (!s.x.allFinite() || !s.P.allFinite()) {
    throw std::invalid_argument("kalman: state (x, P) must be finite");
  }
}

void require_finite_measurement(const Measurement& z) {
  if (!z.position.allFinite()) {
    throw std::invalid_argument("kalman: measurement position must be finite");
  }
}

}  // namespace

// dt and sigma_a are distinct physical quantities (seconds vs m/s^2); a swap is
// caught immediately by the literal-matrix test (they produce very different Q).
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Eigen::Matrix4d process_noise(double dt, double sigma_a) {
  require_dt(dt);
  require_sigma(sigma_a, "kalman: sigma_a must be finite and non-negative");

  // Discrete white-noise-acceleration model (spec). q scales the whole matrix by
  // sigma_a^2 (sigma_a is a std-dev, so it is squared here).
  const double t2 = dt * dt;
  const double t3 = t2 * dt;
  const double t4 = t3 * dt;
  const double q = sigma_a * sigma_a;

  Eigen::Matrix4d result = Eigen::Matrix4d::Zero();
  result(kPx, kPx) = t4 / 4.0;
  result(kPx, kVx) = t3 / 2.0;
  result(kPy, kPy) = t4 / 4.0;
  result(kPy, kVy) = t3 / 2.0;
  result(kVx, kPx) = t3 / 2.0;
  result(kVx, kVx) = t2;
  result(kVy, kPy) = t3 / 2.0;
  result(kVy, kVy) = t2;
  return q * result;
}

MeasurementMatrix measurement_matrix() noexcept {
  MeasurementMatrix h = MeasurementMatrix::Zero();
  h(0, kPx) = 1.0;
  h(1, kPy) = 1.0;
  return h;
}

Eigen::Matrix2d measurement_noise(double sigma_m) {
  require_sigma(sigma_m, "kalman: sigma_m must be finite and non-negative");
  Eigen::Matrix2d r = Eigen::Matrix2d::Zero();
  r(0, 0) = sigma_m * sigma_m;
  r(1, 1) = sigma_m * sigma_m;
  return r;
}

TrackState predict(const TrackState& state, const KalmanConfig& cfg) {
  require_dt(cfg.dt);
  require_sigma(cfg.sigma_a, "kalman: sigma_a must be finite and non-negative");
  require_finite_state(state);

  // F is reused from motion (ADR-0006): one definition of the CV model shared with
  // truth propagation. The common-mode risk is bought off by test_kalman verifying
  // predict against HAND-COMPUTED values, never another cv_transition_matrix call.
  const Eigen::Matrix4d f = cv_transition_matrix(cfg.dt);

  TrackState out;
  out.x = f * state.x;
  out.P = f * state.P * f.transpose() + process_noise(cfg.dt, cfg.sigma_a);
  return out;
}

TrackState initiate_track(const Measurement& z, const KalmanConfig& cfg) {
  require_sigma(cfg.sigma_m, "kalman: sigma_m must be finite and non-negative");
  require_finite_measurement(z);

  TrackState out;
  out.x = Eigen::Vector4d::Zero();
  out.x(kPx) = z.position.x();
  out.x(kPy) = z.position.y();
  // Velocity starts at 0 with a wide variance (kSigmaV0) so the first updates pull it in.
  out.P = Eigen::Matrix4d::Zero();
  out.P(kPx, kPx) = cfg.sigma_m * cfg.sigma_m;
  out.P(kPy, kPy) = cfg.sigma_m * cfg.sigma_m;
  out.P(kVx, kVx) = kSigmaV0 * kSigmaV0;
  out.P(kVy, kVy) = kSigmaV0 * kSigmaV0;
  return out;
}

Eigen::Vector2d innovation(const TrackState& state, const Measurement& z) {
  require_finite_state(state);
  require_finite_measurement(z);
  // H*x is the position block of the state.
  return z.position - state.x.head<2>();
}

Eigen::Matrix2d innovation_covariance(const TrackState& state, const KalmanConfig& cfg) {
  require_finite_state(state);
  // H*P*H^T is the top-left 2x2 (position) block of P.
  return state.P.topLeftCorner<2, 2>() + measurement_noise(cfg.sigma_m);
}

double gate_distance(const Eigen::Vector2d& y, const Eigen::Matrix2d& S) {
  if (!y.allFinite() || !S.allFinite()) {
    throw std::invalid_argument("kalman: innovation and S must be finite");
  }
  // S must be positive-definite for a meaningful Mahalanobis distance; a degenerate S
  // would yield a NaN/Inf d^2 that silently passes or fails the gate.
  const double det = S.determinant();
  if (!(det > 0.0)) {
    throw std::invalid_argument("kalman: S must be positive-definite");
  }
  return (y.transpose() * S.inverse() * y).value();
}

TrackState update(const TrackState& state, const Measurement& z, const KalmanConfig& cfg) {
  require_sigma(cfg.sigma_m, "kalman: sigma_m must be finite and non-negative");
  require_finite_state(state);
  require_finite_measurement(z);

  const MeasurementMatrix h = measurement_matrix();
  const Eigen::Matrix2d r = measurement_noise(cfg.sigma_m);
  const Eigen::Vector2d y = z.position - state.x.head<2>();
  const Eigen::Matrix2d s = state.P.topLeftCorner<2, 2>() + r;

  const double det = s.determinant();
  if (!(det > 0.0)) {
    throw std::invalid_argument("kalman: S must be positive-definite");
  }
  const Eigen::Matrix<double, 4, 2> k = state.P * h.transpose() * s.inverse();

  TrackState out;
  out.x = state.x + k * y;
  // Joseph form: keeps P symmetric and PSD under rounding where (I - K*H)*P does not.
  const Eigen::Matrix4d i_kh = Eigen::Matrix4d::Identity() - k * h;
  out.P = i_kh * state.P * i_kh.transpose() + k * r * k.transpose();
  out.P = symmetrize(out.P);  // cheap extra guard against rounding asymmetry (spec)
  return out;
}

}  // namespace radar
