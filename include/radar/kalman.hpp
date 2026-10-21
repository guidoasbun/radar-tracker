#ifndef RADAR_KALMAN_HPP
#define RADAR_KALMAN_HPP

#include <Eigen/Dense>

#include "radar/types.hpp"

namespace radar {

/// Kalman filter tuning. `dt` and `sigma_m` come from the scenario; `sigma_a` is the
/// one genuinely *tuned* parameter — the default is the spec's 5 m/s² and is refined
/// in M2 against the NIS target. `sigma_m` must be positive and match the sensor's.
struct KalmanConfig {
  double dt{0.1};        ///< scan interval (s)
  double sigma_a{5.0};   ///< process-noise acceleration std-dev (m/s^2); spec default
  double sigma_m{50.0};  ///< measurement noise std-dev per axis (m), > 0; matches the sensor
};

/// Chi-squared gate threshold: 2 DOF, 99% — a measurement is a gating candidate for a
/// track when its Mahalanobis distance d^2 <= kGate99. Provided for callers
/// (association/tracker, M2); `kalman` only computes d^2.
inline constexpr double kGate99 = 9.21;

/// Initial velocity std-dev (m/s) for one-point track initialization — wide enough to
/// cover any scenario aircraft speed.
inline constexpr double kSigmaV0 = 150.0;

/// The measurement matrix H (2x4): it selects the position block of the state.
using MeasurementMatrix = Eigen::Matrix<double, 2, 4>;

/// Process-noise covariance Q for the discrete white-noise-acceleration model with
/// acceleration std-dev `sigma_a` over a step of `dt` seconds (spec formula). Throws
/// std::invalid_argument if `dt` is not finite/positive or `sigma_a` is not finite/>= 0.
[[nodiscard]] Eigen::Matrix4d process_noise(double dt, double sigma_a);

/// The measurement matrix H = [[1,0,0,0],[0,1,0,0]].
[[nodiscard]] MeasurementMatrix measurement_matrix() noexcept;

/// Measurement-noise covariance R = diag(sigma_m^2, sigma_m^2). Throws
/// std::invalid_argument if `sigma_m` is not finite or is negative.
[[nodiscard]] Eigen::Matrix2d measurement_noise(double sigma_m);

/// Predict step: x = F*x, P = F*P*F^T + Q, with F = motion::cv_transition_matrix(dt)
/// (the single source of truth for the CV model, reused per ADR-0006). Throws
/// std::invalid_argument on an invalid config or a non-finite input state.
[[nodiscard]] TrackState predict(const TrackState& state, const KalmanConfig& cfg);

/// One-point track initialization from a single measurement:
/// x = [z_x, z_y, 0, 0], P0 = diag(sigma_m^2, sigma_m^2, kSigmaV0^2, kSigmaV0^2).
/// Throws std::invalid_argument on an invalid `sigma_m` or non-finite measurement.
[[nodiscard]] TrackState initiate_track(const Measurement& z, const KalmanConfig& cfg);

/// Innovation (residual) y = z - H*x: measurement minus predicted measurement.
[[nodiscard]] Eigen::Vector2d innovation(const TrackState& state, const Measurement& z);

/// Innovation covariance S = H*P*H^T + R (the position block of P plus R). Throws
/// std::invalid_argument on an invalid `sigma_m` or non-finite state.
[[nodiscard]] Eigen::Matrix2d innovation_covariance(const TrackState& state,
                                                    const KalmanConfig& cfg);

/// Gated Mahalanobis distance d^2 = y^T * S^-1 * y. A measurement is inside a track's
/// gate when d^2 <= kGate99. S must be symmetric positive-definite: this is checked
/// with a Cholesky (LLT) factorization, not det(S) > 0 — a 2x2 negative-definite S
/// also has positive determinant, which would make d^2 negative and pass the gate for
/// any measurement (fail-open). Throws std::invalid_argument if y is non-finite or S is
/// not symmetric positive-definite. The result is >= 0 by construction.
[[nodiscard]] double gate_distance(const Eigen::Vector2d& y, const Eigen::Matrix2d& S);

/// Measurement update in Joseph form:
///   y = z - H*x;  S = H*P*H^T + R;  K = P*H^T*S^-1;
///   x = x + K*y;  P = (I - K*H)*P*(I - K*H)^T + K*R*K^T;  P = symmetrize(P).
/// Joseph form keeps P symmetric and positive semi-definite under rounding where the
/// textbook (I - K*H)*P does not; the symmetrize is a cheap extra guard (spec). Throws
/// std::invalid_argument on invalid config, non-finite inputs, or a degenerate S.
[[nodiscard]] TrackState update(const TrackState& state, const Measurement& z,
                                const KalmanConfig& cfg);

}  // namespace radar

#endif  // RADAR_KALMAN_HPP
