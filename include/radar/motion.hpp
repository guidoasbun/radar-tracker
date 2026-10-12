#ifndef RADAR_MOTION_HPP
#define RADAR_MOTION_HPP

#include <Eigen/Core>

#include "radar/types.hpp"  // state-vector layout (kPx, kVx, ...)

namespace radar {

/// Constant-velocity (CV) state-transition matrix F for a step of `dt` seconds,
/// over the state x = [p_x, p_y, v_x, v_y]:
///
///       | 1 0 dt 0 |
///   F = | 0 1 0 dt |      x_{k+1} = F * x_k
///       | 0 0 1  0 |
///       | 0 0 0  1 |
///
/// This is the single source of truth for the CV model: the simulator uses it to
/// advance targets, and `kalman`'s predict step reuses the same F for
/// x̂ = F·x̂ and P = F·P·Fᵀ + Q, so the truth model and the filter's model cannot
/// silently diverge. (That `kalman` depends on `motion` for F is a deliberate
/// deviation from the spec's module table — see ADR-0006.) `dt` is seconds;
/// `cv_transition_matrix(0)` is the identity.
///
/// Precondition: `dt` must be finite. A non-finite `dt` is a caller error
/// (asserted in debug/ASan builds); `io` validates `dt_s` at the boundary.
[[nodiscard]] Eigen::Matrix4d cv_transition_matrix(double dt) noexcept;

/// Advance a state vector one CV step: position += velocity * dt, velocity
/// unchanged. For finite inputs this is equal to `cv_transition_matrix(dt) * x`
/// (and implemented that way, so the two forms are provably consistent).
///
/// `dt` may be negative: that propagates backward in time, and `propagate_cv`
/// then `propagate_cv` with the negated `dt` returns to the start (covered by a
/// test). `dt == 0` is a no-op for finite `x`.
///
/// Operates on the plain state vector, not a `TrackState`, because this is *truth*
/// propagation: a true target has a state, not a covariance. Covariance
/// propagation (F·P·Fᵀ + Q, including the process noise Q) is the filter's job and
/// lives in `kalman`; `motion` is deterministic and noise-free.
///
/// Precondition: `dt` and every component of `x` must be finite. This matters
/// because the implementation forms the full matrix product F·x, where the zero
/// entries of F still multiply x — and 0 * NaN == NaN, 0 * inf == NaN — so a
/// single non-finite input contaminates components the explicit `p += v·dt` would
/// leave alone, and a non-finite `dt` turns even a stationary target into NaN.
/// The precondition is asserted in debug/ASan builds; `io` must reject non-finite
/// or non-positive `dt_s` and non-finite target states at the boundary so a
/// malformed scenario fails loudly instead of silently producing a NaN
/// trajectory (mirrors the P-finiteness note in types.hpp).
[[nodiscard]] Eigen::Vector4d propagate_cv(const Eigen::Vector4d& x, double dt) noexcept;

// ---------------------------------------------------------------------------
// DEFERRED (M2): coordinated-turn (CT) maneuver model.
//
// Not implemented in M1 (M1 scenarios are straight-line / CV only). Specified
// here as the forward contract so it is completed before the maneuver scenarios
// run. During a maneuver segment a target turns at rate omega (rad/s):
//
//   v_{k+1} = Rot(omega*dt) * v_k ;   p_{k+1} = p_k + v_{k+1} * dt
//   Rot(theta) = [[cos theta, -sin theta], [sin theta, cos theta]]
//
// Correctness notes for the future implementer:
//   * the Euler position step uses the POST-rotation velocity v_{k+1}, not v_k;
//   * omega == 0 must reduce EXACTLY to propagate_cv -- cos(0)==1, sin(0)==0 are
//     exact, so the future CT test should demand == (bit-for-bit), not isApprox.
//
// Planned signature (intentionally not declared yet to avoid untested surface;
// Vector4d, matching propagate_cv):
//   [[nodiscard]] Eigen::Vector4d ct_propagate(const Eigen::Vector4d& x,
//                                               double dt, double omega) noexcept;
// A maneuver-segment type (time window + turn rate) and the simulator logic that
// selects CV vs CT per target per scan land alongside it.
// ---------------------------------------------------------------------------

}  // namespace radar

#endif  // RADAR_MOTION_HPP
