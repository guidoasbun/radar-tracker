#ifndef RADAR_LINALG_HPP
#define RADAR_LINALG_HPP

#include <Eigen/Dense>

namespace radar {

/// Returns the symmetric part of a 4x4 matrix: (M + Mᵀ) / 2.
///
/// Used to re-symmetrize a Kalman covariance after an update, where rounding can
/// introduce tiny asymmetries (spec: "symmetrize after each update, P = (P + Pᵀ)/2").
/// Units: same as the input matrix entries.
[[nodiscard]] Eigen::Matrix4d symmetrize(const Eigen::Matrix4d& m) noexcept;

}  // namespace radar

#endif  // RADAR_LINALG_HPP
