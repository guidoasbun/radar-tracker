#include "radar/motion.hpp"

#include <Eigen/Core>
#include <cassert>
#include <cmath>

#include "radar/types.hpp"

namespace radar {

Eigen::Matrix4d cv_transition_matrix(double dt) noexcept {
  assert(std::isfinite(dt) && "cv_transition_matrix: dt must be finite");
  Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
  f(kPx, kVx) = dt;  // p_x += v_x * dt
  f(kPy, kVy) = dt;  // p_y += v_y * dt
  return f;
}

Eigen::Vector4d propagate_cv(const Eigen::Vector4d& x, double dt) noexcept {
  // Precondition: finite inputs. The matrix product F*x multiplies x by F's zero
  // entries too, and 0 * NaN == NaN / 0 * inf == NaN, so a non-finite input (or
  // dt) spreads to components the explicit p += v*dt would never touch. Asserting
  // here catches misuse in debug/ASan; io validates scenario input at the boundary.
  assert(std::isfinite(dt) && "propagate_cv: dt must be finite");
  assert(x.allFinite() && "propagate_cv: x must be finite");

  // Implemented as F*x (not an inline p += v*dt) so this and cv_transition_matrix
  // encode the CV law in exactly one place.
  return cv_transition_matrix(dt) * x;
}

}  // namespace radar
