#include "radar/linalg.hpp"

#include <Eigen/Dense>

namespace radar {

Eigen::Matrix4d symmetrize(const Eigen::Matrix4d& m) noexcept { return 0.5 * (m + m.transpose()); }

}  // namespace radar
