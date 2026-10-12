// tests/test_linalg.cpp — regression guard for the Eigen SYSTEM-include fix.
//
// This test includes <Eigen/Dense> and performs real matrix operations. Under
// the ci preset (-Werror with the full strict warning set), it only compiles if
// Eigen's headers are consumed via -isystem. If Eigen were ever reverted to a
// plain -I include, this translation unit would fail the ci build — which is the
// point: it exercises the Eigen path that test_smoke.cpp alone does not.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "radar/linalg.hpp"

TEST(Linalg, SymmetrizeOfSymmetricMatrixIsUnchanged) {
  Eigen::Matrix4d s = Eigen::Matrix4d::Zero();
  s(0, 0) = 2.0;
  s(1, 1) = 3.0;
  s(0, 1) = 1.5;
  s(1, 0) = 1.5;  // already symmetric
  const Eigen::Matrix4d out = radar::symmetrize(s);
  EXPECT_TRUE(out.isApprox(s));
}

TEST(Linalg, SymmetrizeProducesSymmetricResult) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
  m(0, 1) = 4.0;
  m(1, 0) = 0.0;  // asymmetric: off-diagonal differs
  const Eigen::Matrix4d out = radar::symmetrize(m);
  // Result must equal its own transpose.
  EXPECT_TRUE(out.isApprox(out.transpose()));
  // Averaged value: (4 + 0) / 2 == 2 on both sides.
  EXPECT_DOUBLE_EQ(out(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(out(1, 0), 2.0);
}

TEST(Linalg, IdentityMatrixOperations) {
  const Eigen::Matrix4d id = Eigen::Matrix4d::Identity();
  EXPECT_DOUBLE_EQ(id.trace(), 4.0);
  EXPECT_DOUBLE_EQ((id * id).trace(), 4.0);
}
