// tests/test_smoke.cpp — Phase 0 scaffold test.
// Proves the full path works: libradar compiles, the test links against it,
// GoogleTest is wired to CTest, and sanitizers run clean on a real executable.

#include <gtest/gtest.h>

#include <string_view>

#include "radar/version.hpp"

TEST(Smoke, VersionIsNonEmpty) {
  const std::string_view v = radar::version_string();
  EXPECT_FALSE(v.empty());
}

TEST(Smoke, VersionLooksLikeSemver) {
  const std::string_view v = radar::version_string();
  // Expect at least "X.Y.Z" form: two dots present.
  const auto first_dot = v.find('.');
  ASSERT_NE(first_dot, std::string_view::npos);
  const auto second_dot = v.find('.', first_dot + 1);
  EXPECT_NE(second_dot, std::string_view::npos);
}
