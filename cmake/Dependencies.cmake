# cmake/Dependencies.cmake
# Fetches third-party dependencies for reproducible, verifiable builds.
#
# Integrity: each dependency is pinned to a FULL 40-character commit SHA, not a
# mutable tag. A tag can be force-pushed upstream; a SHA cannot change without
# changing content, so a fresh clone and a CI run always build identical code.
# The resolved tag each SHA corresponds to is noted in a comment.
#
# SYSTEM includes: every FetchContent_Declare uses the SYSTEM keyword (CMake
# >= 3.25) so dependency headers are consumed via -isystem, not -I. This keeps
# our strict -Werror warning set (sign-conversion, old-style-cast, ...) from
# firing on third-party code (notably Eigen, which is not -Werror-clean under
# our flags). This is the correct mechanism; marking an ALIAS target's includes
# SYSTEM after the fact does not work.
#
# Toggle RADAR_USE_SYSTEM_DEPS=ON to use find_package instead (e.g. for Docker
# layer caching).

include(FetchContent)

if(RADAR_USE_SYSTEM_DEPS)
  find_package(Eigen3 3.4 REQUIRED)
  find_package(nlohmann_json 3.11 REQUIRED)
  find_package(CLI11 2.3 REQUIRED)
  if(RADAR_BUILD_TESTS)
    find_package(GTest 1.14 REQUIRED)
  endif()
  return()
endif()

# ---- Eigen (header-only linear algebra) ------------------------------------
# 3147391d946bb4b6c68edd901f2add6ac1f31f8c == tag 3.4.0
# Eigen 3.4 registers its own ~900-test suite when BUILD_TESTING is ON (the
# default). We only consume Eigen's headers, so disable it. This is set as a
# normal cache variable (no FORCE) scoped before Eigen is populated; we drive
# our own tests via enable_testing(), not include(CTest), so this does not
# interfere with the project's test harness or with subproject consumers.
set(BUILD_TESTING OFF CACHE BOOL "Build third-party dependency test suites")
FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3147391d946bb4b6c68edd901f2add6ac1f31f8c
  GIT_SUBMODULES ""
  SYSTEM)

# ---- nlohmann/json ---------------------------------------------------------
# 9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03 == tag v3.11.3
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03
  GIT_SUBMODULES ""
  SYSTEM)

# ---- CLI11 -----------------------------------------------------------------
# 291c58789c031208f08f4f261a858b5b7083e8e2 == tag v2.3.2
FetchContent_Declare(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        291c58789c031208f08f4f261a858b5b7083e8e2
  GIT_SUBMODULES ""
  SYSTEM)

FetchContent_MakeAvailable(Eigen3 nlohmann_json CLI11)

# ---- GoogleTest (only when building tests) ---------------------------------
# f8d7d77c06936315286eb55f8de22cd23c188571 == tag v1.14.0
if(RADAR_BUILD_TESTS)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        f8d7d77c06936315286eb55f8de22cd23c188571
    GIT_SUBMODULES ""
    SYSTEM)
  FetchContent_MakeAvailable(googletest)
endif()
