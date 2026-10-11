# cmake/Dependencies.cmake
# Fetches third-party dependencies pinned to explicit Git tags for reproducible
# builds. Toggle RADAR_USE_SYSTEM_DEPS=ON to use find_package instead (e.g. for
# Docker layer caching). Dependency includes are consumed as SYSTEM so their
# internal warnings never trip our -Werror.

include(FetchContent)

# Keep timestamps stable and quiet the FetchContent output a little.
set(FETCHCONTENT_QUIET OFF)

if(RADAR_USE_SYSTEM_DEPS)
  find_package(Eigen3 3.4 REQUIRED)
  find_package(GTest 1.14 REQUIRED)
  find_package(nlohmann_json 3.11 REQUIRED)
  find_package(CLI11 2.3 REQUIRED)
  return()
endif()

# ---- Eigen (header-only linear algebra) ------------------------------------
# Build nothing from Eigen's own tree; we only want the headers.
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE)

# ---- GoogleTest ------------------------------------------------------------
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
  GIT_SHALLOW    TRUE)

# ---- nlohmann/json ---------------------------------------------------------
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
  GIT_SHALLOW    TRUE)

# ---- CLI11 -----------------------------------------------------------------
FetchContent_Declare(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        v2.3.2
  GIT_SHALLOW    TRUE)

# Eigen's CMake exports target Eigen3::Eigen but we only need headers; populate
# manually to avoid pulling its full build/install logic.
FetchContent_MakeAvailable(Eigen3 googletest nlohmann_json CLI11)

# Eigen3::Eigen is an ALIAS to the INTERFACE target 'eigen', whose include dirs
# Eigen already exposes as SYSTEM, so third-party header warnings do not reach
# our -Werror build. We therefore do not (and cannot) call set_target_properties
# on the alias here. Consumers link Eigen3::Eigen as a PUBLIC dependency.
