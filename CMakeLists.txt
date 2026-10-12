cmake_minimum_required(VERSION 3.25)

project(radar_tracker
  VERSION 0.0.1
  DESCRIPTION "Multi-target radar tracking system (C++20)"
  LANGUAGES CXX)

# ---- Standard --------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Position-independent code everywhere (needed for -pie hardening link).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# compile_commands.json for clang-tidy / clangd. (Single source of truth; the
# presets no longer duplicate this.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Options ---------------------------------------------------------------
option(RADAR_USE_SYSTEM_DEPS "Use find_package instead of FetchContent for deps" OFF)
option(RADAR_WERROR          "Treat warnings as errors (enabled by the ci presets)" OFF)
option(RADAR_HARDENING       "Enable OpenSSF-recommended hardening flags" OFF)
option(RADAR_CLANG_TIDY      "Run clang-tidy on our targets (enabled by the ci presets)" OFF)
option(RADAR_BUILD_TESTS     "Build the test suite" ON)

# clang-tidy command applied PER-TARGET (via radar_enable_tidy) so it analyzes
# only our code, never the fetched dependency sources (GoogleTest etc.).
if(RADAR_CLANG_TIDY)
  find_program(RADAR_CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
  set(RADAR_CLANG_TIDY_CMD "${RADAR_CLANG_TIDY_EXE}" "--warnings-as-errors=*")
endif()

# Attach clang-tidy to a target when RADAR_CLANG_TIDY is on. No-op otherwise.
function(radar_enable_tidy target)
  if(RADAR_CLANG_TIDY)
    set_target_properties(${target} PROPERTIES
      CXX_CLANG_TIDY "${RADAR_CLANG_TIDY_CMD}")
  endif()
endfunction()

# ---- Single-source version header ------------------------------------------
# Generated from PROJECT_VERSION so the string never drifts from project().
configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/version.hpp.in
  ${PROJECT_BINARY_DIR}/generated/radar/version.hpp
  @ONLY)

# ---- Warning baseline (applied to our targets only, via radar_warnings) ----
add_library(radar_warnings INTERFACE)
target_compile_options(radar_warnings INTERFACE
  -Wall
  -Wextra
  -Wpedantic
  -Wshadow
  -Wconversion
  -Wsign-conversion
  -Wnon-virtual-dtor
  -Wold-style-cast
  -Wformat=2
  -Wnull-dereference
  -Wdouble-promotion       # catches silent float->double in Kalman numerics
  -Wimplicit-fallthrough)

if(RADAR_WERROR)
  target_compile_options(radar_warnings INTERFACE -Werror)
endif()

# ---- Hardening (OpenSSF compiler-hardening guide) --------------------------
# Enabled by release/ci presets. _FORTIFY_SOURCE needs an optimizing build, so
# it is gated on non-Debug configurations.
add_library(radar_hardening INTERFACE)
if(RADAR_HARDENING)
  target_compile_options(radar_hardening INTERFACE
    $<$<NOT:$<CONFIG:Debug>>:-D_FORTIFY_SOURCE=3>
    -D_GLIBCXX_ASSERTIONS
    -fstack-protector-strong
    -fstack-clash-protection
    -fcf-protection=full)
  target_link_options(radar_hardening INTERFACE
    -Wl,-z,relro,-z,now
    -Wl,-z,noexecstack
    -pie)
endif()

# ---- Dependencies (Eigen, GoogleTest, nlohmann/json, CLI11) ----------------
include(cmake/Dependencies.cmake)

# ---- Library ---------------------------------------------------------------
add_subdirectory(src)

# ---- Tests -----------------------------------------------------------------
if(RADAR_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
