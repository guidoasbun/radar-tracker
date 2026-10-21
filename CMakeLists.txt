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

# The asan preset passes -fsanitize-ignorelist=cmake/sanitizer-ignorelist.txt via a
# compiler flag, which CMake does not track as an input — so editing the ignorelist
# would not, on its own, trigger a rebuild and a stale asan build could hide an added
# or removed suppression. Register it as a configure dependency so a change forces a
# re-configure (and thus a rebuild of the sanitized objects).
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${PROJECT_SOURCE_DIR}/cmake/sanitizer-ignorelist.txt")

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
# Enabled by release/ci presets. Architecture-aware: control-flow protection
# uses -fcf-protection on x86 (CET) and -mbranch-protection on ARM; neither flag
# is portable across both. _FORTIFY_SOURCE needs an optimizing build, and some
# distros predefine it — so we -U it first to avoid a "redefined" -Werror.
add_library(radar_hardening INTERFACE)
if(RADAR_HARDENING)
  target_compile_options(radar_hardening INTERFACE
    # -U before -D so a distro-predefined _FORTIFY_SOURCE cannot clash (N5).
    $<$<NOT:$<CONFIG:Debug>>:-U_FORTIFY_SOURCE>
    $<$<NOT:$<CONFIG:Debug>>:-D_FORTIFY_SOURCE=3>
    -D_GLIBCXX_ASSERTIONS
    -fstack-protector-strong
    -fstack-clash-protection)

  # Control-flow integrity: pick the flag that exists on the target arch (N2).
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i[3-6]86")
    target_compile_options(radar_hardening INTERFACE -fcf-protection=full)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|armv8")
    target_compile_options(radar_hardening INTERFACE -mbranch-protection=standard)
  endif()

  target_link_options(radar_hardening INTERFACE
    -Wl,-z,relro,-z,now
    -Wl,-z,noexecstack
    -pie)
endif()

# ---- Dependencies (Eigen, GoogleTest, nlohmann/json, CLI11) ----------------
include(cmake/Dependencies.cmake)

# ---- Library ---------------------------------------------------------------
add_subdirectory(src)

# Helper: define an application executable with the SAME security posture as the
# library — warnings, hardening, and clang-tidy all applied automatically. Use
# this for every executable under apps/ (radar_sim, radar_tracker, ...), which
# are the binaries that handle untrusted input, so hardening is never forgotten.
# Usage: radar_add_app(radar_sim main.cpp foo.cpp)
function(radar_add_app name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name}
    PRIVATE
      radar::radar
      radar_warnings
      radar_hardening)
  target_compile_features(${name} PRIVATE cxx_std_20)
  radar_enable_tidy(${name})
endfunction()

# ---- Applications (added from M1 onward) -----------------------------------
if(EXISTS "${PROJECT_SOURCE_DIR}/apps/CMakeLists.txt")
  add_subdirectory(apps)
endif()

# ---- Tests -----------------------------------------------------------------
if(RADAR_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
