cmake_minimum_required(VERSION 3.24)

project(radar_tracker
  VERSION 0.0.1
  DESCRIPTION "Multi-target radar tracking system (C++20)"
  LANGUAGES CXX)

# ---- Standard --------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json for clang-tidy / clangd.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Options ---------------------------------------------------------------
option(RADAR_USE_SYSTEM_DEPS "Use find_package instead of FetchContent for deps" OFF)
option(RADAR_WERROR          "Treat warnings as errors (enabled by the ci preset)" OFF)
option(RADAR_BUILD_TESTS     "Build the test suite" ON)

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
  -Wold-style-cast)

if(RADAR_WERROR)
  target_compile_options(radar_warnings INTERFACE -Werror)
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
