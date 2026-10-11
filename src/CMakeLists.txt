# src/CMakeLists.txt — defines the libradar static library.
# All project logic lives here so it is unit-testable; apps/ (added later) and
# tests/ link against this target. New modules add their .cpp here and a matching
# header under include/radar/.

add_library(radar STATIC
  version.cpp)

# Public headers live under include/radar/.
target_include_directories(radar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)

# Link Eigen (header-only, SYSTEM includes) and apply our warning baseline.
target_link_libraries(radar
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    radar_warnings)

target_compile_features(radar PUBLIC cxx_std_20)

# Give the target a friendly archive name: libradar.a
set_target_properties(radar PROPERTIES OUTPUT_NAME radar)

# Namespaced alias so consumers (tests, apps) use radar::radar.
add_library(radar::radar ALIAS radar)
