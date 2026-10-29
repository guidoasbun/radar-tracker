# src/CMakeLists.txt — defines the libradar static library.
# All project logic lives here so it is unit-testable; apps/ (added later) and
# tests/ link against this target.

add_library(radar STATIC
  linalg.cpp
  motion.cpp
  sensor.cpp
  kalman.cpp
  io/config.cpp
  io/csv.cpp)

# Public headers: the checked-in include/ tree plus the generated version header
# under the build dir. Use PROJECT_SOURCE_DIR/PROJECT_BINARY_DIR so the library
# works when consumed as a subproject.
target_include_directories(radar
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/generated>)

target_link_libraries(radar
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    nlohmann_json::nlohmann_json
    radar_warnings
    radar_hardening)

target_compile_features(radar PUBLIC cxx_std_20)
set_target_properties(radar PROPERTIES OUTPUT_NAME radar)
radar_enable_tidy(radar)
add_library(radar::radar ALIAS radar)
