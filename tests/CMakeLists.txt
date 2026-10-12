# tests/CMakeLists.txt — GoogleTest-based unit tests, one executable per module.
# gtest_discover_tests registers each TEST() with CTest automatically.
#
# GoogleTest's headers are consumed as -isystem automatically because its
# FetchContent_Declare uses the SYSTEM keyword (see cmake/Dependencies.cmake),
# so no per-target SYSTEM workaround is needed here.

include(GoogleTest)

# GoogleTest headers reach our test TUs as -isystem (its FetchContent_Declare
# uses the SYSTEM keyword), so its header warnings do not affect our code.
# GoogleTest still emits Clang 21's -Wcharacter-conversion while compiling its
# OWN sources (self-included via a non-SYSTEM path). We do not build GoogleTest
# with -Werror, so it is harmless; we silence it on the gtest targets only to
# keep the CI log clean. Narrow, third-party-only, documented in ADR-0002.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-Wno-character-conversion" RADAR_HAS_WNO_CHAR_CONV)
if(RADAR_HAS_WNO_CHAR_CONV)
  foreach(_gt_tgt gtest gtest_main gmock gmock_main)
    if(TARGET ${_gt_tgt})
      target_compile_options(${_gt_tgt} PRIVATE -Wno-character-conversion)
    endif()
  endforeach()
endif()

# Helper: define a test executable that links libradar + GoogleTest + warnings.
function(radar_add_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name}
    PRIVATE
      radar::radar
      GTest::gtest_main
      radar_warnings
      radar_hardening)
  target_compile_features(${name} PRIVATE cxx_std_20)
  radar_enable_tidy(${name})
  gtest_discover_tests(${name})
endfunction()

radar_add_test(test_smoke test_smoke.cpp)
radar_add_test(test_linalg test_linalg.cpp)
radar_add_test(test_types test_types.cpp)
radar_add_test(test_motion test_motion.cpp)
