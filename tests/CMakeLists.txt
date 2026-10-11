# tests/CMakeLists.txt — GoogleTest-based unit tests, one executable per module.
# gtest_discover_tests registers each TEST() with CTest automatically.

include(GoogleTest)

# Consume GoogleTest headers as SYSTEM includes so their internal warnings
# (e.g. Clang 21's -Wcharacter-conversion in gtest-printers.h) never reach our
# -Werror build. See ADR-0002.
#
# GoogleTest also emits this warning while compiling its OWN sources (gtest-all.cc
# includes its headers via its own non-SYSTEM path). We do not apply -Werror to
# GoogleTest, so it is harmless, but we silence it on the gtest targets to keep the
# CI log pristine. This is a narrowly-scoped, documented suppression (ADR-0002):
# it touches only third-party targets, never our code.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-Wno-character-conversion" RADAR_HAS_WNO_CHAR_CONV)
foreach(_gt_tgt gtest gtest_main gmock gmock_main)
  if(TARGET ${_gt_tgt})
    get_target_property(_gt_inc ${_gt_tgt} INTERFACE_INCLUDE_DIRECTORIES)
    if(_gt_inc)
      set_target_properties(${_gt_tgt} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_gt_inc}")
    endif()
    if(RADAR_HAS_WNO_CHAR_CONV)
      target_compile_options(${_gt_tgt} PRIVATE -Wno-character-conversion)
    endif()
  endif()
endforeach()

# Helper: define a test executable that links libradar + GoogleTest + warnings.
function(radar_add_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name}
    PRIVATE
      radar::radar
      GTest::gtest_main
      radar_warnings)
  target_compile_features(${name} PRIVATE cxx_std_20)
  gtest_discover_tests(${name})
endfunction()

radar_add_test(test_smoke test_smoke.cpp)
