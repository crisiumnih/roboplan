find_package(GTest REQUIRED)

# Build tests
add_executable(test_se3_low_pass_filter test_se3_low_pass_filter.cpp)
set_property(TARGET test_se3_low_pass_filter PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_se3_low_pass_filter
  roboplan_filters
  GTest::gtest_main
)

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if ( ament_cmake_gtest_FOUND )
    ament_add_gtest_test(test_se3_low_pass_filter)
else()
    include(GoogleTest)
    gtest_discover_tests(test_se3_low_pass_filter)
endif()
