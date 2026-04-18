cmake_minimum_required(VERSION 3.22)
project(roboplan_filters VERSION 0.2.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Add libraries
add_library(roboplan_filters SHARED
    src/se3_low_pass_filter.cpp
)
target_include_directories(roboplan_filters PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_filters
    PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
)
set_property(TARGET roboplan_filters PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan_filters
  EXPORT roboplan_filters-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_filtersConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_filtersConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_filtersConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_filtersConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_filters
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_filtersConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_filtersConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_filtersConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_filters"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_filters
        DESTINATION lib/cmake/)

install(EXPORT roboplan_filters-targets
        FILE roboplan_filtersTargets.cmake
        NAMESPACE roboplan_filters::
        DESTINATION lib/cmake/roboplan_filters)

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
