cmake_minimum_required(VERSION 3.10)
project(minimal_joint_impedance)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set path to locally built libfranka
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../libfranka/build")

# Find required packages
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create library for controller and interpolator
add_library(minimal_controllers
    joint_impedance_controller.cpp
    min_jerk_interpolator.cpp
)

target_link_libraries(minimal_controllers
    Franka::Franka
    Eigen3::Eigen
)

# Create test executable
add_executable(test_joint_impedance
    test_joint_impedance.cpp
)

target_link_libraries(test_joint_impedance
    minimal_controllers
    Franka::Franka
    Eigen3::Eigen
)

# Set compiler warnings
target_compile_options(minimal_controllers PRIVATE -Wall -Wextra)
target_compile_options(test_joint_impedance PRIVATE -Wall -Wextra)
