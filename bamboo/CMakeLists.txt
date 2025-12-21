cmake_minimum_required(VERSION 3.10)
project(bamboo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add our install directory to CMAKE_PREFIX_PATH so all packages can be found
# (libfranka and other dependencies are installed here by InstallPackage)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../install")

# Set RPATH so the executable can find libfranka.so at runtime
# Point to the install directory where InstallPackage puts the libraries
set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib")
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib")
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Find required packages
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# Find ZMQ (will use system package)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# msgpack is header-only, just need to find it
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp
    PATHS /usr/include /usr/local/include
    PATH_SUFFIXES msgpack msgpack-cxx
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${MSGPACK_INCLUDE_DIR}
)

# Create library for controllers
add_library(bamboo_controllers
    src/controllers/joint_impedance_controller.cpp
)

target_link_libraries(bamboo_controllers
    Franka::Franka
    Eigen3::Eigen
)

# Create control node executable
add_executable(bamboo_control_node
    src/control_node.cpp
)

target_link_libraries(bamboo_control_node
    bamboo_controllers
    Franka::Franka
    Eigen3::Eigen
    ${ZMQ_LIBRARIES}
)

# Set compiler warnings
target_compile_options(bamboo_controllers PRIVATE -Wall -Wextra)
target_compile_options(bamboo_control_node PRIVATE -Wall -Wextra)

# Print info
message(STATUS "Bamboo build configuration:")
message(STATUS "  Franka found: ${Franka_FOUND}")
message(STATUS "  Eigen3 found: ${EIGEN3_FOUND}")
message(STATUS "  ZMQ found: ${ZMQ_FOUND}")
message(STATUS "  ZMQ include dirs: ${ZMQ_INCLUDE_DIRS}")
message(STATUS "  msgpack include dir: ${MSGPACK_INCLUDE_DIR}")
