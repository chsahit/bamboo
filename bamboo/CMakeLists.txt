cmake_minimum_required(VERSION 3.10)
project(bamboo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set path to locally built libfranka
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../libfranka/build")

# Set RPATH so the executable can find libfranka.so at runtime
set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../libfranka/build")
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../libfranka/build")

# Find required packages
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(cppzmq REQUIRED)

# Generate protobuf files
set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
file(GLOB PROTO_FILES "${PROTO_SRC_DIR}/*.proto")

set(PROTO_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_generated")
file(MAKE_DIRECTORY ${PROTO_GENERATED_DIR})

foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    set(PROTO_SRC "${PROTO_GENERATED_DIR}/${PROTO_NAME}.pb.cc")
    set(PROTO_HDR "${PROTO_GENERATED_DIR}/${PROTO_NAME}.pb.h")

    add_custom_command(
        OUTPUT ${PROTO_SRC} ${PROTO_HDR}
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out=${PROTO_GENERATED_DIR}
             --python_out=${CMAKE_CURRENT_SOURCE_DIR}/examples
             -I${PROTO_SRC_DIR}
             ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating protobuf files for ${PROTO_NAME}"
    )

    list(APPEND PROTO_SRCS ${PROTO_SRC})
    list(APPEND PROTO_HDRS ${PROTO_HDR})
endforeach()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/proto_generated
    ${EIGEN3_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
)

# Create library for controllers
add_library(bamboo_controllers
    src/controllers/joint_impedance_controller.cpp
    ${PROTO_SRCS}
)

target_link_libraries(bamboo_controllers
    Franka::Franka
    Eigen3::Eigen
    ${PROTOBUF_LIBRARIES}
)

# Create control node executable
add_executable(bamboo_control_node
    src/control_node.cpp
)

target_link_libraries(bamboo_control_node
    bamboo_controllers
    Franka::Franka
    Eigen3::Eigen
    ${PROTOBUF_LIBRARIES}
    cppzmq
)

# Set compiler warnings
target_compile_options(bamboo_controllers PRIVATE -Wall -Wextra)
target_compile_options(bamboo_control_node PRIVATE -Wall -Wextra)

# Print info
message(STATUS "Bamboo build configuration:")
message(STATUS "  Franka found: ${Franka_FOUND}")
message(STATUS "  Eigen3 found: ${EIGEN3_FOUND}")
message(STATUS "  Protobuf found: ${Protobuf_FOUND}")
message(STATUS "  Protobuf version: ${Protobuf_VERSION}")
message(STATUS "  Proto files: ${PROTO_FILES}")
