cmake_minimum_required(VERSION 3.10)
project(bamboo)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set path to locally built libfranka and our install directory
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfranka/build" ${CMAKE_PREFIX_PATH})

# Set RPATH so the executable can find libfranka.so at runtime
set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfranka/build")
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/libfranka/build")

# Add our install directory to CMAKE_PREFIX_PATH so all packages can be found
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../install")

# Find required packages
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# Force use of our local protobuf installation ONLY
set(protobuf_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/cmake/protobuf")
find_package(protobuf CONFIG REQUIRED)

# Set protoc executable location for use in custom commands
get_target_property(PROTOBUF_PROTOC_EXECUTABLE protobuf::protoc IMPORTED_LOCATION_NOCONFIG)
if(NOT PROTOBUF_PROTOC_EXECUTABLE)
    get_target_property(PROTOBUF_PROTOC_EXECUTABLE protobuf::protoc IMPORTED_LOCATION)
endif()

# Find gRPC - use local installation
find_package(gRPC CONFIG REQUIRED)
set(GRPC_CPP_PLUGIN_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/../install/bin/grpc_cpp_plugin")
set(GRPC_LIBRARIES gRPC::grpc++ gRPC::grpc)
# Find zmqpp manually since we built it locally
find_library(ZMQPP_LIBRARY zmqpp PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib" REQUIRED)
find_path(ZMQPP_INCLUDE_DIR zmqpp/zmqpp.hpp PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../install/include" REQUIRED)

# Generate protobuf and gRPC files
set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
file(GLOB PROTO_FILES "${PROTO_SRC_DIR}/*.proto")

set(PROTO_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_generated")
file(MAKE_DIRECTORY ${PROTO_GENERATED_DIR})

# Python protobuf output directory
set(PROTO_PYTHON_OUT "${CMAKE_CURRENT_SOURCE_DIR}/proto_gen")
file(MAKE_DIRECTORY ${PROTO_PYTHON_OUT})

foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    set(PROTO_SRC "${PROTO_GENERATED_DIR}/${PROTO_NAME}.pb.cc")
    set(PROTO_HDR "${PROTO_GENERATED_DIR}/${PROTO_NAME}.pb.h")

    # Check if this is the service proto file (contains gRPC service definitions)
    if(PROTO_NAME STREQUAL "bamboo_service")
        set(GRPC_SRC "${PROTO_GENERATED_DIR}/${PROTO_NAME}.grpc.pb.cc")
        set(GRPC_HDR "${PROTO_GENERATED_DIR}/${PROTO_NAME}.grpc.pb.h")

        add_custom_command(
            OUTPUT ${PROTO_SRC} ${PROTO_HDR} ${GRPC_SRC} ${GRPC_HDR}
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
            ARGS --cpp_out=${PROTO_GENERATED_DIR}
                 --grpc_out=${PROTO_GENERATED_DIR}
                 --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXECUTABLE}
                 -I${PROTO_SRC_DIR}
                 ${PROTO_FILE}
            COMMAND python -m grpc_tools.protoc
            ARGS --python_out=${PROTO_PYTHON_OUT}
                 --grpc_python_out=${PROTO_PYTHON_OUT}
                 -I${PROTO_SRC_DIR}
                 ${PROTO_FILE}
            DEPENDS ${PROTO_FILE}
            COMMENT "Generating protobuf and gRPC files for ${PROTO_NAME}"
        )

        list(APPEND PROTO_SRCS ${PROTO_SRC} ${GRPC_SRC})
        list(APPEND PROTO_HDRS ${PROTO_HDR} ${GRPC_HDR})
    else()
        add_custom_command(
            OUTPUT ${PROTO_SRC} ${PROTO_HDR}
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
            ARGS --cpp_out=${PROTO_GENERATED_DIR}
                 --python_out=${PROTO_PYTHON_OUT}
                 -I${PROTO_SRC_DIR}
                 ${PROTO_FILE}
            DEPENDS ${PROTO_FILE}
            COMMENT "Generating protobuf files for ${PROTO_NAME}"
        )

        list(APPEND PROTO_SRCS ${PROTO_SRC})
        list(APPEND PROTO_HDRS ${PROTO_HDR})
    endif()
endforeach()

# Include directories - prioritize our local protobuf
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/proto_generated
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/include"  # Our protobuf headers FIRST
    ${EIGEN3_INCLUDE_DIRS}
    ${ZMQPP_INCLUDE_DIR}
)

# Create library for controllers
add_library(bamboo_controllers
    src/controllers/joint_impedance_controller.cpp
    ${PROTO_SRCS}
)

target_link_libraries(bamboo_controllers
    Franka::Franka
    Eigen3::Eigen
    protobuf::libprotobuf
)

# Create control node executable
add_executable(bamboo_control_node
    src/control_node.cpp
)

# Collect all local libraries needed
file(GLOB ABSL_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libabsl_*.a")
set(UTF8_RANGE_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libutf8_range_lib.a")
set(UPB_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libupb.a")
# Add specific debugging libraries for missing symbols
set(DEBUGGING_LIBS
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libabsl_debugging_internal.a"
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libabsl_demangle_internal.a"
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libabsl_demangle_rust.a"
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libabsl_decode_rust_punycode.a"
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libabsl_utf8_for_code_point.a"
)

target_link_libraries(bamboo_control_node
    bamboo_controllers
    Franka::Franka
    Eigen3::Eigen
    protobuf::libprotobuf  # Use protobuf target
    ${GRPC_LIBRARIES}      # Add gRPC libraries
    gRPC::grpc++_reflection  # Add reflection library
    # Add Pinocchio libraries that libfranka depends on
    /opt/openrobots/lib/libpinocchio_default.so
    /opt/openrobots/lib/libpinocchio_parsers.so
    # Add all absl libraries for our protobuf - link twice to resolve circular deps
    ${DEBUGGING_LIBS}
    ${ABSL_LIBS}
    ${UTF8_RANGE_LIB}
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libutf8_range.a"
    "${CMAKE_CURRENT_SOURCE_DIR}/../install/lib/libutf8_validity.a"
    ${UPB_LIB}
    ${ABSL_LIBS}
    ${DEBUGGING_LIBS}
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
