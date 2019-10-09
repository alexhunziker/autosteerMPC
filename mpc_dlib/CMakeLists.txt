## This file generates the MPC Controller Binaries
## To compile, cmake is required. (see: cmake.org)

# Compiling from this folder using these commands:
#   mkdir build
#   cd build
#   cmake ..
#   cmake --build . --config Release
# ... or run the shell script

cmake_minimum_required(VERSION 2.8.12)

# set variable target name
set (target_name mpc_test)

PROJECT(${target_name})
# set(CMAKE_BUILD_TYPE Release)

# Define missing references on Raspberry Pi (__ATOMIC_LOAD_8 etc)
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -latomic")

add_subdirectory(../dlib dlib_build)

# Executable for testing: add all files to the targets, and tell cmake to use dlib
#ADD_EXECUTABLE(${target_name} standalone_test.cpp)
#TARGET_LINK_LIBRARIES(${target_name} dlib::dlib )
ADD_EXECUTABLE(mpc_cv_test mpc_cv_test.cpp)
TARGET_LINK_LIBRARIES(mpc_cv_test dlib::dlib )

# Shared Object for actual implementation: add all files to the targets, and tell cmake to use dlib
#add_library(mpc_controller SHARED mpc_controller.cpp)
#TARGET_LINK_LIBRARIES(mpc_controller dlib::dlib )

# TODO: Should be one shared object, include in testfile
#add_library(mpc_controller_distance SHARED mpc_distance.cpp)
#TARGET_LINK_LIBRARIES(mpc_controller_distance dlib::dlib )

add_library(mpc_mpc_follow_lane SHARED mpc_follow_lane.cpp)
TARGET_LINK_LIBRARIES(mpc_mpc_follow_lane dlib::dlib )
