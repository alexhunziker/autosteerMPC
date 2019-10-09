## This file generates the MPC Controller Binaries
## To compile, cmake is required. (see: cmake.org)

# Compiling from this folder using these commands:
#   mkdir build
#   cd build
#   cmake ..
#   cmake --build . --config Release
# ... or run the shell script

cmake_minimum_required(VERSION 2.8.12)

PROJECT(mpc_objects)
set(CMAKE_BUILD_TYPE Release)

# Define missing references on Raspberry Pi (__ATOMIC_LOAD_8 etc)
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -latomic")

add_subdirectory(../dlib dlib_build)


# Executable for testing: add all files to the targets, and tell cmake to use dlib
ADD_EXECUTABLE(mpc_cv_unconstr_test mpc_cv_unconstr_test.cpp)
TARGET_LINK_LIBRARIES(mpc_cv_unconstr_test dlib::dlib )

ADD_EXECUTABLE(mpc_cv_obst_test mpc_cv_obst_test.cpp)
TARGET_LINK_LIBRARIES(mpc_cv_obst_test dlib::dlib )

ADD_EXECUTABLE(mpc_gps_obst_test mpc_gps_obst_test.cpp)
TARGET_LINK_LIBRARIES(mpc_gps_obst_test dlib::dlib )

ADD_EXECUTABLE(mpc_gps_unconstr_test mpc_gps_unconstr_test.cpp)
TARGET_LINK_LIBRARIES(mpc_gps_unconstr_test dlib::dlib )


# Shared Object for actual implementation: add all files to the targets, and tell cmake to use dlib
add_library(mpc_cv_unconstr SHARED mpc_cv_unconstr.cpp)
TARGET_LINK_LIBRARIES(mpc_cv_unconstr dlib::dlib )

add_library(mpc_cv_obst SHARED mpc_cv_obst.cpp)
TARGET_LINK_LIBRARIES(mpc_cv_obst dlib::dlib )

add_library(mpc_gps_obst SHARED mpc_gps_obst.cpp)
TARGET_LINK_LIBRARIES(mpc_gps_obst dlib::dlib )

add_library(mpc_gps_unconstr SHARED mpc_gps_unconstr.cpp)
TARGET_LINK_LIBRARIES(mpc_gps_unconstr dlib::dlib )