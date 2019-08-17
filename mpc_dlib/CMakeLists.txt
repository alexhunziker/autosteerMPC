#cmake_minimum_required(VERSION 2.8.12)
# Define Project Name
#project(mpc)

# Declare where to find dlib
#add_subdirectory(../dlib dlib_build)

# Add executable by specifying output executable and respective .cpp files
# If there are multiply .cpp files they can be listed in the add_executable() statement
#add_executable(mpc_test mpc_test.cpp)
# Declare that program depends on dlib
#target_link_libraries(mpc_test dlib::dlib)


# To compile this program cmake is required.
# Compiling from this folder:
#   mkdir build
#   cd build
#   cmake ..
#   cmake --build . --config Release


#
# This is a CMake makefile.  You can find the cmake utility and
# information about it at http://www.cmake.org
#

cmake_minimum_required(VERSION 2.8.12)

# create a variable called target_name and set it to the string "dtest"
set (target_name dtest)
PROJECT(${target_name})

# CUSTOM CHANGE DUE TO UNDEFINED REFERENCE __ATOMIC_LOAD_8
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -latomic")


add_subdirectory(../dlib dlib_build)

# This variable contains a list of all the tests we are building
# into the regression test suite.
set (test mpc_test.cpp)


# add all the cpp files we want to compile to this list.  This tells
# cmake that they are part of our target (which is the executable named dtest)
ADD_EXECUTABLE(${target_name} main.cpp tester.cpp ${tests})


TARGET_LINK_LIBRARIES(${target_name} dlib::dlib )


if (NOT DLIB_NO_GUI_SUPPORT)
   add_subdirectory(gui)
   add_subdirectory(examples)
   add_subdirectory(tools)
endif()
