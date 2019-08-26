cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_sensor)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_BINARY_DIR}
     ${CMAKE_SOURCE_DIR}/../ydlidar_sdk
     ${CMAKE_CURRENT_BINARY_DIR}
)

add_subdirectory(../ydlidar_sdk ydlidar_driver)


LINK_DIRECTORIES(${CMAKE_BINARY_DIR})
ADD_EXECUTABLE(${PROJECT_NAME}
               lidar.cpp)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ydlidar_driver)
