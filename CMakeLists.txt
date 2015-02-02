cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(mvs_opencv)


# compile and build setup
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
LIST(APPEND CMAKE_CXX_FLAGS "-std=c++11 -O3 -ffast-math -Wall")

## using OpenCV for image i/o (at least)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
cuda_include_directories(${OpenCV_INCLUDE_DIRS})

# debug

message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")


# add all the subprojects
add_subdirectory(test_data)
