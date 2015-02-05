cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(PNAME mvs_cloud)
project(${PNAME})





add_executable(test_mvs camera.hpp test_mvs.cpp)
target_link_libraries(test_mvs    ${OpenCV_LIBS}
)

