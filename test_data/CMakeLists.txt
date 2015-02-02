cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(PNAME test_data)
project(${PNAME})

set(SOURCES
   test_data.cpp
)

add_executable(${PNAME} ${SOURCES})
target_link_libraries(${PNAME}
    ${OpenCV_LIBS}
)

