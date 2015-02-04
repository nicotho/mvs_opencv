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

add_executable(test_surf test_surf.cpp)
target_link_libraries(test_surf
    ${OpenCV_LIBS}
)


add_executable(test_sift test_sift.cpp)
target_link_libraries(test_sift   ${OpenCV_LIBS})

add_executable(test_ply test_ply.cpp)
target_link_libraries(test_ply   ${OpenCV_LIBS})

add_executable(test_reconstruct test_reconstruct.cpp)
target_link_libraries(test_reconstruct   ${OpenCV_LIBS})
