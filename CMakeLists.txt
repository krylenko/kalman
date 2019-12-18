cmake_minimum_required(VERSION 3.5)

project(kalman LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include
    /home/junkbox/code/lib/libdfr-matrix
    /home/junkbox/code/lib/libdfr-rv
    /home/junkbox/code/lib/libdfr-pfilt
    )

add_library(matrix
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-matrix/libdfr-matrix.h
    )

add_library(random_var
    ../lib/libdfr-rv/libdfr-rv.cpp
    ../lib/libdfr-rv/libdfr-rv.h
    )
target_link_libraries(random_var
    matrix
    )

add_executable(kalman
    kalmanfilter.cpp
    main.cpp
)

target_link_libraries(kalman
    matrix
    random_var
)

#add_executable(pfilt
#    ../lib/libdfr-pfilt/libdfr-pfilt.cpp
#    ../lib/libdfr-pfilt/pfTest.cpp
#    )

#target_link_libraries(pfilt
#    matrix
#    random_var
#    )
