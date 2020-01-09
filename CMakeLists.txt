cmake_minimum_required(VERSION 3.5)

project(kalman LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

include_directories(include
    /home/junkbox/code/lib/libdfr-matrix
    /home/junkbox/code/lib/libdfr-rv
    )

add_executable(testMtx
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-matrix/testMatrix.cpp
)

add_executable(kalman
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-rv/libdfr-rv.cpp
    KalmanFilter.cpp
    testKalman.cpp
)

add_executable(extKalman
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-rv/libdfr-rv.cpp
    KalmanFilter.cpp
    ExtendedKalmanFilter.cpp
    testExtendedKalman.cpp
)
