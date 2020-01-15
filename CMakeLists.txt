cmake_minimum_required(VERSION 3.5)

project(kalman LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include
    /home/junkbox/code/lib/libdfr-matrix
    /home/junkbox/code/lib/libdfr-rv
    )

add_executable(mtxTest
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-matrix/mtxTest.cpp
)

add_executable(rvTest
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-rv/libdfr-rv.cpp
    ../lib/libdfr-rv/rvTest.cpp
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
