cmake_minimum_required(VERSION 3.5)

project(kalman LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

include_directories(include
    /home/junkbox/code/lib/libdfr-matrix
    /home/junkbox/code/lib/libdfr-rv
    /home/junkbox/code/lib/libdfr-neural
    )

#add_executable(testMtx
#    ../lib/libdfr-matrix/libdfr-matrix.cpp
    #../lib/libdfr-matrix/testMatrix.cpp
#add_executable(mtxTest
#    ../lib/libdfr-matrix/libdfr-matrix.cpp
#    ../lib/libdfr-matrix/mtxTest.cpp
#)

#add_executable(rvTest
#    ../lib/libdfr-matrix/libdfr-matrix.cpp
#    ../lib/libdfr-rv/libdfr-rv.cpp
#    ../lib/libdfr-rv/rvTest.cpp
#)

add_executable(xorTest
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-neural/dfrNeuralNet.cpp
    ../lib/libdfr-neural/dfrNeuralLayer.cpp
    ../lib/libdfr-neural/dfrNeuralLinearLayer.cpp
    ../lib/libdfr-neural/dfrNeuralTanhLayer.cpp
    ../lib/libdfr-neural/dfrNeuralSigmoidLayer.cpp
    ../lib/libdfr-neural/dfrNeuralSoftmaxLayer.cpp
    ../lib/libdfr-neural/dfrNeuralTestWrapper_XOR.cpp
)

add_executable(mnistTest
    ../lib/libdfr-matrix/libdfr-matrix.cpp
    ../lib/libdfr-neural/dfrNeuralNet.cpp
    ../lib/libdfr-neural/dfrNeuralLayer.cpp
    ../lib/libdfr-neural/dfrNeuralLinearLayer.cpp
    ../lib/libdfr-neural/dfrNeuralTanhLayer.cpp
    ../lib/libdfr-neural/dfrNeuralSigmoidLayer.cpp
    ../lib/libdfr-neural/dfrNeuralSoftmaxLayer.cpp
    ../lib/libdfr-neural/dfrNeuralTestWrapper_MNISTdigit.cpp
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
