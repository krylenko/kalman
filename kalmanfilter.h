#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "../lib/libdfr-matrix/libdfr-matrix.h"

class KalmanFilter
{
public:
    KalmanFilter();
    double predict();
    void update();
private:

    // Kalman filter elements
    Matrix _x; // current state estimate
    Matrix _xPrime; // updated state estimate after 1 step forward

    Matrix _F; // state transition model
    Matrix _H; // observation model

    Matrix _W; // process noise
    Matrix _Q; // process noise covariance

    Matrix _V; // observation noise
    Matrix _R; // observation noise covariance

    Matrix _B; // control input model
    Matrix _U; // control input vector

};

#endif // KALMANFILTER_H
