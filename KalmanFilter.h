#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "../lib/libdfr-matrix/libdfr-matrix.h"

class KalmanFilter
{
public:
    KalmanFilter(const Matrix initState_, const double deltaT_s);
    std::pair<double, double> predict();
    void observe(Matrix& observation_, Matrix& obsModel_);
    void update();
    Matrix estVar();

private:

    // Kalman filter elements
    Matrix _z; // latest observation
    Matrix _x; // estimated state
    Matrix _P; // estimated covariance

    Matrix _B; // control input model
    Matrix _U; // control input vector

    Matrix _F; // state transition model
    Matrix _H; // observation model

    Matrix _K; // Kalman gains

    Matrix _Q; // process noise covariance
    Matrix _R; // observation noise covariance
    Matrix _S; // innovation covariance

};

#endif // KALMANFILTER_H
