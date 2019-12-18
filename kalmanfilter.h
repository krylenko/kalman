#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "../lib/libdfr-matrix/libdfr-matrix.h"

class KalmanFilter
{
public:
    KalmanFilter(const double deltaT_s);
    void init(const double initPos_m_, const double initVel_mps_);
    std::pair<double, double> predict();
    void observePos(const double observedPosition_m_, const double noise_);
    void observeVelo(const double observedVelocity_mps_, const double noise_);
    void update();
    double posVar();

private:

    double _deltaT_s;

    // Kalman filter elements
    Matrix _x; // estimated state
    Matrix _P; // estimated covariance

    Matrix _F; // state transition model
    Matrix _H; // observation model

    Matrix _W; // process noise
    Matrix _Q; // process noise covariance

    Matrix _V; // observation noise
    Matrix _R; // observation noise covariance

    Matrix _K; // Kalman gains
    Matrix _S; // innovation covariance

    Matrix _B; // control input model
    Matrix _U; // control input vector

    Matrix _z; // observation

};

#endif // KALMANFILTER_H
