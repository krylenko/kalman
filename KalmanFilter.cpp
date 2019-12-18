#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(const double deltaT_s)
    : _deltaT_s(deltaT_s)
    , _x(2,1)       // estimated state (pos, velo)
    , _P(2,2)       // estimated covariance
    , _F(2,2)       // state update model for pos, velo
    , _H(2,2)       // observation vector (pos, no velo)
    , _W(2,1)       // process noise for pos, velo
    , _Q(2,2)       // process noise covariance
    , _V(2,1)       // observation noise
    , _R(2,2)       // observation noise covariance
    , _K(2,2)       // Kalman gains
    , _S(2,2)       // innovation covariance
    , _B(4,4)       // control input model
    , _U(4,1)       // control input vector
    , _z(2,1)       // latest observation
{
    _x[0][0] = 0.0;
    _x[0][1] = 0.0;

    _P[0][0] = 0.01;
    _P[0][1] = 0.0;
    _P[1][0] = 0.0;
    _P[1][1] = 0.01;

    _F[0][0] = 1.0;
    _F[0][1] = _deltaT_s;
    _F[1][0] = 0.0;
    _F[1][1] = 1;

    _H[0][0] = 1.0;
    _H[0][1] = 0.0;
    _H[1][0] = 0.0;
    _H[1][1] = 0.0;

    _W[0][0] = 0.04;
    _W[0][1] = 0.03;

    _Q[0][0] = 0.01;
    _Q[0][1] = 0.0;
    _Q[1][0] = 0.0;
    _Q[1][1] = 0.01;

    _V[0][0] = 0.0;
    _V[0][1] = 0.0;

    _R[0][0] = 0.01;
    _R[0][1] = 0.0;
    _R[1][0] = 0.0;
    _R[1][1] = 0.01;

    _K[0][0] = 1.0;
    _K[0][1] = 0.0;
    _K[1][0] = 0.0;
    _K[1][1] = 1.0;

    _S[0][0] = 0.01;
    _S[0][1] = 0.0;
    _S[1][0] = 0.0;
    _S[1][1] = 0.01;

    _z[0][0] = 0.0;
    _z[0][1] = 0.0;

}

void KalmanFilter::init(const double initPos_m_, const double initVel_mps_)
{
    _x[0][0] = initPos_m_;
    _x[1][0] = initVel_mps_;
}

std::pair<double, double> KalmanFilter::predict()
{
    // predict new state
    _x = _F*_x + _W;
    const double newPos = _x[0][0];
    const double newVelo = _x[1][0];

    // predict covariance
    _P = _F*_P*_F.T() + _Q;

    return std::make_pair(newPos, newVelo);
}

void KalmanFilter::observePos(const double observedPosition_m_, const double noise_)
{
    _x[0][0] = observedPosition_m_;
    _V[0][0] = noise_;

    _H[0][0] = 1.0;
    _H[0][1] = 0.0;
    _H[1][0] = 0.0;
    _H[1][1] = 0.0;

    _z = _H*_x + _V;
}

void KalmanFilter::observeVelo(const double observedVelocity_mps_, const double noise_)
{
    _x[1][0] = observedVelocity_mps_;
    _V[1][0] = noise_;

    _H[0][0] = 0.0;
    _H[0][1] = 1.0;
    _H[1][0] = 0.0;
    _H[1][1] = 1.0;

    _z = _H*_x + _V;
}

void KalmanFilter::update()
{
    Matrix ident(2,2);
    ident.I();
    Matrix y = _z - _H*_x;
    _S = _H*_P*_H.T() + _R;
    _K = _P*_H.T()*_S.inv();
    _x = _x + _K*y;
    _P = (ident - _K*_H)*_P;
}

double KalmanFilter::posVar()
{
    return _P[0][0];
}
