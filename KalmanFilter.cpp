#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(const Matrix initState_, const double deltaT_s_)
{
    const double initCovariance = 0.01;

    // estimated state
    _x = initState_;
    const int stateRows = _x.rows();
    const int stateCols = _x.cols();

    // estimated covariance
    _P = Matrix(stateRows, stateRows);
    _P.fillDiag(initCovariance);

    // latest observation
    _z = Matrix(stateRows, stateCols);

    // control input model
    _B = Matrix(stateRows, stateRows);

    // control input vector
    _U = Matrix(stateRows, 1);

    // state update model for pos, velo
    _F = Matrix(stateRows, stateRows);
    _F[0][0] = 1.0;
    _F[0][1] = deltaT_s_;
    _F[1][1] = 1;

    // observation model
    _H = Matrix(stateRows, stateRows);

    // Kalman gains
    _K = Matrix(stateRows, stateRows);
    _K.fillDiag(1.0);

    // process noise covariance
    _Q = Matrix(stateRows, stateRows);
    _Q.fillDiag(initCovariance);

    // observation noise covariance
    _R = Matrix(stateRows, stateRows);
    _R.fillDiag(initCovariance);

    // innovation covariance
    _S = Matrix(stateRows, stateRows);
    _S.fillDiag(initCovariance);

}

std::pair<double, double> KalmanFilter::predict()
{
    // predict new state
    _x = _F*_x;
    const double newPos = _x[0][0];
    const double newVelo = _x[1][0];

    // predict covariance
    _P = _F*_P*_F.T() + _Q;

    return std::make_pair(newPos, newVelo);
}

void KalmanFilter::observe(Matrix& observation_, Matrix& obsModel_)
{
    if (observation_.rows() == _x.rows() && observation_.cols() == _x.cols()) {
        _H = obsModel_;
        _z = _H * observation_;
    }
}

void KalmanFilter::update()
{
    Matrix y = _z - _H*_x;
    _S = _H*_P*_H.T() + _R;
    _K = _P*_H.T()*_S.inv();
    _x = _x + _K*y;
    Matrix ident(2,2);
    ident.I();
    _P = (ident - _K*_H)*_P;
}

Matrix KalmanFilter::estVar()
{
    return _P;
}
