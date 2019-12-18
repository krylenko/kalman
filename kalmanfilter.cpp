#include "kalmanfilter.h"

KalmanFilter::KalmanFilter()
    : _x(3)
    , _xPrime(3)
    , _F(3,3)
    , _H(4,4)
    , _W(4,1)
    , _Q(4,4)
    , _V(4,1)
    , _R(4,4)
    , _B(4,4)
    , _U(4,1)
{

}

double KalmanFilter::predict()
{
    return 0.2;
}

void KalmanFilter::update()
{

}
