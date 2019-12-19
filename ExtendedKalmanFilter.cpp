#include "ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(const Matrix initState_, const double deltaT_s_)
    : KalmanFilter(initState_, deltaT_s_)
{
}

std::pair<double, double> ExtendedKalmanFilter::predict()
{
}

void ExtendedKalmanFilter::observe(Matrix& observation_, Matrix& obsModel_)
{
}

void ExtendedKalmanFilter::update()
{
}
