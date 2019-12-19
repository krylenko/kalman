#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include "../lib/libdfr-matrix/libdfr-matrix.h"
#include "KalmanFilter.h"

class ExtendedKalmanFilter: public KalmanFilter
{
public:
    ExtendedKalmanFilter(const Matrix initState_, const double deltaT_s);
    std::pair<double, double> predict();
    void observe(Matrix& observation_, Matrix& obsModel_);
    void update();

private:


};

#endif // EXTENDEDKALMANFILTER_H
