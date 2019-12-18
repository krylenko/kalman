#include <iostream>
#include <cmath>

#include "../lib/libdfr-rv/libdfr-rv.h"
#include "../kalman/kalmanfilter.h"

std::pair<double, double> truePath(const double amplitude=1.0, const double speedFactor=1.0)
{
    static double phase = 0.0;
    const double output = amplitude * sin(phase);
    phase += speedFactor/3.14159;
    return std::make_pair(phase, output);
}

double noisyPath(const double truePoint, const double var=1.0)
{
    const double noise = rvGaussian(0.0, var);
    return truePoint + noise;
}

int main()
{
    rvSeed();
    KalmanFilter filter;
    for (unsigned int i=0; i<200; ++i) {
        std::pair<double, double> truePoint = truePath();
        double phase = std::get<0>(truePoint);
        double truePt = std::get<1>(truePoint);
        double noisyPoint = noisyPath(truePt, 0.06);
        double filterPoint = filter.predict();
        std::cout << phase << " " << truePt << " " << noisyPoint << " " << filterPoint << std::endl;
    }
    return 0;
}
