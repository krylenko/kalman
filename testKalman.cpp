#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

#include "../lib/libdfr-rv/libdfr-rv.h"
#include "../kalman/KalmanFilter.h"

std::pair<double, double> truePathSin(const double amplitude=1.0, const double speedFactor=1.0)
{
    static double phase = 0.0;
    const double output = amplitude * sin(phase);
    phase += speedFactor/3.14159;
    return std::make_pair(phase, output);
}

double truePathLinear(const double deltaT_s_, const double velocity_mps_)
{
    static double position = 0.0;
    const double newPos = position + velocity_mps_ * deltaT_s_;
    position = newPos;
    return newPos;
}

double noisyPath(const double truePoint_, const double var_=1.0)
{
    const double noise = rvGaussian(0.0, var_);
    return truePoint_ + noise;
}

int main()
{
    const unsigned int timesteps = 1000;
    const unsigned int observationRatio = 13;
    double truePos = 0.0;
    double velocity_mps = 1.0;
    double deltaT_s = 0.5;
    std::vector<double> errTracker;

    rvSeed();
    KalmanFilter filter(deltaT_s);
    filter.init(truePos, velocity_mps);

    for (unsigned int i=0; i<timesteps; ++i) {

        // change velocity a couple of times to make things interesting
        if (i >= 95 && i < 350) {
            velocity_mps = 0.25;
        } else if (i >= 350 && i < 750) {
            velocity_mps = 2.3;
        } else if (i >= 650) {
            velocity_mps = 0.0;
        }

        // generate the real path
        truePos = truePathLinear(deltaT_s, velocity_mps);
        auto estimate = filter.predict();
        const double filterPos = std::get<0>(estimate);         // always predict a position
        const double filterVelo = std::get<1>(estimate);

        // generate noisy observations given sparsity ratio
        if (i % observationRatio == 0) {
            const double noise = rvGaussian(0.0, 15.3);
            const double noisyPos = truePos + noise;
            filter.observePos(noisyPos, noise);

            // observe velocity in a sparser way
            if (i % (observationRatio * 2) == 0) {
                const double noiseScale = 0.1;
                const double noisyVelo = velocity_mps + noiseScale * noise;
                filter.observeVelo(noisyVelo, noiseScale * noise);
                std::cerr << "v true " << velocity_mps << " noisy " << noisyVelo << " v est " << filterVelo << " err " << filterVelo - velocity_mps << std::endl;
            }

            filter.update();                                    // only update on observations
            const double error = filterPos - truePos;
            errTracker.push_back(abs(error));
            const double errMean = std::accumulate(errTracker.begin(), errTracker.end(), 0) / static_cast<double>(errTracker.size());
            std::cout << i << " " << truePos << " " << noisyPos << " " << filterPos << " err " << error << " err mean " << errMean << std::endl;

        } else {
            std::cout << i << " " << truePos << " " << " " << std::endl;
        }
    }

    return 0;
}
