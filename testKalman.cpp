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
    const unsigned int timesteps = 1500;
    const unsigned int observationRatio = 11;
    double truePos = 0.0;
    double velocity_mps = 1.0;
    double deltaT_s = 0.5;
    std::vector<double> errTracker;

    Matrix initState(2,1);
    initState[0][0] = truePos;
    initState[1][0] = velocity_mps;

    Matrix obs(2,1);
    Matrix H_pos(2,2), H_vel(2,2);
    H_pos[0][0] = 1.0;
    H_vel[0][1] = 1.0;
    H_vel[1][1] = 1.0;

    rvSeed();
    KalmanFilter filter(initState, deltaT_s);
    Matrix P = filter.estVar();
    std::cerr << P[0][0] << " " << P[0][1] << std::endl;
    std::cerr << P[1][0] << " " << P[1][1] << std::endl;

    for (unsigned int i=0; i<timesteps; ++i) {

        // change velocity a couple of times to make things interesting
        if (i >= 95 && i < 350) {
            velocity_mps = 0.25;
        } else if (i >= 350 && i < 750) {
            velocity_mps = 2.3;
        } else if (i >= 650 && i < 950) {
            velocity_mps = 0.0;
        } else if (i >= 950) {
            velocity_mps = -1.2;
        }

        // generate the real path
        truePos = truePathLinear(deltaT_s, velocity_mps);
        auto estimate = filter.predict();
        const double filterPos = std::get<0>(estimate);         // always predict a position
        const double filterVelo = std::get<1>(estimate);

        // generate noisy observations given sparsity ratio
        if (i % observationRatio == 0) {
            const double noise = rvGaussian(0.0, 60.0);
            const double noisyPos = truePos + noise;

            /*
            // observe both state variables at once
            Matrix H(2,2);
            H[0][0] = 1.0;
            H[0][1] = 1.0;
            H[1][1] = 1.0;
            obs[0][0] = noisyPos;
            obs[1][0] = velocity_mps + 0.5 * noise;
            filter.observe(obs, H);
            */

            obs[0][0] = noisyPos;
            obs[1][0] = 0.0;
            filter.observe(obs, H_pos);

            // observe velocity in a sparser way
            if (i % (observationRatio * 2) == 0) {
                const double noiseScale = 0.75;
                const double noisyVelo = velocity_mps + noiseScale * noise;

                obs[0][1] = 0.0;
                obs[1][0] = noisyVelo;
                filter.observe(obs, H_vel);
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
    P = filter.estVar();
    std::cerr << P[0][0] << " " << P[0][1] << std::endl;
    std::cerr << P[1][0] << " " << P[1][1] << std::endl;

    return 0;
}
