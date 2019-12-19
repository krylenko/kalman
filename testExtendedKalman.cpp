#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

#include "../lib/libdfr-rv/libdfr-rv.h"
#include "../kalman/ExtendedKalmanFilter.h"

double truePathSin(const double deltaT_s_, const double amplitude_=1.0,
                   const double speedFactor_=1.0)
{
    static double phase = 0.0;
    const double phaseMult = speedFactor_/3.14159;
    phase += phaseMult; //* deltaT_s_;
    return amplitude_ * sin(phase);
}

int main()
{
    const unsigned int timesteps = 150;
    const unsigned int observationRatio = 1;
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

        // generate the real path
        truePos = truePathSin(deltaT_s, velocity_mps);

        auto estimate = filter.predict();
        const double filterPos = std::get<0>(estimate);
        const double filterVelo = std::get<1>(estimate);

        // generate noisy observations given sparsity ratio
        if (i % observationRatio == 0) {
            const double noise = rvGaussian(0.0, 0.1);
            const double noisyPos = truePos + noise;

            obs[0][0] = noisyPos;
            obs[1][0] = 0.0;
            filter.observe(obs, H_pos);

            /*
            // observe velocity in a sparser way
            if (i % (observationRatio * 2) == 0) {
                const double noiseScale = 0.75;
                const double noisyVelo = velocity_mps + noiseScale * noise;

                obs[0][1] = 0.0;
                obs[1][0] = noisyVelo;
                filter.observe(obs, H_vel);
                std::cerr << "v true " << velocity_mps << " noisy " << noisyVelo << " v est " << filterVelo << " err " << filterVelo - velocity_mps << std::endl;
            }
            */

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
