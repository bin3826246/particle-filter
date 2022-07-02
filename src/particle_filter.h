//
// Created by Duan Bin on 2022/6/27.
//

#ifndef PARTICLE_PARTICLE_H
#define PARTICLE_PARTICLE_H

#include <string>
#include <vector>
#include "helper_functions.h"

using namespace std;

struct Particle{
    int id;
    double x;
    double y;
    double theta;
    double weight;
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
};

class ParticleFilter{
public:
    ParticleFilter() : num_particles(0), is_initialized(false) {}
    ~ParticleFilter();

    /*
    * init Initializes particle filter by initializing particles to Gaussian
            *   distribution around first position and all the weights to 1.
    * @param x Initial x position [m] (simulated estimate from GPS)
    * @param y Initial y position [m]
    * @param theta Initial orientation [rad]
    * @param std[] Array of dimension 3 [standard deviation of x [m],
    *   standard deviation of y [m], standard deviation of yaw [rad]]
    */
    void init(double x, double y, double theta, double std[]);

    void prediction(double delta_t, double std_pos[], double velocity,
                    double yaw_rate);

    void dataAssociation(std::vector<LandmarkObs> predicted,
                         std::vector<LandmarkObs>& observations);

    void updateWeights(double sensor_range, const double std_landmark[],
                       const std::vector<LandmarkObs> &observations,
                       const Map &map_landmarks);

    void resample();

    void SetAssociations(Particle& particle, const std::vector<int>& associations,
                         const std::vector<double>& sense_x,
                         const std::vector<double>& sense_y);

    const bool initialized() const {
        return is_initialized;
    }
    /**
   * Used for obtaining debugging information related to particles.
   */
    std::string getAssociations(Particle best);
    std::string getSenseCoord(Particle best, std::string coord);

    // Set of current particles
    std::vector<Particle> particles;
private:
    // Number of particles to draw
    int num_particles;

    // Flag, if filter is initialized
    bool is_initialized;

    // Vector of weights of all particles
    std::vector<double> weights;
};

#endif //PARTICLE_PARTICLE_H
