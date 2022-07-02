//
// Created by Duan Bin on 2022/6/27.
//

#include <cmath>
#include <iostream>
#include <string>
#include <iomanip>
#include "particle_filter.h"


int main(){

    // Set up parameters here
    double delta_t = 0.1;  // Time elapsed between measurements [sec]
    double sensor_range = 50;  // Sensor range [m]

    // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_pos [3] = {0.3, 0.3, 0.01};
    // Landmark measurement uncertainty [x [m], y [m]]
    double sigma_landmark [2] = {0.3, 0.3};

    // Read map data
    Map map;
    if (!read_map_data("../data/map_data.txt", map)) {
        std::cout << "Error: Could not open map file" << std::endl;
        return -1;
    }

    vector<control_s> position_meas;
    if (!read_control_data("../data/control_data.txt", position_meas)) {
        std::cout << "Error: Could not open control file" << std::endl;
        return -1;
    }

    // Create particle filter
    ParticleFilter pf;
    vector<ground_truth> result_best_particle;
    for (int i = 0; i < 10; i++){
        if (!pf.initialized()) {
            // Sense noisy position data from the simulator
            double sense_x = std::stod("6.2785");
            double sense_y = std::stod("1.9598");
            double sense_theta = std::stod("0");

            pf.init(sense_x, sense_y, sense_theta, sigma_pos);
        } else{
            // Predict the vehicle's next state from previous
            //   (noiseless control) data.
            double previous_velocity = position_meas[i - 1].velocity;
            double previous_yawrate = position_meas[i - 1].yawrate;
//            cout<<previous_velocity<<','<<previous_yawrate<<endl;
            pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
        }


        stringstream ss;
        ss<<setw(6)<<setfill('0')<<i + 1;
        string fileNum;
        ss>>fileNum;
        string obs_file = "../data/observation/observations_" + fileNum + ".txt";
//        cout<<obs_file<<endl;
        vector<LandmarkObs> noisy_observations;
        if (!read_landmark_data(obs_file, noisy_observations)) {
            std::cout << "Error: Could not open observations file" << std::endl;
            return -1;
        }

        // Update the weights and resample
        pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
        pf.resample();

        vector<Particle> particles = pf.particles;

        double highest_weight = -1.0;
        ground_truth best_particle{};
        double weight_sum = 0.0;
        for (Particle p : particles) {
            if (p.weight > highest_weight) {
                highest_weight = p.weight;
                best_particle.x = p.x;
                best_particle.y = p.y;
                best_particle.theta = p.theta;
            }
            weight_sum += p.weight;
        }

        result_best_particle.push_back(best_particle);

        vector<ground_truth> gt;
        if (!read_gt_data("../data/gt_data.txt", gt)) {
            std::cout << "Error: Could not open gt file" << std::endl;
            return -1;
        }
        double * error;
        error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, best_particle.y, best_particle.theta);

//        cout<<best_particle.x<<' '<<best_particle.y<<' '<<best_particle.theta<<' ';
//        cout<<error[0]<<' '<<error[1]<<' '<<error[2]<<endl;

    }

    if (!write_best_particle_data("../result_data.txt", result_best_particle)) {
        std::cout << "Error: Could not write result file" << std::endl;
        return -1;
    }


    return 0;
}