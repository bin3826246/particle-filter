//
// Created by Duan Bin on 2022/6/27.
//

#include "particle_filter.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double *std) {
    num_particles = 100;
    particles.resize(num_particles);

    std::default_random_engine gen;
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    for (size_t i = 0; i < num_particles; ++i){
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1.0;
    }

    is_initialized = true;
}

ParticleFilter::~ParticleFilter() {

}

void ParticleFilter::prediction(double delta_t, double *std_pos, double velocity, double yaw_rate) {
    std::default_random_engine gen;
    std::normal_distribution<double> noisy_x(0, std_pos[0]);
    std::normal_distribution<double> noisy_y(0, std_pos[1]);
    std::normal_distribution<double> noisy_theta(0, std_pos[2]);

    for (size_t i = 0; i < num_particles; ++i) {
        double x0 = particles[i].x;
        double y0 = particles[i].y;
        double theta0 = particles[i].theta;

        double x_pre = 0.0;
        double y_pre = 0.0;
        double theta_pre = 0.0;

        if (fabs(yaw_rate) < 0.00001){
            x_pre = x0 + velocity * cos(theta0) * delta_t;
            y_pre = y0 + velocity * sin(theta0) * delta_t;
            theta_pre = theta0;
        } else{
            x_pre = x0 + velocity / yaw_rate * (sin(theta0 + yaw_rate * delta_t) - sin(theta0));
            y_pre = y0 + velocity / yaw_rate * (-cos(theta0 + yaw_rate * delta_t) + cos(theta0));
            theta_pre = theta0 + yaw_rate * delta_t;
        }

        // 统一角度至0-2π
        while (theta_pre > 2 * M_PI) theta_pre -= 2. * M_PI;
        while (theta_pre < 0.0) theta_pre += 2. * M_PI;

        particles[i].x = x_pre + noisy_x(gen);
        particles[i].y = y_pre + noisy_y(gen);
        particles[i].theta = theta_pre + noisy_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
    /*
     * Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
     */
    for (LandmarkObs& obs : observations) {
        double distance_min = std::numeric_limits<double>::max();
        for (LandmarkObs& pre : predicted) {
            double dis = dist(pre.x, pre.y, obs.x, obs.y);
//            double dis = (pre.x - obs.x) * (pre.x - obs.x) + (pre.y - obs.y) * (pre.y - obs.y);
            if (dis < distance_min){
                distance_min = dis;
                obs.id = pre.id;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, const double *std_landmark, const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    /*
     * Update the weights of each particle using a mult-variate Gaussian distribution.
     *
     * The observations are given in the VEHICLE'S coordinate system.
     *   1. 将车体坐标系下观测到的路标坐标转换到平面坐标系下
     *   2. 将观测的路标坐标数据与实际地图中的路标坐标匹配
     *   3. 根据匹配的结果更新权重
     */
    for (size_t i = 0; i < num_particles; ++i) {
        double xp = particles[i].x;
        double yp = particles[i].y;
        double thetap = particles[i].theta;

        // convert all observation in vehicle coordinate to map coordinate, without land mark id (id = 0)
        vector<LandmarkObs> obs_in_map(observations.size());
        for (size_t j = 0; j < observations.size(); ++j) {
            double xc = observations[j].x;
            double yc = observations[j].y;
            obs_in_map[j].x = xp + (cos(thetap) * xc) - (sin(thetap) * yc);
            obs_in_map[j].y = yp + (sin(thetap) * xc) + (cos(thetap) * yc);
//            obs_in_map[j].id = observations[j].id;
        }


        // build all landmark in sensor range, with land mark id
        vector<LandmarkObs> map_in_range;
        double distance_threshold = sensor_range * sensor_range;
        for(auto land_mark : map_landmarks.landmark_list){
            double dis = (land_mark.x_f - xp) * (land_mark.x_f - xp) + (land_mark.y_f - yp) * (land_mark.y_f - yp);
            if (dis <= distance_threshold){
                LandmarkObs map_obs{land_mark.id_i, land_mark.x_f, land_mark.y_f};
                map_in_range.push_back(map_obs);
            }
        }

        dataAssociation(map_in_range, obs_in_map);

        double std_x = std_landmark[0];
        double std_y = std_landmark[1];
        particles[i].weight = 1.0;

        // calculate the weight of particle
        // 利用二元高斯分布，将某个粒子下观测到的所有路标的可信度（即概率）累乘，得到这个粒子的权重
        for (auto obs : obs_in_map){
            Map::single_landmark_s landmark = map_landmarks.landmark_list[obs.id - 1];  //要减一是因为map_data文件里路标id是从1开始的
            double x = obs.x;
            double y = obs.y;
            double ux = landmark.x_f;
            double uy = landmark.y_f;
            double exponent = pow(x - ux, 2) / (2 * pow(std_x, 2)) + pow(y - uy, 2) / (2 * pow(std_y, 2));
            double p_xy = 1.0 / (2 * M_PI * std_x * std_y) * exp(-exponent);
            particles[i].weight *= p_xy;
        }

        weights.push_back(particles[i].weight);
    }
}

void ParticleFilter::resample() {
    /*
     * Resample particles with replacement with probability proportional
     * to their weight.
     *
     * 使用梅森旋转算法随机数生成器
     * 利用已计算出的粒子权重生成离散分布，权重的值会决定该权重粒子的出现概率
     * 在离散分布集合上随机采样，权重越大，被采样的可能性越大，从而生成新的粒子集合
     * 重采样思想：消除权值较小的粒子，对权值较大的粒子进行多份复制。重采样后每个粒子的权重相等
     */
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister 19937 generator
    std::discrete_distribution<> d{std::begin(weights), std::end(weights)};

    // build resampled particles
    vector<Particle> resampled_particles(num_particles);
    for (size_t i = 0; i < num_particles; ++i) {
        resampled_particles[i] = particles[d(gen)];
    }

    particles = resampled_particles;

    weights.clear();

}

void ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations, const std::vector<double> &sense_x, const std::vector<double> &sense_y) {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, std::string coord) {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
