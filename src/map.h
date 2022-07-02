//
// Created by Duan Bin on 2022/6/27.
//

#ifndef MAP_H
#define MAP_H

#include <vector>

class Map{
public:
    struct single_landmark_s {
        int id_i ; // Landmark ID
        float x_f; // Landmark x-position in the map (global coordinates)
        float y_f; // Landmark y-position in the map (global coordinates)
    };

    std::vector<single_landmark_s> landmark_list; // List of landmarks in the map
};

#endif //MAP_H
