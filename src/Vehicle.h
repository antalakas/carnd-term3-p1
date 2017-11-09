//
// Created by Andreas Ntalakas on 03/11/2017.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "Trigonometry.h"
#include "Map.h"

using namespace std;

class Vehicle {
public:
    /**
    * Constructor
    */
    Vehicle(double x, double y, double s, double d, double yaw, double v);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    struct next_vals {
        vector<double> x;
        vector<double> y;
    };

    int current_lane;
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double v;

    Trigonometry trigonometry;

    Vehicle::next_vals trajectory_for_state(
        int lane, double target, Map &map, double ref_vel, int prev_size,
        vector<double> previous_path_x, vector<double> previous_path_y);
};

#endif //PATH_PLANNING_VEHICLE_H
