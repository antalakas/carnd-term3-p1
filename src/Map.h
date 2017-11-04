//
// Created by Andreas Ntalakas on 04/11/2017.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <math.h>
#include <vector>
#include "Trigonometry.h"

using namespace std;

class Map {
public:
    /**
    * Map
    */
    Map();

    /**
    * Map
    */
    virtual ~Map();

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    vector<double> getXY(double s, double d, const vector<double> &maps_s,
                         const vector<double> &maps_x, const vector<double> &maps_y);

    double distance(double x1, double y1, double x2, double y2);

    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

    vector<double> getFrenet(double x, double y, double theta,
                             const vector<double> &maps_x, const vector<double> &maps_y);
};


#endif //PATH_PLANNING_MAP_H
