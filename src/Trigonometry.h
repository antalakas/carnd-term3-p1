//
// Created by Andreas Ntalakas on 04/11/2017.
//

#ifndef PATH_PLANNING_TRIGONOMETRY_H
#define PATH_PLANNING_TRIGONOMETRY_H

#include <math.h>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

class Trigonometry {
public:
    /**
    * Map
    */
    Trigonometry();

    /**
    * Map
    */
    virtual ~Trigonometry();

    // For converting back and forth between radians and degrees.
    double deg2rad(double x);
    double rad2deg(double x);
};


#endif //PATH_PLANNING_TRIGONOMETRY_H
