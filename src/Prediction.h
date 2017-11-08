//
// Created by Andreas Ntalakas on 08/11/2017.
//

#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include <map>
#include <vector>

using namespace std;

class Prediction {
public:
    /**
    * Prediction
    */
    Prediction();

    /**
    * Prediction
    */
    virtual ~Prediction();

    bool too_close;
    bool lane_prohibited[3];

    struct Snapshot {
        double x;
        double y;
        double vx;
        double vy;
        double s;
        float d;
    };

public:
    void Init();
    double DoPredict(int lane, int prev_size, double car_s, std::map<int, Snapshot> &sensor_snapshots);
};


#endif //PATH_PLANNING_PREDICTION_H
