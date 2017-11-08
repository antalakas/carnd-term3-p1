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
        int id;
        double x;
        double y;
        double vx;
        double vy;
        double speed;
        double s;
        float d;
        int lane;
        double diff;
        vector<double> pred;
    };

    // predictions for vehicles behind ego
    Snapshot snapshot[3];
    double check_speed;
    double speed_limit;
    int proposed_lane;

public:
    void Init(double speed_limit);
    void DoPredict(int lane, int prev_size, double car_speed, double car_s, std::map<int, Snapshot> &sensor_snapshots);
};


#endif //PATH_PLANNING_PREDICTION_H
