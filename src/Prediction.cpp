//
// Created by Andreas Ntalakas on 08/11/2017.
//

#include <iostream>
#include "Prediction.h"
#include "math.h"

Prediction::Prediction() {
}

Prediction::~Prediction() {

}

void Prediction::Init() {
  too_close = false;
  lane_prohibited[0] = false;
  lane_prohibited[1] = false;
  lane_prohibited[2] = false;
}

double Prediction::DoPredict(int lane, int prev_size, double car_s, std::map<int, Snapshot> &sensor_fusion) {
  std::map<int, Snapshot>::iterator it = sensor_fusion.begin();

  while (it != sensor_fusion.end())
  {
    double d = it->second.d;

    double check_speed = sqrt(it->second.vx*it->second.vx + it->second.vy*it->second.vy);

    double check_car_s = it->second.s;

    if (d<(2+4*lane+2) && d>(2+4*lane-2)) {
      check_car_s += ((double)prev_size*.02*check_speed);

      if ((check_car_s > car_s) &&( (check_car_s-car_s) < 30)) {
        // Decide to leave safe buffer ahead in current lane
        too_close = true;
      }
    } else if (abs(check_car_s - car_s) < 20) {

      if (d<4 && d>0) {
        cout << "car_s: " << car_s << " s: " << it->second.s << " d: " << it->second.d << endl;
        lane_prohibited[0] = true;
      }
      if (d<8 && d>4) {
        lane_prohibited[1] = true;
      }
      if (d<12 && d>8) {
        lane_prohibited[2] = true;
      }
    }

    it++;
  }
}

