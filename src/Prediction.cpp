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

void Prediction::Init(double speed_limit) {
  too_close = false;

  lane_prohibited[0] = false;
  lane_prohibited[1] = false;
  lane_prohibited[2] = false;

  snapshot[0] = {};
  snapshot[1] = {};
  snapshot[2] = {};

  check_speed = speed_limit;
  this->speed_limit = speed_limit;

  proposed_lane = -1;
}

void Prediction::DoPredict(int lane, int prev_size, double car_speed, double car_s, std::map<int, Snapshot> &sensor_fusion) {
  std::map<int, Snapshot>::iterator it = sensor_fusion.begin();

  while (it != sensor_fusion.end())
  {
    double d = it->second.d;

    double check_car_s = it->second.s;

    double check_speed = sqrt(it->second.vx*it->second.vx + it->second.vy*it->second.vy);
    it->second.speed = check_speed * 2.24;

    if (d<(2+4*lane+2) && d>(2+4*lane-2)) {
      check_car_s += ((double)prev_size*.02*check_speed);

      if ((check_car_s > car_s) &&( (check_car_s-car_s) < 30)) {
        // Decide to leave safe buffer ahead in current lane
        if (it->second.speed > this->speed_limit)
          this->check_speed = this->speed_limit;
        else
          this->check_speed = it->second.speed;

//        cout << "check_speed: " << this->check_speed << endl;
        too_close = true;
      }
    } else if (abs(check_car_s - car_s) < 20) {

      if (d<4 && d>0) {
//        cout << "car_s: " << car_s << " s: " << it->second.s << " d: " << it->second.d << endl;
        lane_prohibited[0] = true;
      }
      if (d<8 && d>4) {
        lane_prohibited[1] = true;
      }
      if (d<12 && d>8) {
        lane_prohibited[2] = true;
      }
    }

    // find the closest vehicle, behind ego vehicle
    double min_s[3] {7000.0, 7000.0, 7000.0};
    it->second.diff= car_s - check_car_s;

    if ((check_car_s < car_s) && ((it->second.diff) < 50)) {
      switch (lane) {
        case 0:
          if (d < 8 && d > 4) {
            it->second.lane = 1;
            if (it->second.diff < min_s[1]) {
              snapshot[1] = it->second;
              min_s[1] = it->second.diff;
            }
          }
          break;
        case 1:
          if (d < 4 && d > 0) {
            it->second.lane = 0;
            if (it->second.diff < min_s[0]) {
              snapshot[0] = it->second;
              min_s[0] = it->second.diff;
            }
          }
          if (d < 12 && d > 8) {
            it->second.lane = 2;
            if (it->second.diff < min_s[2]) {
              snapshot[2] = it->second;
              min_s[2] = it->second.diff;
            }
          }
          break;
        case 2:
          if (d < 8 && d > 4) {
            it->second.lane = 1;
            if (it->second.diff < min_s[1]) {
              snapshot[1] = it->second;
              min_s[1] = it->second.diff;
            }
          }
          break;
      }
    }

    // cout << "car_id_front: " << veh_id_in_front << " line: " << lane << " ref_vel:" << ref_vel << endl;
    for (int J=0; J<3; J++) {
      if (snapshot[J].id == 0)
        continue;

      if (snapshot[J].speed > car_speed)
        lane_prohibited[J] = true;

//      if (J==0) {
//        cout << "lane: " << J << " speed: " << snapshot[J].speed << " check_car_s: " << snapshot[J].s << " ego_car_speed: " << car_speed << " ego_car_s: " << car_s << endl;
//      }
      // 1sec
//      for (int I=0; I<50; I++) {
//        check_car_s += (.02 * check_speed);
//        cout << "time: " << .02 * (I+1) << " check_car_s: " << check_car_s << " ego_car_s: " << car_s << endl;
//        it->second.pred.push_back(check_car_s);
//      }

//      cout << "end of predictions calc" << endl << endl;
      // cout << "car_id_back: " << veh_id[J] << " lane: " << J << endl;
    }
    it++;
  }
}

