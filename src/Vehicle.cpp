//
// Created by Andreas Ntalakas on 03/11/2017.
//

#include <iostream>
#include "Vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"
#include "Trigonometry.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double v) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->v = v;
}

Vehicle::~Vehicle() {}

Vehicle::next_vals Vehicle::trajectory_for_state(
    int lane, Map &map, double ref_vel, int prev_size,
    vector<double> previous_path_x, vector<double> previous_path_y) {
  vector<double> ptsx;
  vector<double> ptsy;

  Vehicle::next_vals next_vals;

  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = trigonometry.deg2rad(this->yaw);

  if (prev_size < 2) {
    double prev_car_x = this->x - cos(this->yaw);
    double prev_car_y = this->y - sin(this->yaw);

    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);

    ptsx.push_back(this->x);
    ptsy.push_back(this->y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsy.push_back(ref_y_prev);

    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = map.getXY(this->s+30, (2+4*lane), map.waypoints_s, map.waypoints_x, map.waypoints_y);
  vector<double> next_wp1 = map.getXY(this->s+60, (2+4*lane), map.waypoints_s, map.waypoints_x, map.waypoints_y);
  vector<double> next_wp2 = map.getXY(this->s+90, (2+4*lane), map.waypoints_s, map.waypoints_x, map.waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsy.push_back(next_wp0[1]);

  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);

  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);

  for(int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
  }

  tk::spline s;

  s.set_points(ptsx, ptsy);

  for(int i = 0; i < previous_path_x.size(); i++) {
    next_vals.x.push_back((previous_path_x[i]));
    next_vals.y.push_back((previous_path_y[i]));
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for(int i = 1; i <= 50-previous_path_x.size(); i++) {
    double N = (target_dist/(.02*ref_vel/2.24));
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_vals.x.push_back(x_point);
    next_vals.y.push_back(y_point);
  }

  return next_vals;
}