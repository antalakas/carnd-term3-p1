#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "Map.h"
#include "Vehicle.h"
#include "Prediction.h"
#include <chrono>

using  ns = chrono::nanoseconds;
using get_time = chrono::steady_clock ;

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;
  Trigonometry trigonometry;
  Map map;
  Prediction prediction;

  int lane = 1;
  double ref_vel = 0;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map.waypoints_x.push_back(x);
    map.waypoints_y.push_back(y);
    map.waypoints_s.push_back(s);
    map.waypoints_dx.push_back(d_x);
    map.waypoints_dy.push_back(d_y);
  }

  double speed_limit = 49.5;
  double desired_speed = speed_limit;

  int state = 0;

  h.onMessage([&map, &prediction, &trigonometry, &lane, &ref_vel, &speed_limit, &desired_speed, &state](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      //auto sdata = string(data).substr(0, length);
      //cout << sdata << endl;
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasData(data);

        if (s != "") {
          auto j = json::parse(s);

          string event = j[0].get<string>();

          if (event == "telemetry") {
            // j[1] is the data JSON object

            // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            std::map< int, Prediction::Snapshot > sensor_snapshots;
            for (int i=0; i<sensor_fusion.size(); i++) {
              Prediction::Snapshot snapshot;
              snapshot.id = sensor_fusion[i][0];
              snapshot.x = sensor_fusion[i][1];
              snapshot.y = sensor_fusion[i][2];
              snapshot.vx = sensor_fusion[i][3];
              snapshot.vy = sensor_fusion[i][4];
              snapshot.s = sensor_fusion[i][5];
              snapshot.d = sensor_fusion[i][6];
              snapshot.diff = 7000;

              sensor_snapshots.insert(std::make_pair(snapshot.id, snapshot));
            }

            json msgJson;

            int prev_size = previous_path_x.size();

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            if (prev_size > 0) {
              car_s = end_path_s;
            }

//            auto start = get_time::now();

            prediction.Init(speed_limit);
            prediction.DoPredict(lane, prev_size, car_speed, car_s, sensor_snapshots);

            // Control
            if (prediction.too_close) {

              if (state == 0) {
                desired_speed = prediction.check_speed;
                state = 1;
              }

              ref_vel -= 0.224;
            }
            else if (ref_vel < desired_speed) {
              ref_vel += 0.224;
            }

            Vehicle ego(car_x, car_y, car_s, car_d, car_yaw, car_speed);
            vector<Vehicle::next_vals> possible_trajectories;

            switch (lane) {
              case 0:
                if (state == 1) {
                  if (!prediction.lane_prohibited[1]) {
                    lane = 1;
                    state = 0;
                    desired_speed = speed_limit;
                  }
                }
                break;
              case 1:
                if (state == 1) {
                  if (!prediction.lane_prohibited[0]) {
                    lane = 0;
                    state = 0;
                    desired_speed = speed_limit;
                  } else if (!prediction.lane_prohibited[2]) {
                    lane = 2;
                    state = 0;
                    desired_speed = speed_limit;
                  }
                }
                break;
              case 2:
                if (state == 1) {
                  if (!prediction.lane_prohibited[1]) {
                    lane = 1;
                    state = 0;
                    desired_speed = speed_limit;
                  }
                }
                break;
            }

            possible_trajectories.push_back(
                ego.trajectory_for_state(lane, 30.0, map, ref_vel, prev_size, previous_path_x, previous_path_y));

//          auto end = get_time::now();
//          cout<<"Elapsed time is :  "<< chrono::duration_cast<ns>(end - start).count()<<" ns "<<endl;

            msgJson["next_x"] = possible_trajectories[0].x;
            msgJson["next_y"] = possible_trajectories[0].y;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

//          for(int i = 0; i < 50; i++) {
//            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//          }
//
//          double dist_inc = 0.5;
//          for(int i = 0; i < 50; i++)
//          {
//            double next_s = car_s + (i+1) * dist_inc;
//            double next_d = 6;
//
//            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//
//            next_x_vals.push_back(xy[0]);
//            next_y_vals.push_back(xy[1]);
//          }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
