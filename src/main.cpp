#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include <string>
#include "helpers.h"
#include "spline.h"
#include "cost.h"
#include "vehicle.h"
#include <ctime>

using namespace std;
using namespace ego;
using namespace ego_help;

// for convenience
using json = nlohmann::json;

int main(int argc, char *argv[]) {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from

  string map_file_ = "../data/highway_map.csv";
  if (argc > 1 && strcmp(argv[1], "-b") == 0) {
    map_file_ = "../data/highway_map_bosch1.csv";
    cout << "\nLoading ../data/highway_map_bosch1.csv\n";
    cout << "Download Bosch simulator from https://github.com/udacity/Bosch-Challenge/releases\n";
  }
  else {
    cout << "\nLoading ../data/highway_map.csv\n";
    cout << "Download Udacity simulator from https://github.com/udacity/self-driving-car-sim/releases\n";
    cout << "Find \"Term 3 Simulator\" there.\n";
    cout << "To load Bosch data, run with -b\n";
  }

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  World world;
  world.map_waypoints_x = &map_waypoints_x;
  world.map_waypoints_y = &map_waypoints_y;
  world.map_waypoints_s = &map_waypoints_s;
  world.map_waypoints_dx = &map_waypoints_dx;
  world.map_waypoints_dy = &map_waypoints_dy;

  // State of the last Ego Car object.
  State previous_state = STATE_KL;

  // init(*world.map_waypoints_s, *world.map_waypoints_x, *world.map_waypoints_y);

  h.onMessage([&world, &previous_state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // Initial experiment - see if frenet conversion works:
    // TestFrenetConversion1(world);
    // exit(0);

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
          // clock_t begin = clock();
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

          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // Each car contains this data: [id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          json msgJson;

          int prev_size = previous_path_x.size();

          // Time each simulation step in seconds.
          double dt = 0.02;

          // Length between front and rear wheels in meter.
          double car_length = 1.0;

          // ---INIT EGO---
          EgoCar ego;
          EgoConfig ego_config;
          ego_config.default_target_speed = mph2mps(49.5);
          ego_config.default_max_acceleration = 8.0;
          ego_config.target_speed = mph2mps(49.5); // mps
          ego_config.dt = dt;
          ego_config.car_length = car_length;
          ego_config.max_jerk = 3.0; // m/s^3
          ego_config.previous_path_x = &previous_path_x;
          ego_config.previous_path_y = &previous_path_y;
          ego_config.end_path_s = &end_path_s;
          ego_config.end_path_d = &end_path_d;
          ego_config.target_lane = d2lane(car_d);

          // Number of waypoints.
          ego_config.num_wp = 40;

          // Number of plan points. Higher value means
          // farther road segment to predict.
          // If this value is too low, the car does not
          // predict the environment after lane change,
          // but if it is too high, the prediction becomes
          // less relevant to the car.
          ego_config.num_pp = 25;

          // ego_config.num_last_path = 50;

          // Horizon and spline anchors are used to create
          // temporary trajectory that starts from the end of
          // a waypoints line.
          ego_config.spline_anchors = 3;
          ego_config.horizon = 30.0;
          ego_config.anchor_ddist_threshold = 2.0;

          // target_x points to horizon when there is nothing ahead,
          // but otherwise set this to a car in front of ego car.
          ego_config.target_x = ego_config.horizon;           

          Position pos;
          pos.x = car_x;
          pos.y = car_y;
          pos.s = car_s;
          pos.d = car_d;

          // Find car speed based on previous positions.
          // The car WILL break without this!
          if (prev_size < 2) {
            pos.vx = 0;
            pos.vy = 0;
            pos.a = 0;
          }
          else {
            double x1 = (double)previous_path_x[prev_size-1];
            double x2 = (double)previous_path_x[prev_size-2];
            double y1 = (double)previous_path_y[prev_size-1];
            double y2 = (double)previous_path_y[prev_size-2];
            pos.vx = (x1 - x2)/dt;
            pos.vy = (y1 - y2)/dt;
            pos.v = sqrt((pos.vx*pos.vx) + (pos.vy*pos.vy));

            if (prev_size >= 3) {
              double x3 = (double)previous_path_x[prev_size-3];
              double y3 = (double)previous_path_y[prev_size-3];
              double vx1 = (x2 - x3)/dt;
              double vy1 = (y2 - y3)/dt;
              pos.a = distance(pos.vx, pos.vy, vx1, vy1)/dt;
            }
          }

          // cout << "ego car vx, vy: " << pos.vx << ", " << pos.vy << endl;

          pos.yaw = deg2rad(car_yaw);

          ego = EgoCar(world, pos, ego_config);
          ego.state = previous_state;
          // ---END---

          // ---INIT OTHER VEHICLES---
          vector<OtherCar> other_cars;
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            OtherConfig car_config;
            car_config.dt = dt;
            car_config.id = sensor_fusion[i][0];

            // In a real project, car length should depend on
            // sensor fusion data.
            car_config.car_length = car_length;
            Position pos;
            pos.x = sensor_fusion[i][1];
            pos.y = sensor_fusion[i][2];
            pos.s = sensor_fusion[i][5];
            pos.d = sensor_fusion[i][6];
            pos.vx = sensor_fusion[i][3];
            pos.vy = sensor_fusion[i][4];
            pos.v = sqrt((pos.vx * pos.vx) + (pos.vy * pos.vy));
            pos.yaw = 0.0;
            car_config.target_lane = d2lane(pos.d);
            OtherCar car = OtherCar(world, pos, car_config);
            other_cars.push_back(car);
          }
          // ---END---

          CostWeights weights;
          weights.collision = 1.0;
          weights.efficiency = 0.1;
          weights.max_accel = 0.8;
          weights.max_jerk = 0.5;
          weights.change_state = 0.5;

          Trajectory best_trajectory = ego.PlanTrajectory(other_cars, weights);
          previous_state = ego.state;

          // cout << "\rhellow" << flush;
          // cout << "\rcar1 x: " << other_cars[0].position.x << flush;
          // cout << "\rcar1 y: " << other_cars[0].position.y << flush;
            // "\nv: " << car_speed <<
            // "\nvx: " << ego.position.vx <<
            // "\nvy " << ego.position.vy << flush;

          // cout << "Current state: " << State2Str(ego.state) << endl;

          for (int i = 0; i < best_trajectory.x.size(); ++i) {
            next_x_vals.push_back(best_trajectory.x[i]);
            next_y_vals.push_back(best_trajectory.y[i]);
          }

          // cout << next_x_vals[0] << " to "
          //   << next_x_vals[next_x_vals.size()-1] << endl;

          // cout << endl << "final [x, y] (" << next_x_vals.size() << ") " << endl;
          // for (int i=0; i < next_x_vals.size(); ++i) {
          //   cout << next_x_vals[i] << ", " << next_y_vals[i] << endl;
          // }
          // cout << endl;

          // ===END===
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          // this_thread::sleep_for(chrono::milliseconds(1000));

          // clock_t end = clock();
          // double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
          // cout << "time: " << elapsed_secs << endl;

        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else {
        json msgJson;
        msgJson["next_x"] = {};
        msgJson["next_y"] = {};
        auto msg = "42[\"control\","+ msgJson.dump()+"]";
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