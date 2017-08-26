#include <iostream>
#include "vehicle.h"
#include "json.hpp"
#include "cost.h"
#include "helpers.h"
#include "spline.h"
#include <float.h>
#include <math.h>

using json = nlohmann::json;

using namespace ego;
using namespace ego_cost;
using namespace ego_help;

/**
 * Initialize Vehicle
 */
EgoCar::EgoCar(World world, Position position, EgoConfig config) {
  this->position = position;
  this->config = config;
  this->world = world;
}

OtherCar::OtherCar(World world, Position position, OtherConfig config) {
  this->world = world;
  this->position = position;
  this->config = config;
}

EgoCar::~EgoCar() {}
OtherCar::~OtherCar() {}

Snapshot EgoCar::getSnapshot() {
  Snapshot snap;
  snap.world = this->world;
  snap.position = this->position;
  snap.config = this->config;
  return snap;
}

void EgoCar::InitFromSnapshot(const Snapshot &snap) {
  this->world = snap.world;
  this->position = snap.position;
  this->config = snap.config;
}

Trajectory EgoCar::PlanTrajectory(const vector<OtherCar> &other_cars) {
  Trajectory t;
  State best_state = this->ChooseBestState(other_cars, &t);
  this->state = best_state;
  return t;
}

State EgoCar::ChooseBestState(const vector<OtherCar> &other_cars, Trajectory *best_t) {
  double best_cost = DBL_MAX;
  State best_state;

  // Go through the states one by one and calculate the cost for walking down
  // each state.
  Snapshot snap = this->getSnapshot();
  Snapshot best_snap;

  for ( int state = 1; state != ENUM_END; state++ ) {
    // For each state, the ego vehicle "imagines" following a trajectory
    Trajectory trajectory = this->CreateTrajectory((State)state, other_cars);

    double cost = CalculateCost(trajectory, other_cars);

    if (cost < best_cost) {
      cout << "Chosen state: " << State2Str(State(state)) << endl;
      best_state = (State)state;
      (*best_t) = trajectory;
      best_cost = cost;
      best_snap = this->getSnapshot();
    }
    this->InitFromSnapshot(snap);
  }
  cout << "chosen target speed 1: " << this->config.target_speed << endl;
  this->InitFromSnapshot(best_snap);
  cout << "chosen target speed 2: " << this->config.target_speed << endl;

  return STATE_KL;
}

Trajectory EgoCar::CreateTrajectory(State state, const vector<OtherCar> &other_cars) {
  Trajectory t;
  vector<double> x = {};
  vector<double> y = {};
  vector<double> s = {};
  vector<double> d = {};
  t.x = x;
  t.y = y;
  t.s = s;
  t.d = d;
  switch(state) {
    case STATE_CS: {
      break;
    }
    case STATE_KL: {
      this->RealizeKeepLane(other_cars);
      break;
    }
    case STATE_LCL: {
      // target_lane = this->RealizeLaneChangeLeft(1);
      break;
    }
    case STATE_LCR: {
      // target_lane = this->RealizeLaneChangeRight(1);
      break;
    }
    default: {

    }
  }

  double &car_x = this->position.x;
  double &car_y = this->position.y;
  double &car_length = this->config.car_length;
  double &car_s = this->position.s;

  // Reference velocity in mph.
  double &ref_v = this->config.target_speed;

  // Reference yaw in rad.
  double &ref_yaw = this->position.yaw;
  // double ref_yaw = 0.0;

  // Current lane: 0 - left, 1 - center, 2 - right
  int &lane = this->config.target_lane;

  json &previous_path_x = *this->config.previous_path_x;
  json &previous_path_y = *this->config.previous_path_y;

  int prev_size = previous_path_x.size();

  vector<double> &map_waypoints_s = *this->world.map_waypoints_s;
  vector<double> &map_waypoints_x = *this->world.map_waypoints_x;
  vector<double> &map_waypoints_y = *this->world.map_waypoints_y;

  // Number of waypoints.
  int &num_wp = this->config.num_wp;

  // Spline anchors
  int &spline_anchors = this->config.spline_anchors;
  double &anchor_distance = this->config.anchor_distance;

  double &dt = this->config.dt;

  // cout << "ego targets:\n"
  //      << "v: " << ref_v << "\n"
  //      << "ref_yaw: " << ref_yaw << "\n"
  //      << "lane: " << lane << endl;

  // Local-coordinates of waypoints (i.e. car position is [0,0])
  vector<double> localwp_x;
  vector<double> localwp_y;

  if (prev_size < 2) {
    // Create the initial two waypoints.
    // This is important otherwise the car would jump to nowhere.

    // Calculate previous position i.e. the position of
    // rear wheels.
    double prev_car_x = car_x - car_length * cos(ref_yaw);
    double prev_car_y = car_y - car_length * sin(ref_yaw);

    localwp_x.push_back(prev_car_x);
    localwp_x.push_back(car_x);
    localwp_y.push_back(prev_car_y);
    localwp_y.push_back(car_y);
  }
  else {
    // For subsequent steps, continue from previous path.

    car_x = previous_path_x[prev_size-1];
    car_y = previous_path_y[prev_size-1];

    // cout << "car_y from previous path: " << car_y << endl;

    double prev_car_x = previous_path_x[prev_size-2];
    double prev_car_y = previous_path_y[prev_size-2];
    ref_yaw = atan2(car_y - prev_car_y, car_x - prev_car_x);

    if (car_x > prev_car_x) {
      localwp_x.push_back(prev_car_x);
      localwp_x.push_back(car_x);
      localwp_y.push_back(prev_car_y);
      localwp_y.push_back(car_y);
    }
  }

  // Place anchor points. They are located far ahead of the car.
  for (int i = 1; i <= spline_anchors; ++i) {
    vector<double> next_xy = getXY(car_s+i*anchor_distance, lane2d(lane),
                                   map_waypoints_s, map_waypoints_x,
                                   map_waypoints_y);

    localwp_x.push_back(next_xy[0]);
    localwp_y.push_back(next_xy[1]);
  }

  // Shift and rotate reference to 0 degree and origin coordinate.
  for (int i = 0; i < localwp_x.size(); ++i) {
    double shift_x = localwp_x[i] - car_x;
    double shift_y = localwp_y[i] - car_y;
    localwp_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    localwp_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // On error "Assertion `m_x[i]<m_x[i+1]' failed.", get log from this code.
  // cout << "localwp [x, y] so far:" <<endl;
  // for (int i = 0; i < localwp_y.size(); i++) {
  //   cout << localwp_x[i] << ", " << localwp_y[i] << endl;
  // }

  // Spline Local
  tk::spline spline_l;
  spline_l.set_points(localwp_x, localwp_y);

  // Re-include previous waypoints if any.
  for (int i = 0; i < previous_path_x.size(); ++i) {
    t.x.push_back(previous_path_x[i]);
    t.y.push_back(previous_path_y[i]);
    // TODO: Also include s and d of trajectory.
  }

  // Create target position in front of the car
  double target_x = 30.0;
  double target_y = spline_l(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
  double x_add_on = 0;

  // The path between the car and the target contains several points.
  // In the code below we place these points onto this path.
  for (int i = 0; i < num_wp - previous_path_x.size(); ++i) {
    double point_dist = (target_dist/(dt * mph2mps(ref_v)));
    double point_x = x_add_on + (target_x)/point_dist;
    double point_y = spline_l(point_x);

    x_add_on = point_x;

    // Rotate back to world coordinates.
    double temp_x = point_x;
    double temp_y = point_y;
    point_x = (temp_x * cos(ref_yaw) - temp_y * sin(ref_yaw));
    point_y = (temp_x * sin(ref_yaw) + temp_y * cos(ref_yaw));

    point_x += car_x;
    point_y += car_y;

    // cout << endl << "point_x is " << point_x << endl; 

    t.x.push_back(point_x);
    t.y.push_back(point_y);
    // TODO: Also add s and d of trajectory.
  }

  // cout << endl << "trajectory [x, y]: " << endl;
  // for (int i; i < t.x.size(); ++i) {
  //   cout << t.x[i] << ", " << t.y[i] << endl;
  // }
  // cout << endl;

  return t;
}

void EgoCar::RealizeKeepLane(const vector<OtherCar> &other_cars) {
  // Set max acceleration to the car in front of ego car.
  this->config.target_speed = this->TargetSpeedForLane(
    other_cars, this->config.target_lane, this->position.s);
}

bool EgoCar::is_behind(const OtherCar &car) {
  // TODO: Does higher s always means in front of the car?
  return (this->position.s < car.position.s);
}

double EgoCar::TargetSpeedForLane(const vector<OtherCar> &other_cars,
                                  const int &lane, const double &s) {

  // Find the closest car in the same lane and ahead of the ego car.
  bool found_car = false;
  double distance = DBL_MAX;
  OtherCar closest_car;
  for (OtherCar const& car : other_cars) {
    if (car.config.target_lane == this->config.target_lane &&
        this->is_behind(car)) {
      // Initial setting, register the first car found.
      if (found_car == false) {
        closest_car = car;
      }
      double new_distance = abs(closest_car.position.s - this->position.s);

      if (new_distance < distance && new_distance < 60.0) {
        closest_car = car;
        found_car = true;
        cout << "closest car id: " << closest_car.config.id <<
                " distance (prev|new): " << distance << "|" <<
                new_distance << " car speed: " <<
                closest_car.position.v << endl;
        cout << "s(this|closest car): " << this->position.s << "|" <<
                closest_car.position.s << endl;
        distance = new_distance;
      }
    }
  }

  if (found_car == false) {
    return this->config.target_speed;  
  }
  else {
    cout << "Should set target speed to " << closest_car.position.v << endl;
    return closest_car.position.v;
  }
  
}

