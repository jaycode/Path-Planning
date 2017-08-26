#include <iostream>
#include "vehicle.h"
#include "json.hpp"
#include "cost.h"
#include "helpers.h"
#include "spline.h"
#include <float.h>
#include <math.h>
#include <tuple>

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

Trajectory EgoCar::PlanTrajectory(const vector<OtherCar> &other_cars,
                                  const CostWeights &weights) {
  Trajectory t;
  State best_state = this->ChooseBestState(other_cars, weights, &t);
  this->state = best_state;
  return t;
}

State EgoCar::ChooseBestState(const vector<OtherCar> &other_cars,
                              const CostWeights &weights,
                              Trajectory *best_t) {
  double best_cost = DBL_MAX;
  State best_state;

  // Go through the states one by one and calculate the cost for walking down
  // each state.
  Snapshot snap = this->getSnapshot();
  Snapshot best_snap;

  Trajectory current_best_t;

  for ( int state = 1; state != ENUM_END; state++ ) {
    // For each state, the ego vehicle "imagines" following a trajectory
    Trajectory trajectory = this->CreateTrajectory((State)state, other_cars);

    // TODO: Looks like reinforcement learning can be implemented here
    //       by treating (state + other_cars) as a state
    tuple<State, vector<OtherCar>> cf_state = make_tuple(State(state),
                                                         other_cars);
    double cost = CalculateCost(cf_state, trajectory, weights);

    if (cost < best_cost) {
      // cout << "Chosen state: " << State2Str(State(state)) << endl;
      best_state = (State)state;
      current_best_t = trajectory;
      best_cost = cost;
      best_snap = this->getSnapshot();
    }
    this->InitFromSnapshot(snap);
  }
  (*best_t) = current_best_t;
  this->InitFromSnapshot(best_snap);
  // cout << "chosen target speed: " << this->config.target_speed << endl;
  cout << "Target lane: " << this->config.target_lane << " or in d: " << lane2d(this->config.target_lane) << endl;

  return (State)state;
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

  Position ref;
  ref.x = this->position.x;
  ref.y = this->position.y;
  ref.yaw = this->position.yaw;
  ref.v = this->config.target_speed;

  double &car_length = this->config.car_length;

  // Current velocity in meter/second.
  double &cur_v = this->position.v;

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

  double &max_a = this->config.max_acceleration;


  // cout << "ego targets:\n"
  //      << "v: " << ref_v << "\n"
  //      << "ref_yaw: " << ref_yaw << "\n"
  //      << "lane: " << lane << endl;

  // Local-coordinates of waypoints (i.e. car position is [0,0])
  vector<double> localwp_x;
  vector<double> localwp_y;

  // cout << "car position [x,y]|[s,d]: [" << ref_x << ", " << ref_y <<
  //         "][" << car_s << ", " << car_d << "]" << endl;

  // cout << "Checking conversion accuracy" << endl;
  // cout << "Car s, d converted into x, y: " << endl;
  // vector<double> cur_xy = getXY(car_s, car_d,
  //                                map_waypoints_s, map_waypoints_x,
  //                                map_waypoints_y);
  // cout << cur_xy[0] << ", " << cur_xy[1] << endl;

  if (prev_size < 2) {
    // Create the initial two waypoints.
    // This is important otherwise the car would jump to nowhere.

    // Calculate previous position i.e. the position of
    // rear wheels.
    double prev_car_x = ref.x - car_length * cos(ref.yaw);
    double prev_car_y = ref.y - car_length * sin(ref.yaw);

    localwp_x.push_back(prev_car_x);
    localwp_x.push_back(ref.x);
    localwp_y.push_back(prev_car_y);
    localwp_y.push_back(ref.y);
  }
  else {
    // For subsequent steps, continue from previous path.

    ref.x = previous_path_x[prev_size-1];
    ref.y = previous_path_y[prev_size-1];

    double prev_ref_x = previous_path_x[prev_size-2];
    double prev_ref_y = previous_path_y[prev_size-2];
    ref.yaw = atan2(ref.y - prev_ref_y, ref.x - prev_ref_x);

    if (ref.x > prev_ref_x) {
      localwp_x.push_back(prev_ref_x);
      localwp_x.push_back(ref.x);
      localwp_y.push_back(prev_ref_y);
      localwp_y.push_back(ref.y);
    }
  }
  vector<double> ref_sd = getFrenet(ref.x, ref.y, ref.yaw,
                                    map_waypoints_x, map_waypoints_y);
  ref.s = ref_sd[0];
  ref.d = ref_sd[1];

  // From this point on, use reference positions to decide on things.
  // In other words, make decisions based on the last point in waypoint.

  switch(state) {
    case STATE_CS: {
      break;
    }
    case STATE_KL: {
      this->RealizeKeepLane(other_cars, &ref);
      break;
    }
    case STATE_LCL: {
      if (d2lane(this->position.d) > 0) {
        this->RealizeLaneChange(other_cars, -1, &ref);
      }
      break;
    }
    case STATE_LCR: {
      if (d2lane(this->position.d) < 2) {
        this->RealizeLaneChange(other_cars, 1, &ref);
      }
      break;
    }
    default: {

    }
  }
  
  // Place anchor points. They are located ahead of the car.
  // cout << "Before pushing anchors, num of x: " << localwp_x.size() << endl;
  // cout << "x values:" << endl;
  for (int i=0; i<localwp_x.size(); ++i) {
    cout << localwp_x[i] << endl;
  }

  for (int i = 1; i <= spline_anchors; ++i) {
    vector<double> next_xy = getXY(ref.s+i*anchor_distance, lane2d(lane),
                                   map_waypoints_s, map_waypoints_x,
                                   map_waypoints_y);

    // cout << "push back x (" << "anchor dist: " <<
    //         anchor_distance << ", i: "<< i << "): " << next_xy[0] << endl;
    localwp_x.push_back(next_xy[0]);
    localwp_y.push_back(next_xy[1]);
  }

  // Shift and rotate reference to 0 degree and origin coordinate.
  for (int i = 0; i < localwp_x.size(); ++i) {
    double shift_x = localwp_x[i] - ref.x;
    double shift_y = localwp_y[i] - ref.y;
    localwp_x[i] = (shift_x * cos(0 - ref.yaw) - shift_y * sin(0 - ref.yaw));
    localwp_y[i] = (shift_x * sin(0 - ref.yaw) + shift_y * cos(0 - ref.yaw));
  }

  // Speed trajectory. We assume that the speed
  // follows a linear trajectory. From the calculation below
  // we get the total required acceleration to reach target speed.
  double total_time = dt * num_wp;
  double dv = ref.v - cur_v;
  double req_accel = dv / total_time;
  // cout << "n leftover waypoints: " << prev_size << endl;
  // cout << "car position [x,y]: [" << ref.x << ", " << ref.y <<
  //         "]"<< endl;
  // cout << "current v: " << cur_v << " target v: " << ref.v << endl;
  // cout << "total time (sec): " << total_time << endl;
  // cout << "required acceleration: " << req_accel << endl;
  // Since the car needs to adhere to a maximum acceleration,
  // we substract max acceleration from required acceleration
  // in each second.

  // Spline for position
  tk::spline spline_pos;

  // PruneWaypoint(&localwp_x, &localwp_y);

  // There are some problems with the trajectory e.g.
  // x points are not sorted or there are duplicates.
  // This could be caused by the car running too
  // slow.
  // cout << "Car speed at error: " << cur_v << endl;

  // cout << "num of x: " << localwp_x.size() << endl;
  // cout << "x values:" << endl;
  // for (int i=0; i<localwp_x.size(); ++i) {
  //   cout << localwp_x[i] << endl;
  // }
  spline_pos.set_points(localwp_x, localwp_y);

  // Re-include previous waypoints if any.
  for (int i = 0; i < prev_size; ++i) {
    t.x.push_back(previous_path_x[i]);
    t.y.push_back(previous_path_y[i]);
    // TODO: Also include s and d of trajectory.
  }

  // Create target position in front of the car
  double target_x = this->config.target_x;
  double target_y = spline_pos(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
  double x_add_on = 0;

  // The path between the car and the target contains several points.
  // In the code below we place these points onto this path.
  for (int i = 0; i < num_wp - prev_size; ++i) {
    double v;
    if (dv < 0) {
      // cout << "decelerate by " << (dt * max_a) << endl;
      v = cur_v - (dt * i * max_a);
    }
    else {
      // cout << "accelerate by " << (dt * max_a) << endl;
      v = min(cur_v + (dt * i * max_a), ref.v);
    }
    double point_dist = (target_dist/(dt * v));
    double point_x = x_add_on + (target_x)/point_dist;
    double point_y = spline_pos(point_x);

    x_add_on = point_x;

    // Rotate back to world coordinates.
    double temp_x = point_x;
    double temp_y = point_y;
    point_x = (temp_x * cos(ref.yaw) - temp_y * sin(ref.yaw));
    point_y = (temp_x * sin(ref.yaw) + temp_y * cos(ref.yaw));

    point_x += ref.x;
    point_y += ref.y;

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

void EgoCar::RealizeKeepLane(const vector<OtherCar> &other_cars, Position *ref) {
  // Set max acceleration to the car in front of ego car.
  (*ref).v = this->TargetSpeedForLane(other_cars, (*ref));
}

bool EgoCar::is_behind(const OtherCar &car, const Position &ref) {
  // TODO: Does higher s always means in front of the car?
  return (this->position.s < car.position.s);
}

double EgoCar::TargetSpeedForLane(const vector<OtherCar> &other_cars,
                                  const Position &ref) {

  // Find the closest car in the same lane and ahead of the ego car.
  bool found_car = false;
  double distance = DBL_MAX;
  OtherCar closest_car;
  for (OtherCar const& car : other_cars) {
    if (car.config.target_lane == this->config.target_lane &&
        this->is_behind(car, ref)) {
      // Initial setting, register the first car found.
      if (found_car == false) {
        closest_car = car;
      }
      double new_distance = abs(closest_car.position.s - ref.s);

      if (new_distance < distance && new_distance < this->config.car_length*20) {
        closest_car = car;
        found_car = true;
        // cout << "closest car id: " << closest_car.config.id <<
        //         " distance (prev|new): " << distance << "|" <<
        //         new_distance << " car speed: " <<
        //         closest_car.position.v << endl;
        // cout << "s(this|closest car): " << ref.s << "|" <<
        //         closest_car.position.s << endl;
        distance = new_distance;
      }
    }
  }

  if (found_car == false) {
    return ref.v;  
  }
  else {
    // cout << "Should set target speed to " << closest_car.position.v << endl;
    this->config.target_x = closest_car.position.x;
    this->config.anchor_distance = distance;
    return closest_car.position.v;
  }
  
}

void EgoCar::RealizeLaneChange(const vector<OtherCar> &other_cars,
                               int num_lanes, Position *ref) {
  // If the next line is empty, then it is time to consider line change.

  this->config.target_lane = d2lane(this->position.d) + num_lanes;
}
