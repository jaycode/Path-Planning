/**
 * A header file that contains all data structures used in this project.
 */

#ifndef BASE_H
#define BASE_H

#include <vector>
#include "json.hpp"
#include "spline.h"

extern tk::spline sx;
extern tk::spline sy;
extern tk::spline sh;

namespace ego {

typedef struct Trajectory {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> d;
  // We use trailing distance since getFrenet method is not accurate
  // enough to calculate the distance over a long time.
  // Trailing distance is calculated in function EgoCar::CreateTrajectory.
  double distance;
} Trajectory;

enum State {
  // Keep Lane
  STATE_KL = 0,
  // Lane Change Left
  STATE_LCL,
  // Lane Change Right
  STATE_LCR,
  // Follow a Car
  STATE_FC,
  // Used for enum loop, do not remove, and keep it as the last element.
  // See EgoCar::ChooseBestState() function for an example of implementation.
  ENUM_END
};

// To return state names for logging purposes
inline const char* State2Str(State v)
{
    switch (v)
    {
        case STATE_KL:   return "Keep Lane";
        case STATE_LCL:  return "Lane Change Left";
        case STATE_LCR:  return "Lane Change Right";
        case STATE_FC:   return "Follow Car";
        default:         return "[Unknown State]";
    }
}

typedef struct Config {

  // Time each iteration
  double dt;

  // Distance between front and rear wheels in meter.
  double car_length;

  // Which lane the car is going?
  int target_lane = 1;

} Config;

typedef struct EgoConfig : Config {
  double default_target_speed = 0.0;
  double default_max_acceleration = 0.0;

  // mph
  double target_speed = 0.0;

  // meter / second^3
  double max_acceleration = 0.0;

  // meter / second^3
  double max_jerk = 0.0;

  nlohmann::json *previous_path_x;
  nlohmann::json *previous_path_y;
  double *end_path_s;
  double *end_path_d;

  // Number of waypoints.
  int num_wp = 60;

  // Number of plan points ahead of the end of
  // waypoints.
  int num_pp = 80;

  // Maximum number of points from last path to use.
  // int num_last_path = 15;

  // Spline anchors
  int spline_anchors;
  double horizon;

  // If true, use the spline anchors defined below.
  bool override_spline_anchors = false;
  std::vector<std::tuple<double, double>> active_spline_anchors;
  
  bool use_spline_v = false;
  std::vector<std::tuple<double, double>> spline_v_anchors;

  bool use_spline_a = false;
  std::vector<std::tuple<double, double>> spline_a_anchors;

  // If the difference of anchor distance of xy and sd positions
  // are higher than this value, then something is wrong in
  // Frenet to XY conversion (`getXY()` function). The car must be
  // less sure on what to do now.
  double anchor_ddist_threshold;

  // Target position x meter ahead of the ego car.
  // The ego car will spline its trajectory into this position.
  // When the car is unsure, set this to a car in front of ego car.
  double target_x = 30.0;

  // When following a car, what's the distance to keep?
  double follow_distance = 8;

  std::map<ego::State, double> state_durations = {
    {ego::STATE_KL, 0.0},
    {ego::STATE_LCL, 0.0},
    {ego::STATE_LCR, 0.0},
    {ego::STATE_FC, 0.0}
  };

} EgoConfig;

typedef struct OtherConfig : Config {
  int id;
} OtherConfig;

/**
 * Can be considered as the "physical" state of the vehicle.
 */
typedef struct Position {
  double x;
  double y;
  double s;
  double d;
  double v;
  double a;
  // radian
  double yaw = 0.0;
  // The other two here are not super useful, but they are
  // included in the sensor fusion, so I store them here as well.
  double vx = 0.0;
  double vy = 0.0;
} Position;

/**
 * World-related data
 */
typedef struct World {
  std::vector<double> *map_waypoints_x;
  std::vector<double> *map_waypoints_y;
  std::vector<double> *map_waypoints_s;
  std::vector<double> *map_waypoints_dx;
  std::vector<double> *map_waypoints_dy;
} World;

/**
 * A snapshot of the current EgoCar object.
 * 
 */
typedef struct Snapshot {
  State state;
  World world;
  Position position;
  EgoConfig config;
} Snapshot;

typedef struct CostWeights {
  double time_diff;
  double s_diff;
  double efficiency;
  double max_accel;
  double max_jerk;
  double collision;
  double change_state;
} CostWeights;

}

#endif