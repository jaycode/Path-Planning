/**
 * A header file that contains all data structures used in this project.
 */

#ifndef BASE_H
#define BASE_H

#include <vector>
#include "json.hpp"

namespace ego {

typedef struct Trajectory {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> d;
} Trajectory;

enum State {
  // Constant Speed (useful for other cars)
  STATE_CS = 0,
  // Keep Lane
  STATE_KL,
  // Lane Change Left
  STATE_LCL,
  // Lane Change Right
  STATE_LCR,
  // Used for enum loop, do not remove, and keep it as the last element.
  // See EgoCar::ChooseBestState() function for an example of implementation.
  ENUM_END
};

// To return state names for logging purposes
inline const char* State2Str(State v)
{
    switch (v)
    {
        case STATE_CS:   return "Constant Speed";
        case STATE_KL:   return "Keep Lane";
        case STATE_LCL: return "Lane Change Left";
        case STATE_LCR: return "Lane Change Right";
        default:      return "[Unknown State]";
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
  int num_wp = 30;

  // Spline anchors
  int spline_anchors = 3;
  double anchor_distance = 30.0;

  // Target position x meter ahead of the ego car.
  // The ego car will spline its trajectory into this position.
  double target_x = 30.0;

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
  World world;
  Position position;
  EgoConfig config;
} Snapshot;

/**
 * Weights used in cost function.
 */
typedef struct CostWeights {
  double time_diff;
  double s_diff;
  double efficiency;
  double max_jerk;
  double total_jerk;
  double collision;
  double buffer;
  double max_accel;
  double total_accel;
} CostWeights;

}

#endif