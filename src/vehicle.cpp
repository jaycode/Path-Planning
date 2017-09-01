#include <iostream>
#include "vehicle.h"
#include "json.hpp"
#include "helpers.h"
#include "spline.h"
#include <float.h>
#include <math.h>
#include <tuple>
#include "cost.h"

using json = nlohmann::json;

using namespace ego;
using namespace ego_help;

/**
 * Initialize Vehicle
 */
EgoCar::EgoCar(World world, Position position, EgoConfig config) {
  this->state = STATE_KL;
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
  snap.state = this->state;
  snap.world = this->world;
  snap.position = this->position;
  snap.config = this->config;
  return snap;
}

void EgoCar::InitFromSnapshot(const Snapshot &snap) {
  this->state = snap.state;
  this->world = snap.world;
  this->position = snap.position;
  this->config = snap.config;
}

Trajectory EgoCar::PlanTrajectory(const vector<OtherCar> &other_cars,
                                  const ego::CostWeights &weights) {
  Trajectory waypoints;
  State best_state = this->ChooseBestState(other_cars, weights, &waypoints);
  this->state = best_state;
  return waypoints;
}


int debug_iter = 0;
State EgoCar::ChooseBestState(const vector<OtherCar> &other_cars,
                              const ego::CostWeights &weights,
                              Trajectory *waypoints
                              ) {
  double best_cost = DBL_MAX;
  State best_state;

  // Go through the states one by one and calculate the cost for walking down
  // each state.
  Snapshot initial_snap = this->getSnapshot();
  Snapshot best_snap;

  Trajectory current_best_waypoints;
  vector<char> minimap = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

  int ii = 0;
  for ( int state = 0; state != ENUM_END; state++ ) {
    cout << "\n" << ii << ". Now calculating cost of state " << state << " / " << State2Str((State)state) << endl;
    ii++;
    Snapshot cur_snap = this->getSnapshot();
    // For each state, the ego vehicle "imagines" following a trajectory
    Trajectory waypoints;
    Trajectory plan;

    this->CreateTrajectories((State)state, other_cars, &cur_snap,
                             &waypoints, &plan);

    // cout << "Now calculating cost of state " << State2Str((State)state)
    //   << "\nmean s: " << mean(plan.s)
    //   << " d: " << mean(plan.d)
    //   << " stdev s: " << stdev(plan.s)
    //   << " d: " << stdev(plan.d)
    //   << "\n"
    //   << "last s: " << (plan.s[plan.s.size()-1])
    //   << " d: " << (plan.d[plan.d.size()-1])
    //   << endl;


    // TODO: Looks like reinforcement learning can be implemented here
    //       by treating (state + other_cars) as a state
    tuple<State, Snapshot, vector<OtherCar>> cf_state = 
      make_tuple(State(state), cur_snap, other_cars);

    vector<char> cur_minimap = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
    double cost = ego_cost::CalculateCost(cf_state, plan, 
      waypoints, weights, &cur_minimap);

    if (cost < best_cost) {
      best_state = (State)state;
      current_best_waypoints = waypoints;
      best_cost = cost;
      best_snap = cur_snap;
      minimap = cur_minimap;
    }
    this->InitFromSnapshot(initial_snap);
  }
  (*waypoints) = current_best_waypoints;
  this->InitFromSnapshot(best_snap);

  cout << endl << "Chosen State: " << State2Str((State)best_state) << endl;

  // When a state is chosen, set other states' durations to 0.
  map<State, double> state_durations = *best_snap.config.state_durations_ptr;
  for ( int state = 0; state != ENUM_END; state++ ) {
    if ((State)state != best_state) {
      state_durations[(State)state] = 0.0;
      // cout << "Reset duration of state " << State2Str((State)state) << " to 0." << endl;
    }
  }

  cout << "state collision map:" << endl <<
  "-------------" << endl <<
  "| "<< minimap[0] <<" | "<< minimap[1] <<" | "<< minimap[2] <<" |" << endl <<
  "-------------" << endl <<
  "| "<< minimap[3] <<" | "<< minimap[4] <<" | "<< minimap[5] <<" |" << endl <<
  "-------------" << endl <<
  "| "<< minimap[6] <<" | "<< minimap[7] <<" | "<< minimap[8] <<" |" << endl <<
  "-------------" << endl;

  // cout << "\n==========================================\n\n";

  // cout << "chosen target speed: " << best_snap.config.target_speed << endl;
  // cout << "Target lane: " << best_snap.config.target_lane << " or in d: " << lane2d(best_snap.config.target_lane) << endl;
  // assert((*waypoints).x.size() > 0);
  // cout << "The car will then move from xy of " << best_snap.position.x <<
  //   ", " << best_snap.position.y << " to " <<
  //   (*waypoints).x[0] << ", " <<
  //   (*waypoints).y[0] << endl;
  // cout << "-----" << endl << endl;

  return (State)best_state;
}

void EgoCar::CreateTrajectories(State state,
                                const vector<OtherCar> &other_cars,
                                Snapshot *snap,
                                Trajectory *waypoints,
                                Trajectory *plan) {
  (*waypoints).x = {};
  (*waypoints).y = {};
  (*waypoints).s = {};
  (*waypoints).d = {};
  (*waypoints).distance = 0.0;

  (*plan).x = {};
  (*plan).y = {};
  (*plan).s = {};
  (*plan).d = {};
  (*plan).distance = 0.0;

  vector<double> &map_waypoints_s = *this->world.map_waypoints_s;
  vector<double> &map_waypoints_x = *this->world.map_waypoints_x;
  vector<double> &map_waypoints_y = *this->world.map_waypoints_y;

  json &previous_path_x = *(*snap).config.previous_path_x;
  json &previous_path_y = *(*snap).config.previous_path_y;

  int prev_size = previous_path_x.size();
  Position ref = (*snap).position;

  double &car_length = (*snap).config.car_length;

  // Local-coordinates of waypoints (i.e. car position is [0,0])
  vector<double> localwp_x;
  vector<double> localwp_y;

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
  // Switch state step
  switch(state) {
    case STATE_KL: {
      this->RealizeKeepLane(other_cars, ref, snap);
      break;
    }
    case STATE_LCL: {
      if (ref.d > 2.1) {
        this->RealizeLaneChange(other_cars, -1, ref, snap);
      }
      break;
    }
    case STATE_FC: {
      this->RealizeFollowCar(other_cars, ref, snap);
      break;
    }
    case STATE_LCR: {
      if (ref.d < 9.9) {
        this->RealizeLaneChange(other_cars, 1, ref, snap);
      }
      break;
    }
    default: {

    }
  }

  ref.v = (*snap).config.target_speed;

  // ===PARAMETERS===
  // All these parameters will have been changed by the switch state step above.
  // They are used in changing the trajectory.
 
  // Current velocity in meter/second.
  double &cur_v = (*snap).position.v;

  // Current acceleration in meter/second^2.
  double &cur_a = (*snap).position.a;

  // double ref_yaw = 0.0;

  // Current lane: 0 - left, 1 - center, 2 - right
  int &lane = (*snap).config.target_lane;

  // Number of waypoints.
  int &num_wp = (*snap).config.num_wp;

  // Number of plan points.
  int &num_pp = (*snap).config.num_pp;

  // The points to draw will be passed on as waypoints
  int num_points_to_draw = num_wp - prev_size;

  // All the points to imagine (which also consist of waypoints)
  // will be used to choose a best state in the behavior selection step.
  int num_points_to_imagine = num_points_to_draw + num_pp;

  double &dt = (*snap).config.dt;

  double max_a = (*snap).config.max_acceleration;

  double max_j = (*snap).config.max_jerk;

  // Duration in seconds in a state. Useful for determining speed.
  map<ego::State, double> *state_durations_ptr = (*snap).config.state_durations_ptr;
  double sdt = (*state_durations_ptr)[state];

  // cout << "sdt of " << State2Str(state) << ":" << endl;
  // cout << (*state_durations_ptr)[state] << endl;

  // ===END===
  
  if ((*snap).config.override_spline_anchors == false) {
    // Spline anchors are placed with some distance between each pair.
    int &spline_anchors = (*snap).config.spline_anchors;
    double anchor_distance = ((*snap).config.horizon / spline_anchors);

    for (int i = 1; i <= spline_anchors; ++i) {
      vector<double> next_xy = getXY(ref.s+i*anchor_distance, lane2d(lane),
                                     map_waypoints_s, map_waypoints_x,
                                     map_waypoints_y);
      localwp_x.push_back(next_xy[0]);
      localwp_y.push_back(next_xy[1]);
    }
  }
  else {
    // Distance so far
    double sum_s = 0.0;
    // cout << "spline anchors: " << endl;
    for (int i = 0; i < (*snap).config.active_spline_anchors.size(); ++i) {
      sum_s += get<0>((*snap).config.active_spline_anchors[i]);
      vector<double> next_xy = getXY(ref.s + sum_s,
                                     get<1>((*snap).config.active_spline_anchors[i]),
                                     map_waypoints_s, map_waypoints_x,
                                     map_waypoints_y);
      localwp_x.push_back(next_xy[0]);
      localwp_y.push_back(next_xy[1]);
      // cout << "- sd: " << sum_s << ", " << get<1>((*snap).config.active_spline_anchors[i])
      //   << endl;
      // cout << "current s: " << (*snap).position.s << endl;

    }
  }

  // cout << "Anchor points of spline_pos:" << endl;
  for (int i = 1; i < localwp_x.size(); ++i) {
    ref.yaw = atan2(localwp_y[i] - localwp_y[i-1], localwp_x[i] - localwp_x[i-1]);

    vector<double> sd = getFrenet(localwp_x[i], localwp_y[i], ref.yaw, map_waypoints_x,
                                   map_waypoints_y);
    // cout << "xy: " << localwp_x[i] << ", " << localwp_y[i] << endl;
    // cout << "sd: " << sd[0] << ", " << sd[1] << endl;
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

  // Spline for position
  tk::spline spline_pos;


  PruneWaypoint(&localwp_x, &localwp_y);

  spline_pos.set_points(localwp_x, localwp_y);

  // cout << "When x is " << ref.x + 60 << ", y is " << spline_pos(ref.x+60) << endl;
  // double _x = ref.x + 60;
  // double _y = spline_pos(_x);

  // double temp_x = _x;
  // double temp_y = _y;
  // _x = (temp_x * cos(ref.yaw) - temp_y * sin(ref.yaw));
  // _y = (temp_x * sin(ref.yaw) + temp_y * cos(ref.yaw));

  // _x += ref.x;
  // _y += ref.y;


  // vector<double> sd = getFrenet(_x, _y, ref.yaw, map_waypoints_x,
  //                                map_waypoints_y);
  // cout << "Converted to sd position: " << sd[0] << ", " << sd[1] << endl;


  // Re-include previous waypoints if any.
  for (int i = 0; i < prev_size; ++i) {
    (*waypoints).x.push_back(previous_path_x[i]);
    (*waypoints).y.push_back(previous_path_y[i]);
    (*plan).x.push_back(previous_path_x[i]);
    (*plan).y.push_back(previous_path_y[i]);
    vector<double> point_sd = getFrenet(previous_path_x[i],
                                        previous_path_y[i], ref.yaw,
                                        map_waypoints_x, map_waypoints_y);
    (*waypoints).s.push_back(point_sd[0]);
    (*waypoints).d.push_back(point_sd[1]);
    (*plan).s.push_back(point_sd[0]);
    (*plan).d.push_back(point_sd[1]);
    if (i > 0) {
      (*waypoints).distance += distance((*waypoints).x[i-1], (*waypoints).y[i-1],
                                   (*waypoints).x[i], (*waypoints).y[i]);

      (*plan).distance += distance((*plan).x[i-1], (*plan).y[i-1],
                                   (*plan).x[i], (*plan).y[i]);
    }
  }

  // Spline for velocity and time.
  tk::spline spline_v;
  if ((*snap).config.use_spline_v == true) {
    double sum_t = 0.0;
    vector<double> v_axis;
    vector<double> t_axis;
    for (int i = 0; i < (*snap).config.spline_v_anchors.size(); ++i) {
      sum_t += get<0>((*snap).config.spline_v_anchors[i]);
      double v = get<1>((*snap).config.spline_v_anchors[i]);
      t_axis.push_back(sum_t);
      v_axis.push_back(v);
    }
    spline_v.set_points(t_axis, v_axis);
  }

  // Spline for acceleration and time.
  tk::spline spline_a;
  if ((*snap).config.use_spline_a == true) {
    double sum_t = 0.0;
    vector<double> a_axis;
    vector<double> t_axis;
    for (int i = 0; i < (*snap).config.spline_a_anchors.size(); ++i) {
      sum_t += get<0>((*snap).config.spline_a_anchors[i]);
      double a = get<1>((*snap).config.spline_a_anchors[i]);
      t_axis.push_back(sum_t);
      a_axis.push_back(a);
    }
    spline_a.set_points(t_axis, a_axis);
  }


  // Create target position in front of the car
  double target_x = (*snap).config.target_x;
  double target_y = spline_pos(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

  /**
   * The path between the car and the horizon contains several points:
   * - waypoints / permament trajectory
   * - plan / temporary trajectory
   * `target_x` should be in temporary trajectory before `horizon`.
   * 
   * In the code below we place points onto the path between end of current
   * waypoints all the way to the horizon.
   *
   * One thing to note here is that the car has a maximum acceleration
   * it can use, so there is no guarantee that the car
   * will reach the target distance nor the horizon specified above. 
   */ 

  double x_so_far = 0.0;
  double v_so_far = 0.0;
  double prev_point_x = 0;
  double prev_point_y = 0;
  double yaw = 0.0;
  // cout << "sdt starts at " << sdt << endl;
  // Number of points to draw as waypoints.
  for (int i = 0; i < num_points_to_imagine ; ++i) {
    sdt += dt;
    double a;
    double v;


    if ((*snap).config.use_spline_a == true){
      a = spline_a(sdt);      
      v = v_so_far + (dt * a) * cos(yaw);
      if (state == STATE_KL) {
        // cout << "v_so_far: " << v_so_far << endl;
        // cout << "(dt * a) = " << (dt * a) << endl;
        // cout << "(dt * a) * cos(yaw) = " << (dt * a) * cos(yaw) << endl;
        // cout << "v_so_far + (dt * a) * cos(yaw) = " << (v_so_far + ((dt * a) * cos(yaw))) << endl;
        // cout << "accel at time " << sdt << "s (point " <<
        //   (int)(sdt / dt) << "): " << a << endl;
      }
    }
    else if ((*snap).config.use_spline_v == true){
      v = spline_v(sdt);
      // cout << "speed at time " << sdt << "s (point " << 
      //   (int)(sdt / dt) << "): " << v << endl;
    }
    else {
      if (dv < 0) {
        a = cur_a - (dt * i * max_j);
      } {
        a = min(cur_a + (dt * i * max_j), max_a);
      }

      if (dv < 0) {
        // cout << "decelerate by " << (dt * max_a) << endl;
        v = cur_v - (dt * i * a);
      }
      else {
        // cout << "accelerate by " << (dt * max_a) << endl;
        v = min(cur_v + (dt * i * max_a), ref.v);
      }
    }


    v_so_far = v;
    v = min(cur_v + v_so_far, ref.v);
    // cout << "current speed (mps/mph): " << cur_v << "/" << mps2mph(cur_v) << " new speed (mps/mph): " <<
    //   v << "/" << mps2mph(v) << " yaw: " << yaw << "(" << rad2deg(yaw) << "deg, cos: " << 
    //   cos(yaw) << ") limit (mps/mph): " << 
    //   (ref.v * cos(yaw)) << "/" << mps2mph(ref.v * cos(yaw)) << endl;
    double point_dist = (dt * v);
    double point_x = min((x_so_far + point_dist), target_x);
    // cout << "point_x: " << point_x << endl;

    double point_y = spline_pos(point_x);

    x_so_far = point_x;

    // Rotate back to world coordinates.
    double temp_x = point_x;
    double temp_y = point_y;
    point_x = (temp_x * cos(ref.yaw) - temp_y * sin(ref.yaw));
    point_y = (temp_x * sin(ref.yaw) + temp_y * cos(ref.yaw));

    point_x += ref.x;
    point_y += ref.y;

    // Update yaw
    if (prev_point_y > 0.0 && prev_point_x > 0.0) {
      // cout << "calculate yaw from prev y " << prev_point_y <<
      //   " y " << point_y << " prev x " << prev_point_x << " x " <<
      //   point_x << endl;
      yaw = atan2(point_y - prev_point_y, point_x - prev_point_x);
    }
    prev_point_x = point_x;
    prev_point_y = point_y;


    // Insert into waypoints and plan
    // cout << "point_x is " << point_x << endl; 
    vector<double> point_sd = getFrenet(point_x, point_y, yaw,
                                        map_waypoints_x, map_waypoints_y);
    (*plan).x.push_back(point_x);
    (*plan).y.push_back(point_y);
    (*plan).s.push_back(point_sd[0]);
    (*plan).d.push_back(point_sd[1]);
    (*plan).distance += point_dist;

    if (i < num_points_to_draw) {
      (*waypoints).x.push_back(point_x);
      (*waypoints).y.push_back(point_y);
      (*waypoints).s.push_back(point_sd[0]);
      (*waypoints).d.push_back(point_sd[1]);
      (*waypoints).distance += point_dist;
    }
  }

  // And finally, set snapshot position to the end point of waypoints
  // to make life easier in cost calculation.
  (*snap).position.x = (*waypoints).x[(*waypoints).x.size()-1];
  (*snap).position.y = (*waypoints).y[(*waypoints).y.size()-1];
  (*snap).position.s = (*waypoints).s[(*waypoints).s.size()-1];
  (*snap).position.d = (*waypoints).d[(*waypoints).d.size()-1];

  if ((*waypoints).x.size() >= 2) {
    (*snap).position.v = distance(
                         (*waypoints).x[(*waypoints).x.size()-1],
                         (*waypoints).y[(*waypoints).y.size()-1],
                         (*waypoints).x[(*waypoints).x.size()-2],
                         (*waypoints).y[(*waypoints).y.size()-2]) / dt;
  }

  if ((*waypoints).x.size() >= 3) {
    double v1 = (*snap).position.v;
    double v2 = distance(
                         (*waypoints).x[(*waypoints).x.size()-2],
                         (*waypoints).y[(*waypoints).y.size()-2],
                         (*waypoints).x[(*waypoints).x.size()-3],
                         (*waypoints).y[(*waypoints).y.size()-3]) / dt;
    (*snap).position.a = (v2 - v1) / dt;
  }

  // Update state durations

  (*(*snap).config.state_durations_ptr)[state] = sdt;
  // cout << "Updated state duration of " << State2Str(state) << " to " << sdt;

  // if (state == STATE_LCR) {
  //   cout << "stored tj lane: " << d2lane(t.d[t.d.size()-1]) << endl;
  //   debug_iter++;
  //   if (debug_iter==50) {
  //     exit(0);
  //   }
  // }

  // cout << "Current car's lane: " << d2lane(this->position.d) << endl;
  // cout << "target lane: " << (*snap).config.target_lane << endl;
  // cout << "target_x: " << target_x << endl;
  // cout << "target dist: " << target_dist << endl;
  // cout << "max a: " << max_a << " target_v: " << ref.v << endl;
  // cout << "ref.x: " << ref.x << " ref.y: " << ref.y << " ref.yaw " << ref.yaw << endl;
  // cout << "ref.s: " << ref.s << " ref.d: " << ref.d << endl;
  // cout << "car.x: " << this->position.x << " car.y: " << this->position.y << endl;
  // cout << "car.s: " << this->position.s << " car.d: " << this->position.d << endl;

  // The code below shows the difference between trailing distance and calculated
  // from s. The difference was almost 10 meters!
  // double s1 = t.s[t.s.size()-1];
  // double s2 = t.s[0];
  // double dist = (s1 - s2);
  // cout << "After creation, trajectory dist (from s|trailing): " <<
  //   dist << "|" << t.distance << endl;
  // Output for 1st run:
  // After creation, trajectory dist (from s|trailing): 35.1848|16.929

  // cout << "Number of trajectory points: " << t.s.size() << endl;

  // cout << endl << "trajectory [x, y]: " << endl;
  // for (int i; i < t.x.size(); ++i) {
  //   cout << t.x[i] << ", " << t.y[i] << endl;
  // }
  // cout << endl;
}


void EgoCar::RealizeKeepLane(const vector<OtherCar> &other_cars,
                             const Position &ref, Snapshot *snap) {
  (*snap).config.override_spline_anchors = false;
  (*snap).config.use_spline_v = false;
  (*snap).config.use_spline_a = true;
  (*snap).config.override_spline_anchors = true;
  (*snap).config.max_acceleration = (*snap).config.default_max_acceleration;
  (*snap).config.target_speed = (*snap).config.default_target_speed;
  (*snap).config.target_lane = d2lane((*snap).position.d );
  (*snap).config.num_wp = 40;
  (*snap).config.num_pp = 40;

  // Lane

  // Trajectory to use:
  // https://www.desmos.com/calculator/kdlhxlwutv
  vector<tuple<double, double>> pos_anchors;
  pos_anchors.push_back(make_tuple(7, (*snap).position.d ));
  pos_anchors.push_back(make_tuple(12, (*snap).position.d ));
  pos_anchors.push_back(make_tuple(8, (*snap).position.d ));
  (*snap).config.active_spline_anchors = pos_anchors;

  // Velocity

  // vector<tuple<double, double>> spline_v_anchors;
  // double target_v = (*snap).config.target_speed;
  // double dt = (*snap).config.dt;

  // spline_v_anchors.push_back(make_tuple(-dt, 0.01 * target_v));
  // spline_v_anchors.push_back(make_tuple(20/dt, 0.01 * target_v));
  // spline_v_anchors.push_back(make_tuple(40/dt, 0.015 * target_v));
  // spline_v_anchors.push_back(make_tuple(15/dt, 0.08 * target_v));
  // // spline_v_anchors.push_back(make_tuple(25/dt, 0.12 * target_v));
  // // spline_v_anchors.push_back(make_tuple(pts/dt, 0.3 * target_v));
  // spline_v_anchors.push_back(make_tuple(100/dt, 1.0 * target_v));

  // (*snap).config.spline_v_anchors = spline_v_anchors;

  // Acceleration
  // Division with dt gives us seconds, basically. I used it so it is
  // easier to imagine. I found the coefficients below from trial and error.

  vector<tuple<double, double>> spline_a_anchors;
  double target_a = (*snap).config.max_acceleration;
  double dt = (*snap).config.dt;

  spline_a_anchors.push_back(make_tuple(-dt, 0.1 * target_a));
  spline_a_anchors.push_back(make_tuple(0.5/dt, (0.5) * target_a));
  spline_a_anchors.push_back(make_tuple(1.0/dt, 1.0 * target_a));

  (*snap).config.spline_a_anchors = spline_a_anchors;

}

// void EgoCar::RealizeFollowCar(const vector<OtherCar> &other_cars,
//                               const Position &ref, Snapshot *snap) {
//   (*snap).config.override_spline_anchors = false;
//   // Similar to RealizeKeepLane but find a car to follow.
//   double minimum_distance = (*snap).config.follow_distance;
//   vector<int> lanes = {d2lane(ref.d)};
//   (*snap).config.max_acceleration = config.default_max_acceleration;
//   // Set target speed to the car in front of ego car.
//   (*snap).config.target_lane = d2lane(ref.d);
//   vector<OtherCar> closest_car_v = this->FindClosestCar(
//     other_cars, ref, snap, lanes, minimum_distance);
//   if (closest_car_v.size() > 0) {
//     OtherCar closest_car = closest_car_v[0];
//     // cout << "Should set target speed to " << closest_car.position.v << endl;
//     (*snap).config.target_x = abs(closest_car.position.s - (*snap).position.s);
//     (*snap).config.target_speed = closest_car.position.v;
//     // cout << "Set target speed to " << (*snap).config.target_speed << endl;
//   }
// }

void EgoCar::RealizeFollowCar(const vector<OtherCar> &other_cars,
                              const Position &ref, Snapshot *snap) {
  // Similar to RealizeKeepLane but find a car to follow.

  (*snap).config.override_spline_anchors = false;
  (*snap).config.use_spline_a = false;
  (*snap).config.use_spline_v = false;
  (*snap).config.override_spline_anchors = false;
  (*snap).config.num_pp = 60;
  double minimum_distance = (*snap).config.follow_distance;
  (*snap).config.max_acceleration = config.default_max_acceleration;

  vector<int> lanes = {d2lane(ref.d)};
  // vector<int> lanes = {0, 1, 2};
  // Set target speed to the car in front of ego car.
  (*snap).config.target_lane = d2lane(ref.d);
  vector<OtherCar> closest_car_v = this->FindClosestCar(
    other_cars, ref, snap, lanes, minimum_distance);
  if (closest_car_v.size() > 0) {
    (*snap).config.use_spline_a = true;
    OtherCar closest_car = closest_car_v[0];
    // cout << "Should set target speed to " << closest_car.position.v << endl;
    (*snap).config.target_x = abs(closest_car.position.s - (*snap).position.s);
    (*snap).config.target_speed = closest_car.position.v;
    // cout << "Set target speed to " << (*snap).config.target_speed << endl;

    // vector<tuple<double, double>> spline_v_anchors;
    // double target_v = (*snap).config.target_speed;
    // double dt = (*snap).config.dt;
    // int num_pp = (*snap).config.num_pp;

    // spline_v_anchors.push_back(make_tuple(-dt, 0.01 * target_v));
    // spline_v_anchors.push_back(make_tuple(10*dt, 0.1 * target_v));
    // spline_v_anchors.push_back(make_tuple(15*dt, 0.15 * target_v));
    // spline_v_anchors.push_back(make_tuple(25*dt, 0.25 * target_v));
    // spline_v_anchors.push_back(make_tuple(num_pp*dt, 0.35 * target_v));
    // spline_v_anchors.push_back(make_tuple(num_pp*3*dt, target_v));

    // (*snap).config.spline_v_anchors = spline_v_anchors;

    vector<tuple<double, double>> spline_a_anchors;
    double target_a = (*snap).config.max_acceleration;
    double dt = (*snap).config.dt;

    spline_a_anchors.push_back(make_tuple(-dt, 0.02 * target_a));
    spline_a_anchors.push_back(make_tuple(0.7/dt, (0.5) * target_a));
    spline_a_anchors.push_back(make_tuple(2.4/dt, 1.0 * target_a));

    (*snap).config.spline_a_anchors = spline_a_anchors;

  }


}

bool EgoCar::is_behind(const OtherCar &car) {
  // TODO: Does higher s always mean in front of the car?
  return (this->position.s < car.position.s);
}

std::vector<OtherCar> EgoCar::FindClosestCar(
  const vector<OtherCar> &other_cars,
  const Position &ref, Snapshot *snap,
  const vector<int> &lanes,
  double minimum_distance = 5.0) {

  // Find the closest car in the same lane and ahead of the ego car.
  bool found_car = false;
  double distance = DBL_MAX;

  OtherCar closest_car;
  for (OtherCar const& car : other_cars) {
    bool in_lanes = false;
    if ( std::find(
      lanes.begin(), lanes.end(), d2lane(car.position.d)) != lanes.end() ) {
      in_lanes = true;
    }
    if (in_lanes && this->is_behind(car)) {
      // Initial setting, register the first car found.
      if (found_car == false) {
        closest_car = car;
      }
      double new_distance = abs(closest_car.position.s - ref.s);

      if (new_distance < distance && new_distance < minimum_distance) {
        closest_car = car;
        found_car = true;
        cout << "closest car id: " << closest_car.config.id <<
                " distance (prev|new): " << distance << "|" <<
                new_distance << " car speed: " <<
                closest_car.position.v << endl;
        cout << "s(this|closest car): " << ref.s << "|" <<
                closest_car.position.s << endl;
        distance = new_distance;
      }
    }
  }
  vector<OtherCar> v;
  if (found_car == true) {
     v.push_back(closest_car);
  }
  return v;
  
}

void EgoCar::RealizeLaneChange(const vector<OtherCar> &other_cars,
                               int num_lanes, const Position &ref, Snapshot *snap) {
  // If the next line is empty, then it is time to consider line change.
  // TODO: Determine sharpness.

  // cout << "yaw at turns: " << ref.yaw << endl;

  (*snap).config.use_spline_a = true;
  (*snap).config.override_spline_anchors = true;
  (*snap).config.use_spline_v = false;
  (*snap).config.target_x = 80.0;
  // (*snap).config.horizon = 70.0;
  (*snap).config.num_wp = 40;
  (*snap).config.num_pp = 50;
  (*snap).config.target_lane = d2lane((*snap).position.d) + num_lanes;

  // Turning would naturally result in smaller acceleration.
  // TODO: Find this decrease value with the right physics.
  (*snap).config.max_acceleration = 0.8 * (*snap).config.default_max_acceleration;
  (*snap).config.target_speed = 1.0 * (*snap).config.default_target_speed;

  double lane_width = 4;
  double v = (*snap).config.target_speed;
  double dt = (*snap).config.dt;
  double a = (*snap).config.max_acceleration;
  double i = 2/dt;
  double s0 = 0;
  double s = s0 + (v * dt * i) + 0.5 * (a * dt * dt * i);
  double target_a = (*snap).config.max_acceleration;
  // cout << "s is " << s << endl;

  // Trajectory to use:
  // https://www.desmos.com/calculator/kdlhxlwutv
  // X-axis not to scale.
  vector<tuple<double, double>> pos_anchors;

  pos_anchors.push_back(make_tuple(s, lane2d((*snap).config.target_lane)));
  pos_anchors.push_back(make_tuple(10, lane2d((*snap).config.target_lane)));
  pos_anchors.push_back(make_tuple(10, lane2d((*snap).config.target_lane)));

  (*snap).config.active_spline_anchors = pos_anchors;

  vector<tuple<double, double>> spline_a_anchors;

  // Acceleration
  // spline_a_anchors.push_back(make_tuple(2/dt, -1.0));
  // spline_a_anchors.push_back(make_tuple(i, target_a));
  // spline_a_anchors.push_back(make_tuple(i, (*snap).config.default_max_acceleration));

  spline_a_anchors.push_back(make_tuple(-dt, 0.1 * target_a));
  spline_a_anchors.push_back(make_tuple(2.5/dt, (0.2) * target_a));
  spline_a_anchors.push_back(make_tuple(2.5/dt, 0.4 * target_a));
  spline_a_anchors.push_back(make_tuple(2.0/dt, 0.7 * target_a));

  (*snap).config.spline_a_anchors = spline_a_anchors;

}
