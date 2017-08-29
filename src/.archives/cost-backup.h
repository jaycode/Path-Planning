#ifndef COST_H
#define COST_H

#include <fstream>
#include <tuple>
#include "base.h"
#include <string>
#include "vehicle.h"
#include <math.h>
#include "helpers.h"
#include <cfloat>

namespace {

namespace ego_cost {

using namespace std;
using namespace std;
using namespace ego_help;


/**
 * Look at the s distance traveled and penalize smaller distance.
 */
double EfficiencyCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                      const ego::Trajectory &trajectory, double weight) {

  double s1 = trajectory.s[trajectory.s.size()-1];
  double s2 = trajectory.s[trajectory.s.size()-2];
  double dt = get<1>(cf_state).config.dt;
  double v = (s1-s2)/dt;
  ego::Snapshot snap = get<1>(cf_state);
  double max_v = snap.config.target_speed;
  double max_a = snap.config.max_acceleration;
  double max_j = snap.config.max_jerk;
  double max_s = dt*max_v+(dt*dt)*max_a+(dt*dt*dt)*max_j;
  return (abs(max_s-trajectory.distance) * weight + abs(snap.config.target_speed-v) * weight);
}

/**
 * For each point in trajectory, check if there is any car that will cross path
 * at the same dt.
 */
  
  
double CollisionCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                     const ego::Trajectory &trajectory, double weight) {

  double cost = 0.0;
  double s_threshold = 2.0;
  double d_threshold = 6.0;
  double dist_threshold = 10.0;
  ego::Snapshot snap = get<1>(cf_state);
  vector<ego::OtherCar> cars = get<2>(cf_state);
  ego::State state = get<0>(cf_state);
  double dt = snap.config.dt;

  // Check trajectory points from the last point down to this id:
  int n_points = 30;
  int i = trajectory.s.size()-1;
  double s_fin = trajectory.s[i];
  double d_fin = trajectory.d[i];
  // ---USING SD---
  // Assume that the other car is going straight all the time.
  cout << "Car positions(s, d) and distance(xy):" << endl;
  double min_car_dist = DBL_MAX;
  for (int j = 0; j < cars.size(); ++j) {
    ego::OtherCar car = cars[j];
    cout << "Car " << car.config.id << "(" << car.position.s
      << ", " << car.position.d << "): ";

    double car_dist = DBL_MAX;

    for (int i = (trajectory.s.size()-1); i > max(0, (int)((trajectory.s.size()-1)-n_points)); --i) {
      double cs = car.position.s + (double)(i)*car.config.dt*car.position.v;
      double cd = car.position.d;

      double s = trajectory.s[i];
      double d = trajectory.d[i];
      // cout << "Checks at lane " << d2lane(d) << endl;

      // if (d2lane(cd) == d2lane(trajectory))
      double s_dist = cs - s;
      double d_dist = cd - d;
      double dist = sqrt(s_dist*s_dist + d_dist*d_dist);
      if (dist < car_dist) {
        car_dist = dist;
      }
    }
    if (car_dist < min_car_dist) {
      cout << "Looks like I'm going to crash into car " << car.config.id << endl;
      min_car_dist = car_dist;
    }
  }
  // cout << "collision check: " << s_dist << ", " << d_dist << endl;
  if (min_car_dist <= dist_threshold) {
    cost = (dist_threshold-min_car_dist) * weight;
  }

  // ---END - USING SD---


  // ---SINGLE CAR DEBUGGING---
  // Uncomment to debug what happens when there is a single car in front of ego car.
  // double closest_ahead_dist = DBL_MAX;
  // double closest_behind_dist = -DBL_MAX;
  // vector<ego::OtherCar> closest_ahead;
  // vector<ego::OtherCar> closest_behind;
  // ego::OtherCar closest_car;
  // for (int j = 0; j < cars.size(); ++j) {
  //   ego::OtherCar car = cars[j];

  //   double car_dist = car.position.s - trajectory.s[0];
  //   if (car_dist > 0) {
  //     closest_ahead.push_back(car);
  //     if (d2lane(car.position.d) == d2lane(snap.position.d)) {
  //       closest_car = car;
  //       closest_ahead_dist = car.position.s - snap.position.s;
  //     }
  //   }
  //   else {
  //     closest_behind.push_back(car);
  //   } 
  // }

  // // Keep for debugging later
  // cout << "The closest Car " << closest_car.config.id << "(" << closest_car.position.s
  //   << ", " << closest_car.position.d << ") : " << closest_ahead_dist << endl;

  // cout << "Trajectory point " << i << endl;
  // cout << "time   | Ego car s   |    Car s    |    dist" << endl;
  // double min_dist = DBL_MAX;
  // int min_collision_pos = trajectory.s.size();

  // for (int i = (trajectory.s.size()-1); i > max(0, (int)((trajectory.s.size()-1)-n_points)); --i) {
  //   double cs = closest_car.position.s + (double)(i)*closest_car.config.dt*closest_car.position.v;
  //   double cd = closest_car.position.d;

  //   double s = trajectory.s[i];
  //   double d = trajectory.d[i];
  //   cout << "Checks at lane " << d2lane(d) << endl;

  //   double s_dist = cs - s;
  //   double d_dist = cd - d;
  //   double dist = sqrt(s_dist*s_dist + d_dist*d_dist);
  //   cout << ((double)(i)*closest_car.config.dt) << "  |  " << s << "  |  " << cs << "  |  " << dist << endl;
  //   if (dist < dist_threshold && dist < min_dist) {
  //     min_dist = dist;
  //     min_collision_pos = i;
  //   }
  // }
  // // The closer the collission, and the closer the distance, the higher the cost.
  // // The smallest cost should be close to 0.
  // cost = ((trajectory.s.size()-min_collision_pos) * weight);


  // ---END - AHEAD AND BEHIND---

  // ---SIMPLISTIC---
  // double min_dist = DBL_MAX;
  // // Detect collision
  // for (int i = 0; i < other_cars.size(); ++i) {
  //   ego::OtherCar car = closest_ahead[i];
  //   double cs = car.position.s;
  //   double cd = car.position.d;

  //   double s = trajectory.s[trajectory.s.size()-1];
  //   double d = trajectory.d[trajectory.d.size()-1];

  //   double s_dist = cs - s;
  //   double d_dist = abs(d2lane(cd) - d2lane(d));

  //   if (s_dist < min_dist && d_dist == 0) {
  //     min_dist = s_dist;
  //   }

  // }

  // if (min_dist < dist_threshold) {
  //   cost = (dist_threshold - min_dist) * weight;
  // }

  // cout << "collision check: " << s_dist << ", " << d_dist << endl;
  // if (min_car_dist <= dist_threshold) {
  //   cost = (dist_threshold-min_car_dist) * weight;
  // }


  // if (state == ego::STATE_FC) {
  //   cost -= 1;
  // }
  return cost;
}

/** 
 * Small cost for changing state
 */
double ChangeStateCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                       const ego::Trajectory &trajectory, double weight) {
  ego::State state = get<0>(cf_state);
  ego::State prev_state = get<1>(cf_state).state;
  double cost = 0.0;
  if (state != prev_state) {
    cost = weight;
  }
  return cost;
}

/**
 * Jerk is basically just one differentiation away from acceleration.
 * Find jerk in the trajectory by comparing various points.
 * Here is a cool diagram to illustrate it:
 *
 *        _j_
 *       |   |
 *      _a_ _a_
 *     |   |   |
 *    _v_ _v_ _v_
 *   |   |   |   |
 *   s   s   s   s
 *
 * To find jerk, we need at least four points
 */
double JerkAndAccelCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                        const ego::Trajectory &trajectory, double weight_j,
                        double weight_a, double max_allowed_j,
                        double max_allowed_a) {

  double cost = 0.0;
  if (trajectory.x.size() >= 4) {
    double dt = get<1>(cf_state).config.dt;
    vector<double> x = {0.0, 0.0, 0.0, 0.0};
    vector<double> y = {0.0, 0.0, 0.0, 0.0};
    vector<double> v = {0.0, 0.0, 0.0};
    vector<double> a = {0.0, 0.0};
    double j = 0.0;
    double max_j = 0.0;
    double max_a = 0.0;
    for (int i = 3; i < trajectory.x.size(); ++i) {
      x[0] = trajectory.x[i-3];
      x[1] = trajectory.x[i-2];
      x[2] = trajectory.x[i-1];
      x[3] = trajectory.x[i];
      
      y[0] = trajectory.y[i-3];
      y[1] = trajectory.y[i-2];
      y[2] = trajectory.y[i-1];
      y[3] = trajectory.y[i];

      v[0] = distance(x[0], y[0], x[1], y[1]) / dt;
      v[1] = distance(x[1], y[1], x[2], y[2]) / dt;
      v[2] = distance(x[2], y[2], x[3], y[3]) / dt;

      a[0] = (v[1] - v[0]) / dt;
      a[1] = (v[2] - v[1]) / dt;

      j = a[1] - a[0];
      if (a[0] > max_a && a[0] > max_allowed_a) {
        max_a = a[0];
      }
      if (a[1] > max_a && a[1] > max_allowed_a) {
        max_a = a[1];
      }
      if (j > max_j && j > max_allowed_j) {
        max_j = j;
      }
    }
    // cout << "jerk: " << max_j << endl;
    // cout << weight_a << ", " << max_a << ", " << weight_j << ", " << max_j << endl;
    cost = weight_a * max(max_a-max_allowed_a, 0.0) + weight_j * max(max_j-max_allowed_j, 0.0);
  }

  return cost;
}

/**
 * Calculate cost of given state and cars tuple when our ego car runs a trajectory.
 * the costs are weighted according to a given CostWeights object.
 */
double CalculateCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                     const ego::Trajectory &trajectory,
                     const ego::CostWeights &weights,
                     bool verbose=false) {
  double total_cost = 0.0;
  double collision_cost = CollisionCost(cf_state, trajectory, weights.collision);
  double efficiency_cost = EfficiencyCost(cf_state, trajectory, weights.efficiency);
  double jerk_and_accel_cost = JerkAndAccelCost(
    cf_state, trajectory, weights.max_jerk, weights.max_accel,
    get<1>(cf_state).config.max_jerk,
    get<1>(cf_state).config.default_max_acceleration);
  double change_state_cost = ChangeStateCost(cf_state, trajectory, weights.change_state);
  total_cost = collision_cost + efficiency_cost + jerk_and_accel_cost + change_state_cost;
  total_cost = collision_cost;
  cout << "collision_cost: " << collision_cost << endl;
  // cout << "efficiency_cost: " << efficiency_cost << endl;
  // cout << "jerk_and_accel_cost: " << jerk_and_accel_cost << endl;
  // cout << "change_state_cost: " << change_state_cost << endl;
  // cout << "total cost: " << total_cost << endl;

  return total_cost;
}

}
}
#endif