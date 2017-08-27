#ifndef COST_H
#define COST_H

#include <fstream>
#include <tuple>
#include "base.h"
#include <string>
#include "vehicle.h"
#include <math.h>
#include "helpers.h"

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
  double fin_v;
  // cout << "final speed: " << fin_v << endl;
  return ((-trajectory.distance * weight) + (-fin_v * weight));
}

/**
 * For each point in trajectory, check if there is any car that will cross path
 * at the same dt.
 */
  
  
double CollisionCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                     const ego::Trajectory &trajectory, double weight) {

  double cost;
  double s_threshold = 10.0;
  double d_threshold = 8.0;
  vector<ego::OtherCar> cars = get<2>(cf_state);
  ego::State state = get<0>(cf_state);
  double dt = get<1>(cf_state).config.dt;

  // Simple collision model (only look at final car position):

  // int i = trajectory.s.size()-1;
  // double s = trajectory.s[i];
  // double d = trajectory.d[i];

  // for (int j = 0; j < cars.size(); ++j) {
  //   ego::OtherCar car = cars[j];

  //   // TODO: There is a possibility the car is no longer
  //   //       at the same d location. Will work on this later.
  //   double cd = car.position.d;
  //   // cout << "Checking cars in lane " << d2lane(d) << endl;
  //   // cout << "looking at car with cd " << cd;
  //   if (d2lane(cd) == d2lane(d)) {
  //     // if (d2lane(cd) == d2lane(trajectory))
  //     double cs = car.position.s + i*car.config.dt*car.position.v;
  //     double s_dist = cs - s;
  //     double d_dist = abs((double)d2lane(cd) - d);
  //     // cout << "dt is " << car.config.dt << endl;
  //     // cout << "collision check: " << s_dist << ", " << d_dist << endl;
  //     if (s_dist < s_threshold && s_dist > 0 &&
  //         d_dist < d_threshold) {
  //       // If the car does not want to turn even when it is about to collide,
  //       // check `horizon` setting in RealizeChangeLine method.

  //       cout << "I think I'll collide into car " << j <<
  //         (i*car.config.dt) << " seconds from now when I am at [s,d]: " << s << ", " << d << endl;
  //       cout << "s_dist: " << s_dist << " d_dist: " << d_dist << endl;
  //       return (d_dist*2) * weight + (s_dist/2) * weight;
  //     }
  //   }
  // }

  // More complex collision model (compare multiple trajectories):

  // cout << "check collision. Num trajectory points: " << trajectory.s.size() << endl;
  int i = trajectory.s.size()-1;
  double s_fin = trajectory.s[i];
  double d_fin = trajectory.d[i];
  // cout << "At " << (double)(i*dt) << "s, the car will have position [s,d]: "
  //   << s_fin << ", " << d_fin << endl; 

  // cout << "Size of trajectory: " << trajectory.s.size() << endl;

  for (int i = 0; i < trajectory.s.size(); ++i) {
    double s = trajectory.s[i];
    double d = trajectory.d[i];
    // cout << "Checks at lane " << d2lane(d) << endl;
    for (int j = 0; j < cars.size(); ++j) {
      ego::OtherCar car = cars[j];

      // TODO: There is a possibility the car is no longer
      //       at the same d location. Will work on this later.
      double cd = car.position.d;
      // if (d2lane(cd) == d2lane(trajectory))
      double cs = car.position.s + (double)(i+1)*car.config.dt*car.position.v;
      double s_dist = cs - s;
      double d_dist = abs(cd - d);
      // cout << "collision check: " << s_dist << ", " << d_dist << endl;
      if (s_dist < s_threshold && s_dist > 0 &&
          d_dist < d_threshold) {
        // If the car does not want to turn even when it is about to collide,
        // check `horizon` setting in RealizeChangeLine method.
        // cout << "car.config.dt is " << car.config.dt << " and i is " << i << endl;
        // cout << "+1 together they are " << ((double)(i+1)*car.config.dt) << endl;
        // cout << "I think I'll collide into car " << j << "(" << cs << ", " << cd << ") " <<
        //   ((double)(i+1)*car.config.dt) << " seconds from now when I am at [s,d]: " << s << ", " << d << endl;
        // cout << "s_dist: " << s_dist << " d_dist: " << d_dist << endl;
        cost += ((d_dist*2) * weight + (s_dist/2) * weight);
      }
    }
  }
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
  // cout << "collision_cost: " << collision_cost << endl;
  // cout << "efficiency_cost: " << efficiency_cost << endl;
  // cout << "jerk_and_accel_cost: " << jerk_and_accel_cost << endl;
  // cout << "change_state_cost: " << change_state_cost << endl;
  // cout << "total cost: " << total_cost << endl;

  return total_cost;
}

}
}
#endif