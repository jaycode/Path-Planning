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
  return (abs(max_s-trajectory.distance) * weight);
}

bool FindCarInCell(const vector<double> &target, double w1, double w2,
                   double h1, double h2, const vector<double> &ref,
                   bool reverse) {

  if ((target[1] > (w1 + ref[1])) &&
      (target[1] < (w2 + ref[1])) &&
      (target[0] > (h1 + ref[0])) &&
      (target[0] < (h2 + ref[0]))) {
    return true;
  }
  else {
    return false;
  }
}


/**
 * For each point in trajectory, check if there is any car that will cross path
 * at the same dt.
 */
  
double CollisionCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                     const ego::Trajectory &trajectory, double weight) {

  double cost = 0.0;
  double dist_threshold = 10.0;
  ego::Snapshot snap = get<1>(cf_state);
  vector<ego::OtherCar> cars = get<2>(cf_state);
  ego::State state = get<0>(cf_state);
  double dt = snap.config.dt;

  // Check trajectory points from the last point down to this id:
  double s_fin = trajectory.s[trajectory.s.size()-1];
  double d_fin = trajectory.d[trajectory.s.size()-1];
  double t_length = trajectory.s[trajectory.s.size()-1] - trajectory.s[0];

  // ---VIZ---
  // Visualize other cars' positions with grid cells.
  // The reference point is located at the center.
  // 
  // -------------
  // |   |   |   |
  // -------------
  // |   |   |   |
  // -------------
  // |   |   |   |
  // -------------

  int lanes = 3;
  int rows = 3;
  double cw = 4.0; // cell width
  double cl = 4.0; // cell length

  bool reverse = false;
  if (trajectory.s.size() > 1) {
    if (trajectory.s[trajectory.s.size()-1] < trajectory.s[trajectory.s.size()-2]) {
      reverse = true;
    }
  }
  // vector<double> ref = {snap.position.s, snap.position.d};  
  vector<double> ref = {s_fin, d_fin};  

  vector<char> c = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
  // Can do something smart here, but better make it easy to read.
  for (int k = 0; k < cars.size(); ++k) {
    vector<double> car = {cars[k].position.s, cars[k].position.d};
    // ref[0] = trajectory.s[i];
    // ref[1] = trajectory.d[i];
    // car[0] += (trajectory.s.size()-1)*dt*cars[k].position.v;

    // Defaults to up = positive numbers

    double dist = car[0] - s_fin;
    if (FindCarInCell(car,(-3*cw/2), (-cw/2), (cl/2),(3*cl/2),ref, reverse) == true) {
      c[0] = '*';
    }
    if (FindCarInCell(car,  (-cw/2),   (cw/2), (cl/2),(3*cl/2),ref, reverse) == true) {
      c[1] = '*';
      cost = max((dist_threshold - dist) * weight, cost);
    }
    if (FindCarInCell(car, (cw/2), (3*cw/2), (cl/2),(3*cl/2),ref, reverse) == true) {
      c[2] = '*';
    }
    if (FindCarInCell(car,(-3*cw/2), (-cw/2), (-cl/2),(cl/2),ref, reverse) == true) {
      c[3] = '*';
      if (state == ego::STATE_LCL) {
        cost = max(0.8 * (dist_threshold - dist) * weight, cost);
      }
    }
    if (FindCarInCell(car,(-cw/2), (cw/2), (-cl/2),(cl/2),ref, reverse) == true) {
      c[4] = '*';
      cost = max(1.5 * (dist_threshold - dist) * weight, cost);
      if (state == ego::STATE_FC) {
        cost = max(0.5 * (dist_threshold - dist) * weight, cost);
      }
    }
    if (FindCarInCell(car,(cw/2), (3*cw/2), (-cl/2),(cl/2),ref, reverse) == true) {
      c[5] = '*';
      if (state == ego::STATE_LCL) {
        cost = max(0.8 * (dist_threshold - dist) * weight, cost);
      }
    }
    if (FindCarInCell(car,(-3*cw/2), (-cw/2), -t_length,(-cl/2),ref, reverse) == true) {
      c[6] = '*';
      if (state == ego::STATE_LCL) {
        cost = max(1.0 * (dist_threshold - dist) * weight, cost);
      }
    }
    if (FindCarInCell(car,(-cw/2), (cw/2), -t_length,(-cl/2),ref, reverse) == true) {
      c[7] = '*';
      cost = max(1.0 * (dist_threshold - dist) * weight, cost);
      if (state == ego::STATE_FC) {
        cost: max(0.1 * (dist_threshold - dist) * weight, cost);
      }
    }
    if (FindCarInCell(car,(cw/2), (3*cw/2), -t_length,(-cl/2),ref, reverse) == true) {
      c[8] = '*';
      if (state == ego::STATE_LCR) {
        cost = max(1.0 * (dist_threshold - dist) * weight, cost);
      }
    }
  }

  // if (c[1] == '*' || c[4] == '*' || c[7] == '*') {
    cout <<
    "-------------" << endl <<
    "| "<< c[0] <<" | "<< c[1] <<" | "<< c[2] <<" |" << endl <<
    "-------------" << endl <<
    "| "<< c[3] <<" | "<< c[4] <<" | "<< c[5] <<" |" << endl <<
    "-------------" << endl <<
    "| "<< c[6] <<" | "<< c[7] <<" | "<< c[8] <<" |" << endl <<
    "-------------" << endl;
  // }
  // ---END - VIZ---

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
    double max_a_cost = weight_a * max(max_a-max_allowed_a, 0.0);
    double jerk_cost = weight_j * max(max_j-max_allowed_j, 0.0);

    cout << "Jerk cost: " << jerk_cost << endl;
    cout << "Max A cost: " << max_a_cost << endl;
    cost = max_a_cost + jerk_cost;
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
  // double jerk_and_accel_cost = JerkAndAccelCost(
  //   cf_state, trajectory, weights.max_jerk, weights.max_accel,
  //   get<1>(cf_state).config.max_jerk,
  //   get<1>(cf_state).config.default_max_acceleration);
  double change_state_cost = ChangeStateCost(cf_state, trajectory, weights.change_state);
  // total_cost = collision_cost + efficiency_cost + jerk_and_accel_cost + change_state_cost;
  total_cost = collision_cost + efficiency_cost + change_state_cost;
  cout << "collision_cost: " << collision_cost << endl;
  cout << "efficiency_cost: " << efficiency_cost << endl;
  // cout << "jerk_and_accel_cost: " << jerk_and_accel_cost << endl;
  cout << "change_state_cost: " << change_state_cost << endl;
  cout << "total cost: " << total_cost << endl;

  return total_cost;
}

}
}
#endif