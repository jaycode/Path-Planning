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
bool is_changing_lane(double d) { 
  double center0 = lane2d(0);
  double center1 = lane2d(1);
  double center2 = lane2d(2);

  // cout << "d: " << d << endl;
  // cout << "c1: " << center0 << endl;
  // cout << "c2: " << center1 << endl;
  // cout << "c3: " << center2 << endl;
  // If car is not in the center of any lane.
  if (fabs(center0 - d) > 1.0 &&
      fabs(center1 - d) > 1.0 &&
      fabs(center2 - d) > 1.0) {
    // cout << "Car is changing lane" << endl;
    return true;
  }
  else {
    // cout << "Car is on a lane" << endl;
    return false;
  }
}

/**
 * Look at the s distance traveled and penalize smaller distance.
 */
double EfficiencyCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                      const ego::Trajectory &plan,
                      const ego::Trajectory &waypoints, double weight) {

  double dt = get<1>(cf_state).config.dt;
  ego::Snapshot snap = get<1>(cf_state);
  double max_v = snap.config.default_target_speed;
  double max_a = snap.config.default_max_acceleration;
  double max_j = snap.config.max_jerk;
  double t = dt * waypoints.s.size();
  // cout << "max_s = " << (t*max_v) << " + " << (0.5*max_a*t*t) << " + " <<
  //   ((1/6)*max_j*t*t*t) << endl;
  // // x = x0 + v0*t + (1/2)*a0*t^2 + (1/6)*j*t^3
  double max_s = max_v*t + (0.5*max_a*t*t) + ((1/6)*max_j*t*t*t);
  // cout << "max_s = " << max_s << endl;
  // cout << "waypoints distance = " << waypoints.distance
  //      << " over " << t << " seconds" << endl;
  double cost = 0;
  if (waypoints.distance > 0) {
    double cost = fabs(max_s-waypoints.distance)/waypoints.distance * weight;
  }
  // cout << "cost: " << cost << endl;
  return cost;
}

bool FindCarInCell(const vector<double> &target, double w1, double w2,
                   double h1, double h2, const vector<double> &ref,
                   bool reverse, string label) {

  // cout << endl << label << ": checks between " << (w1 + ref[1]) << " to " << (w2 + ref[1]) <<
  //   " in d, and " << (h1 + ref[0]) << " to " << (h2 + ref[0]) << " in s " <<
  //   endl;

  if ((target[1] > (w1 + ref[1])) &&
      (target[1] < (w2 + ref[1])) &&
      (target[0] > (h1 + ref[0])) &&
      (target[0] < (h2 + ref[0]))) {
    // cout << "**FOUND**" << endl;
    return true;
  }
  else {
    return false;
  }
}


/**
 * For each point in plan trajectory, check if there is any car that will cross path
 * at the same dt.
 */
  
double CollisionCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                     const ego::Trajectory &plan,
                     const ego::Trajectory &waypoints, double weight,
                     vector<char> *minimap) {

  double cost = 0.0;
  double dist_threshold = 10.0;
  ego::Snapshot snap = get<1>(cf_state);
  vector<ego::OtherCar> cars = get<2>(cf_state);
  ego::State state = get<0>(cf_state);
  double dt = snap.config.dt;

  // Check plan trajectory points from the last point down to this id:
  double s_fin = waypoints.s[waypoints.s.size()-1];
  double d_fin = waypoints.d[waypoints.s.size()-1];
  // double s_fin = plan.s[plan.s.size()-1];
  // double d_fin = plan.d[plan.s.size()-1];
  double t_length = plan.s[plan.s.size()-1] - plan.s[0];

  // The id of first plan plan point.
  // int plan_start_id = plan.s.size() - snap.config.num_pp;

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
  double cl = 10.0; // cell length

  bool reverse = false;
  if (plan.s.size() > 1) {
    if (plan.s[plan.s.size()-1] < plan.s[plan.s.size()-2]) {
      reverse = true;
    }
  }
  // vector<double> ref = {snap.position.s, snap.position.d};  
  vector<double> ref = {s_fin, d_fin};  

  vector<char> &c = (*minimap);
  // vector<char> c = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
  // Can do something smart here, but better make it easy to read.
  for (int k = 0; k < cars.size(); ++k) {
    vector<double> car = {cars[k].position.s, cars[k].position.d};
    // ref[0] = plan.s[i];
    // ref[1] = plan.d[i];
    // car[0] += (snap.config.num_p-1)*dt*cars[k].position.v;

    // Defaults to up = positive numbers
    // cout << endl << "---------" << endl;
    // cout << "ref sd: " << ref[0] << ", " << ref[1] << ", target sd = " <<
    //   car[0] << ", " << car[1] << endl << endl;
    // Waypoint model
    double dist = car[0] - s_fin;
    if (FindCarInCell(car,(-3*cw/2), (-cw/2), (cl/2),(3*cl/2),ref, reverse, "top left") == true) {
      c[0] = '*';
      switch (state) {
        case ego::STATE_LCL: {
          cost = 0.6 * fabs(dist_threshold - dist) * weight;
          break;
        }
        default: {
          
        }
      }
    }
    if (FindCarInCell(car,  (-cw/2),   (cw/2), (cl/2),(3*cl/2),ref, reverse, "top center") == true) {
      c[1] = '*';
      switch (state) {
        case ego::STATE_FC: {
          cost = 0.8 * fabs(dist_threshold - dist) * weight;
          break;
        }
        case ego::STATE_KL: {
          cost = max((dist_threshold - dist) * weight, cost);
          break;
        }
        default: {
          
        }
      }
    }
    if (FindCarInCell(car, (cw/2), (3*cw/2), (cl/2),(3*cl/2),ref, reverse, "top right") == true) {
      c[2] = '*';
      switch (state) {
        case ego::STATE_LCR: {
          cost = 0.6 * fabs(dist_threshold - dist) * weight;
          break;
        }
        default: {
          
        }
      }
    }
    if (FindCarInCell(car,(-3*cw/2), (-cw/2), (-cl/2),(cl/2),ref, reverse, "mid left") == true) {
      c[3] = '*';
      switch (state) {
        case ego::STATE_LCL: {
          cost = 1.0 * fabs(dist_threshold - dist) * weight;
          break;
        }
        default: {
          
        }
      }
    }
    if (FindCarInCell(car,(-cw/2), (cw/2), (-cl/2),(cl/2),ref, reverse, "mid center") == true) {
      c[4] = '*';

      // We use fabs and not max here since we do not want to have 0 cost when
      // center lane is occupied.
      switch (state) {
        case ego::STATE_FC: {
          cost = 0.3 * fabs(dist_threshold - dist) * weight;
          break;          
        }
        case ego::STATE_KL: {
          cost = 1.5 * fabs(dist_threshold - dist) * weight;
          break;
        }
        default: {
          
        }
      }
    }
    if (FindCarInCell(car,(cw/2), (3*cw/2), (-cl/2),(cl/2),ref, reverse, "mid right") == true) {
      c[5] = '*';
      switch (state) {
        case ego::STATE_LCR: {
          cost = 0.8 * fabs(dist_threshold - dist) * weight;
          break;
        }
        default: {

        }
      }
    }
    if (FindCarInCell(car,(-3*cw/2), (-cw/2), (-3*cl/2),(-cl/2),ref, reverse, "bottom left") == true) {
      c[6] = '*';
      switch (state) {
        case ego::STATE_LCL: {
          cost = 0.8 * fabs(dist_threshold - dist) * weight;
          break;
        }
        default: {
          
        }
      }
    }
    if (FindCarInCell(car,(-cw/2), (cw/2), (-3*cl/2),(-cl/2),ref, reverse, "bottom center") == true) {
      c[7] = '*';
    }
    if (FindCarInCell(car,(cw/2), (3*cw/2), (-3*cl/2),(-cl/2),ref, reverse, "bottom right") == true) {
      c[8] = '*';
      switch (state) {
        case ego::STATE_LCR: {
          cost = 0.8 * fabs(dist_threshold - dist) * weight;
          break;
        }
        default: {
          
        }
      }
    }

    // Plan model
    // double dist = car[0] - s_fin;
    // if (FindCarInCell(car,(-3*cw/2), (-cw/2), (cl/2),(3*cl/2),ref, reverse) == true) {
    //   c[0] = '*';
    // }
    // if (FindCarInCell(car,  (-cw/2),   (cw/2), (cl/2),(3*cl/2),ref, reverse) == true) {
    //   c[1] = '*';
    //   if (state == ego::STATE_FC) {
    //     cost = 0.8 * fabs(dist_threshold - dist) * weight;
    //   }
    //   else {
    //     cost = max((dist_threshold - dist) * weight, cost);
    //   }
    // }
    // if (FindCarInCell(car, (cw/2), (3*cw/2), (cl/2),(3*cl/2),ref, reverse) == true) {
    //   c[2] = '*';
    // }
    // if (FindCarInCell(car,(-3*cw/2), (-cw/2), (-cl/2),(cl/2),ref, reverse) == true) {
    //   c[3] = '*';
    //   if (state == ego::STATE_LCL) {
    //     cost = max(0.8 * (dist_threshold - dist) * weight, cost);
    //   }
    // }
    // if (FindCarInCell(car,(-cw/2), (cw/2), (-cl/2),(cl/2),ref, reverse) == true) {
    //   c[4] = '*';

    //   // We do not use max here since we do not want to have 0 cost when
    //   // center lane is occupied.
    //   if (state == ego::STATE_FC) {
    //     cost = 0.2 * fabs(dist_threshold - dist) * weight;
    //   }
    //   else {
    //     cost = 1.5 * fabs(dist_threshold - dist) * weight;
    //   }
    // }
    // if (FindCarInCell(car,(cw/2), (3*cw/2), (-cl/2),(cl/2),ref, reverse) == true) {
    //   c[5] = '*';
    //   if (state == ego::STATE_LCR) {
    //     cost = max(0.8 * (dist_threshold - dist) * weight, cost);
    //   }
    // }
    // if (FindCarInCell(car,(-3*cw/2), (-cw/2), (-3*cl/2),(-cl/2),ref, reverse) == true) {
    //   c[6] = '*';
    // }
    // if (FindCarInCell(car,(-cw/2), (cw/2), (-3*cl/2),(-cl/2),ref, reverse) == true) {
    //   c[7] = '*';
    //   // If after turning left or right the car gets behind us, there is a risk
    //   // of hitting it while turning.
    //   if (state == ego::STATE_LCL || state == ego::STATE_LCR) {
    //     cost = max(0.1 * (dist_threshold - dist) * weight, cost);
    //   }
    // }
    // if (FindCarInCell(car,(cw/2), (3*cw/2), (-3*cl/2),(-cl/2),ref, reverse) == true) {
    //   c[8] = '*';
    // }
  }

  // if (c[1] == '*' || c[4] == '*' || c[7] == '*') {
    // cout <<
    // "-------------" << endl <<
    // "| "<< c[0] <<" | "<< c[1] <<" | "<< c[2] <<" |" << endl <<
    // "-------------" << endl <<
    // "| "<< c[3] <<" | "<< c[4] <<" | "<< c[5] <<" |" << endl <<
    // "-------------" << endl <<
    // "| "<< c[6] <<" | "<< c[7] <<" | "<< c[8] <<" |" << endl <<
    // "-------------" << endl;
  // }
  // ---END - VIZ---

  return cost;
}

/** 
 * Small cost for changing state.
 * If in-between lane, huge cost.
 */
double ChangeStateCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                       const ego::Trajectory &plan,
                       const ego::Trajectory &waypoints, double weight) {
  ego::State state = get<0>(cf_state);
  ego::State prev_state = get<1>(cf_state).state;
  ego::Snapshot snap = get<1>(cf_state);
  double cost = 0.0;
  bool a = is_changing_lane(snap.position.d);

  if (state != prev_state) {
    // If the car is physically (not the end of waypoint) on top of a
    // lane line, do not allow line changing.
    if (is_changing_lane(snap.position.d)) {
      cost = 10*weight;
    }
    else {
      if (snap.state == ego::STATE_FC) {
        cost = 0;
      }
      else {
        cost = weight;
      }
    }
  }
  return cost;
}

/**
 * Jerk is basically just one differentiation away from acceleration.
 * Find jerk in the plan trajectory by comparing various points.
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
                        const ego::Trajectory &plan,
                        const ego::Trajectory &waypoints, double weight_j,
                        double weight_a, double max_allowed_j,
                        double max_allowed_a) {

  double cost = 0.0;
  if (plan.x.size() >= 4) {
    double dt = get<1>(cf_state).config.dt;
    vector<double> x = {0.0, 0.0, 0.0, 0.0};
    vector<double> y = {0.0, 0.0, 0.0, 0.0};
    vector<double> v = {0.0, 0.0, 0.0};
    vector<double> a = {0.0, 0.0};
    double j = 0.0;
    double max_j = 0.0;
    double max_a = 0.0;
    for (int i = 3; i < plan.x.size(); ++i) {
      x[0] = plan.x[i-3];
      x[1] = plan.x[i-2];
      x[2] = plan.x[i-1];
      x[3] = plan.x[i];
      
      y[0] = plan.y[i-3];
      y[1] = plan.y[i-2];
      y[2] = plan.y[i-1];
      y[3] = plan.y[i];

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

    // cout << "Jerk cost: " << jerk_cost << endl;
    // cout << "Max A cost: " << max_a_cost << endl;
    cost = max_a_cost + jerk_cost;
  }

  return cost;
}

/**
 * Calculate cost of given state and cars tuple when our ego car runs a trajectory.
 * the costs are weighted according to a given CostWeights object.
 */
double CalculateCost(const std::tuple<ego::State, ego::Snapshot, std::vector<ego::OtherCar>> &cf_state, 
                     const ego::Trajectory &plan,
                     const ego::Trajectory &waypoints,
                     const ego::CostWeights &weights,
                     vector<char> *minimap,
                     bool verbose=false) {
  double total_cost = 0.0;
  double collision_cost = CollisionCost(cf_state, plan, waypoints,
    weights.collision, minimap);
  double efficiency_cost = EfficiencyCost(cf_state, plan, waypoints, weights.efficiency);
  double change_state_cost = ChangeStateCost(cf_state, plan, waypoints, weights.change_state);
  total_cost = collision_cost + efficiency_cost + change_state_cost;

  // Unused for now.
  // double jerk_and_accel_cost = JerkAndAccelCost(
  //   cf_state, plan, waypoints, weights.max_jerk, weights.max_accel,
  //   get<1>(cf_state).config.max_jerk,
  //   get<1>(cf_state).config.default_max_acceleration);

  // total_cost = collision_cost + efficiency_cost + jerk_and_accel_cost + change_state_cost;

  // cout << "collision_cost: " << collision_cost << endl;
  // cout << "efficiency_cost: " << efficiency_cost << endl;
  // cout << "change_state_cost: " << change_state_cost << endl;
  // cout << "total cost: " << total_cost << endl;

  // cout << "jerk_and_accel_cost: " << jerk_and_accel_cost << endl;


  return total_cost;
}

}
}
#endif