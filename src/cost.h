#ifndef COST_H
#define COST_H

#include <fstream>
#include <tuple>
#include "helpers.h"

namespace {
  namespace cost {

    using namespace std;
    using json = nlohmann::json;
    using namespace helpers;

    bool IsWrongDirection(const tuple<vector<double>, vector<double>> &traj);

    double OrientationCost(const tuple<vector<double>, vector<double>> &traj) {
      /**
       * Raises flag when the vehicle goes the opposite direction.
       */
      double cost = 0.0;
      if (IsWrongDirection(traj)) {
        cost = 999.0;
        cout << "failed orientation" << endl;
      }
      return cost;
    }

    double SpeedCost(const tuple<vector<double>, vector<double>> &traj,
                     double target_v,
                     double dt) {
      /**
       * How close is final waypoints' speed to the target speed?
       */
      vector<double> tj_s = get<0>(traj);
      vector<double> state = StateFromTJ(tj_s, dt);
      double v = state[1];
      // cout << "target_v | v: " << target_v << " | " << v << endl;
      return (fabs(target_v - v));
    }

    double AccelerationCost(const tuple<vector<double>, vector<double>> &traj,
                            double max_at,
                            double max_an,
                            double dt,
                            const vector<double> map_waypoints_s,
                            const vector<double> map_waypoints_x,
                            const vector<double> map_waypoints_y) {
      /**
       * Raises flag when maximum acceleration is higher than the allowed limit.
       */
      vector<double> tj_s = get<0>(traj);
      vector<double> tj_d = get<1>(traj);
      double cost = 0.0;
      for (int i = 2; i < (int)tj_s.size(); ++i) {

        vector<double> xy1 = getXY(tj_s[i], tj_d[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> xy2 = getXY(tj_s[i-1], tj_d[i-1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> xy3 = getXY(tj_s[i-2], tj_d[i-2], map_waypoints_s, map_waypoints_x, map_waypoints_y);

        // double at = acceleration(xy1[0], xy1[1],
        //                          xy2[0], xy2[1],
        //                          xy3[0], xy3[1],
        //                          dt);
        double at = acceleration(tj_s[i], // tj_d[i],
                                 tj_s[i-1], // tj_d[i-1],
                                 tj_s[i-2], // tj_d[i-2],
                                 dt);
        if (at > max_at) {
          cost = 999.0;
          cout << "err_acc (" << at << " > " << max_at << ")" << endl;
          break;
        }
      }

      if (cost == 0.0) {
        // cout << "-" << endl;
      }
      return cost;
    }

    double ChangeLaneCost(const vector<double> initial_state, double target_d) {
      /**
       * Incur a small cost for changing lane.
       */

      return(fabs(initial_state[3] - target_d));
    }

    bool IsWrongDirection(const tuple<vector<double>, vector<double>> &traj) {
      /**
       * If the vehicle is going the opposite direction, return false.
       */

      vector<double> tj_s = get<0>(traj);

      bool wrong_dir = false;
      for (int i = 1; i < tj_s.size(); ++i) {
        if (tj_s[i] < tj_s[i-1]) {
          wrong_dir = true;
          // cout << "err_dir (" << tj_s[i] << " < " << tj_s[i-1] << ") ";
          break;
        }
      }
      return wrong_dir;
    }

    void testIsWrongDirection() {

      // Direction is correct: s gets larger on each step.
      tuple<vector<double>, vector<double>> traj;
      vector<double> tj_s = {1.0, 2.0, 3.0};
      vector<double> tj_d = {1.0, 1.0, 1.0};

      get<0>(traj) = tj_s;
      get<1>(traj) = tj_d;

      assert(IsWrongDirection(traj) == false);

      // Direction is incorrect: s gets smaller in one of the steps.
      vector<double> tj_s1 = {1.0, 2.0, 1.9};
      vector<double> tj_d1 = {1.0, 1.0, 1.0};

      get<0>(traj) = tj_s1;
      get<1>(traj) = tj_d1;
      assert(IsWrongDirection(traj));

    }

    void testSpeedCost() {

      tuple<vector<double>, vector<double>> traj;
      vector<double> tj_s = {1.0, 3.0, 3.5};
      vector<double> tj_d = {1.0, 1.0, 1.0};
      double dt = 0.02;
      double target_v = 0.5 / 0.02;

      get<0>(traj) = tj_s;
      get<1>(traj) = tj_d;

      assert(SpeedCost(traj, target_v, dt) == 0.0);

    }

    void testAccelerationCost() {

    }
  }

}

#endif