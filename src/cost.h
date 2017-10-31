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

    double CalcOrientationCost(const tuple<vector<double>, vector<double>> &traj) {
      /**
       * Is the vehicle going in the right direction?
       */
      double cost = 0.0;
      if (IsWrongDirection(traj)) {
        cost = 999.0;
      }
      return cost;
    }

    double CalcSpeedCost(const tuple<vector<double>, vector<double>> &traj,
                         double target_v,
                         double dt) {
      /**
       * How close is final waypoints' speed to the target speed?
       */
      vector<double> tj_s = get<0>(traj);
      vector<double> state = StateFromTJ(tj_s, dt);
      double v = state[1];
      return (fabs(target_v - v));
    }

    bool IsWrongDirection(const tuple<vector<double>, vector<double>> &traj) {
      /**
       * If the vehicle is going the opposite direction, return false.
       */

      vector<double> tj_s = get<0>(traj);

      bool wrong_dir = false;
      for (int i = 0; i < tj_s.size(); ++i) {
        if (i > 0 and tj_s[i] < tj_s[i-1]) {
          wrong_dir = true;
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

    void testCalcSpeedCost() {

      tuple<vector<double>, vector<double>> traj;
      vector<double> tj_s = {1.0, 3.0, 3.5};
      vector<double> tj_d = {1.0, 1.0, 1.0};
      double dt = 0.02;
      double target_v = 0.5 / 0.02;

      get<0>(traj) = tj_s;
      get<1>(traj) = tj_d;

      assert(CalcSpeedCost(traj, target_v, dt) == 0.0);

    }
  }

}

#endif