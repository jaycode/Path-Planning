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

    double MovementCost(const tuple<vector<double>, vector<double>> &traj,
                            double target_v,
                            double max_at,
                            double max_jerk,
                            double dt) {
      /**
       * Raises flag (i.e. large error) when either:
       * - maximum velocity,
       * - maximum acceleration,
       * - or maximum jerk,
       * is higher than the allowed limit.
       */
      vector<double> tj_s = get<0>(traj);
      vector<double> tj_d = get<1>(traj);
      double max_v = 0.0;
      double cost = 0.0;
      // cost = fabs(target_v - velocity(tj_s[(int)tj_s.size()-1], tj_s[(int)tj_s.size()-2], dt));
      for (int i = 1; i < (int)tj_s.size(); ++i) {

        double v = velocity(tj_s[i], tj_s[i-1], dt);
        if (v > max_v) {
          max_v = v;
        }
        cost = fabs(target_v - v);
        if (v < 0.0) {
          cost = 999.0;
          // cout << "err_orient" << endl;
          break;
        }

        if (i > 1) {
          double at = acceleration(tj_s[i], // tj_d[i],
                                   tj_s[i-1], // tj_d[i-1],
                                   tj_s[i-2], // tj_d[i-2],
                                   dt);
          if (fabs(at) > max_at) {
            cost = 997.0;
            // cout << "err_acc (" << at << " > " << max_at << ")" << endl;
            break;
          }

          // TODO: Not sure how to limit jerk. Enabling this code breaks the system.
          // if (i > 2) {
          //   double at1 = acceleration(tj_s[i-1],
          //                             tj_s[i-2],
          //                             tj_s[i-3],
          //                             dt);
          //   double jerk = (at - at1) / dt;
          //   if (fabs(jerk) > max_jerk) {
          //     cost += 999.0;
          //     cout << "err_jerk (" << jerk << " > " << max_jerk << ")" << endl;
          //     break;
          //   }
          // }
        }
      }

      if (max_v > target_v) {
        cost += (max_v - target_v);
      }

      if (cost == 0.0) {
        // cout << "-" << endl;
      }
      return cost;
    }

    double LaneChangeCost(const vector<double> &fwp_state,
                          double target_lane) {
      /**
       * Calculate the cost of a lane given current vehicle and other vehicles.
       */
      double cost = 0.0;
      double s = fwp_state[0];
      double dist = 10.0;
      int current_lane = d2lane(fwp_state[3]);

      // Incur a small cost for changing lane
      int lane_diff = abs(current_lane - target_lane);
      switch(lane_diff) {
        case 1: cost = 1.0;
                break;
        case 2: cost = 10.0;
                break;
      }

      return(cost);
    }

    double CollisionCost(const vector<double> &tj_s,
                         const vector<double> &tj_d,
                         const json &sensor_fusion) {
      /**
       * See if an obstacle exists within a given trajectory.
       * TODO: Predict obstacle movements.
       * 
       * INPUTS:
       * sensor_fusion: [ id, x, y, vx, vy, s, d]
       */
      double cost = 0;
      double d = 0.5;
      for (int i = 0; i < (int)tj_s.size(); ++i) {
        int car_lane = d2lane(tj_d[i]);
        for (int j = 0; j < (int)sensor_fusion.size(); ++j) {
          int obs_lane = d2lane((double)sensor_fusion[j][6]);
          // cout << "car lane: " << car_lane << " obs_lane: " << obs_lane << endl;
          if (car_lane == obs_lane) {
            // cout << "check if " << (double)sensor_fusion[j][5] <<
            //         " is between " << ((double)tj_s[i] - d) << " and " <<
            //         ((double)tj_s[i] + d) << endl;
            if ((double)sensor_fusion[j][5] > (double)tj_s[i] - d &&
                (double)sensor_fusion[j][5] < (double)tj_s[i] + d) {
              // cout << "yep" << endl;
              cout << "Car " << sensor_fusion[j][0] << " (" <<
                      sensor_fusion[j][5] << ", " << sensor_fusion[j][6] << ")" <<
                      " was found within " << ((double)tj_s[i] - d) <<
                      " and " << ((double)tj_s[i] + d) << " (lane " << car_lane << ")" <<
                      endl;
              cost = 999.0;
              // One of the few cases where using goto is forgivable:
              // To quit multiple loops.
              // Ref: http://en.cppreference.com/w/cpp/language/goto
              goto out;
            }
          }
        }
      }
out:
      return cost;

    }

    void TestCollisionCost() {
      // The car is travelling from center lane
      // to right lane.
      vector<double> tj_s = {1, 2, 3, 4, 5, 6,  7,  8,  9,  10};
      vector<double> tj_d = {6, 6, 7, 8, 9, 10, 10, 10, 10, 10};

      // [ id, x, y, vx, vy, s, d]
      json sf1 = {
        {0, 0, 0, 0, 0, 3.0, 10.0}
      };
      double cost1 = CollisionCost(tj_s, tj_d, sf1);
      assert(cost1 < 999.0);

      json sf2 = {
        {0, 0, 0, 0, 0, 7.0, 10.0}
      };
      double cost2 = CollisionCost(tj_s, tj_d, sf2);
      assert(cost2 == 999.0);
    }

  }

}

#endif