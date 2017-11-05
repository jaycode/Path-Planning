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

    double TangentialMovementCost(const vector<double> &tj,
                        double target_s,
                        double target_v,
                        double max_a,
                        double max_jerk,
                        double dt) {
      /**
       * Raises flag (i.e. large error) when either:
       * - maximum velocity,
       * - maximum acceleration,
       * - or maximum jerk,
       * is higher than the allowed limit.
       */
      double max_v = 0.0;
      double cost = 0.0;
      // cost = fabs(target_v - velocity(tj[(int)tj.size()-1], tj[(int)tj.size()-2], dt));
      for (int i = 1; i < (int)tj.size(); ++i) {

        double v = velocity(tj[i], tj[i-1], dt);
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
          double at = acceleration(tj[i],
                                   tj[i-1],
                                   tj[i-2],
                                   dt);
          if (fabs(at) > max_a) {
            cost = 997.0;
            // cout << "err_acc (" << at << " > " << max_a << ")" << endl;
            break;
          }

          // TODO: Not sure how to limit jerk. Enabling this code breaks the system.
          // if (i > 2) {
          //   double at1 = acceleration(tj[i-1],
          //                             tj[i-2],
          //                             tj[i-3],
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

    // double LaneDeviationCost(const vector<double> &fwp_state, double target_lane) {
    double LaneDeviationCost(const vector<double> &tj_d, double target_d) {
      // double cost = fabs(lane2d(target_lane) - fwp_state[3]);
      double cost = 0.0;
      for (int i = 0; i < tj_d.size(); ++i) {
        double c = fabs(target_d - fabs(tj_d[i]));
        cost += c;
      }
      return cost;
    }

    double LateralMovementCost(const vector<double> &tj,
                        double target_d,
                        double target_v,
                        double max_a,
                        double max_jerk,
                        double dt) {
      /**
       * Raises flag (i.e. large error) when either:
       * - maximum velocity,
       * - maximum acceleration,
       * - or maximum jerk,
       * is higher than the allowed limit.
       */
      double max_v = 0.0;
      double cost = 0.0;
      // cost = fabs(target_v - velocity(tj[(int)tj.size()-1], tj[(int)tj.size()-2], dt));
      for (int i = 1; i < (int)tj.size(); ++i) {

        double v = velocity(tj[i], tj[i-1], dt);
        if (v > max_v) {
          max_v = v;
        }
        cost = fabs(target_v - v);

        if (i > 1) {
          double at = acceleration(tj[i],
                                   tj[i-1],
                                   tj[i-2],
                                   dt);
          if (fabs(at) > max_a) {
            cost = max_a - fabs(at);
            // cout << "err_acc (" << at << " > " << max_a << ")" << endl;
            // break;
          }

          // TODO: Not sure how to limit jerk. Enabling this code breaks the system.
          // if (i > 2) {
          //   double at1 = acceleration(tj[i-1],
          //                             tj[i-2],
          //                             tj[i-3],
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


      cost += LaneDeviationCost(tj, target_d);

      if (max_v > target_v) {
        cost += (max_v - target_v);
      }

      if (cost == 0.0) {
        // cout << "-" << endl;
      }
      return cost;
    }

    double LaneChangeCost(const vector<double> &fwp_state, double target_lane) {
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
        case 1: cost = 20.0;
                break;
        case 2: cost = 1001.0;
                break;
      }

      return(cost);
    }

    double CollisionCost(const vector<double> &tj_s,
                         const vector<double> &tj_d,
                         const json &sensor_fusion,
                         double dt) {
      /**
       * See if an obstacle exists within a given trajectory.
       * TODO: Predict obstacle movements.
       * 
       * INPUTS:
       * sensor_fusion: [ id, x, y, vx, vy, s, d]
       */
      double cost = 0;

      double ds;
      for (int i = 1; i < (int)tj_s.size(); ++i) {
        int car_lane = d2lane(tj_d[i]);
        ds = fabs(tj_s[i] - tj_s[i-1]);
        for (int j = 0; j < (int)sensor_fusion.size(); ++j) {
          int obs_lane = d2lane((double)sensor_fusion[j][6]);
          // cout << "car lane: " << car_lane << " obs_lane: " << obs_lane << endl;
          if (car_lane == obs_lane) {
            // cout << "check if " << (double)sensor_fusion[j][5] <<
            //         " is between " << ((double)tj_s[i] - d) << " and " <<
            //         ((double)tj_s[i] + d) << endl;
            // if ((double)sensor_fusion[j][5] > (double)tj_s[i] - ds &&
            //     (double)sensor_fusion[j][5] <= (double)tj_s[i]) {
            //   // cout << "yep" << endl;
            //   cout << "Car " << sensor_fusion[j][0] << " (" <<
            //           sensor_fusion[j][5] << ", " << sensor_fusion[j][6] << ")" <<
            //           " was found within " << ((double)tj_s[i] - ds) <<
            //           " and " << ((double)tj_s[i]) << " (lane " << car_lane << ")" <<
            //           endl;
            //   cost = 999.0;
            //   // One of the few cases where using goto is forgivable:
            //   // To quit multiple loops.
            //   // Ref: http://en.cppreference.com/w/cpp/language/goto
            //   goto out;
            // }

            // Add by velocity
            double vx = (double)sensor_fusion[j][3];
            double vy = (double)sensor_fusion[j][4];
            double v_norm = 0.8;
            double v = sqrt(vx*vx + vy*vy) * v_norm;
            double obs_s = (double)sensor_fusion[j][5] + (i*dt*v);
            // cout << "obs: " << obs_s << ", s range: " << 
            //         ((double)tj_s[i] - ds) << " to " << 
            //         ((double)tj_s[i]) << endl;
            if (obs_s > (double)tj_s[i] - ds && obs_s <= (double)tj_s[i]) {
              // cout << "yep" << endl;
              cout << "Car " << sensor_fusion[j][0] << " +v (" <<
                      obs_s << ", " << sensor_fusion[j][6] << ")" <<
                      " was found within " << ((double)tj_s[i] - ds) <<
                      " and " << ((double)tj_s[i]) << " (lane " << car_lane << ")" <<
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

    double SimpleCollisionCost(double car_s, double target_lane,
                               const json &sensor_fusion) {

      double cost = 0.0;
      double dist_ahead = 60;
      double dist_behind = -2;
      for (int i = 0; i < (int)sensor_fusion.size(); ++i) {
        int obs_lane = d2lane((double)sensor_fusion[i][6]);
        // cout << "obs_lane " << obs_lane << ", target_lane " << target_lane << endl;
        if (obs_lane == target_lane) {
          double obs_s = (double)sensor_fusion[i][5];
          // cout << "checks if " << obs_s << " is between " << (car_s + dist_ahead) <<
          //         " and " << (car_s + dist_behind) << endl;
          if (obs_s <= (car_s + dist_ahead) && obs_s >= (car_s + dist_behind)) {
            cost = 100.0;
            break;
          }
        }
      }
      return cost;
    }

    double DistanceCost(const vector<double> &tj_s) {
      /**
       * The farther the car travels, the smaller this cost will be.
       */
      double dist = (tj_s[tj_s.size()-1] - tj_s[0]);
      cout << "dist: " << dist << endl;
      return -dist;
    }

    void TestCollisionCost() {
      // The car is travelling from center lane
      // to right lane.
      vector<double> tj_s = {1, 2, 3, 4, 5, 6,  7,  8,  9,  10};
      vector<double> tj_d = {6, 6, 7, 8, 9, 10, 10, 10, 10, 10};

      // [ id, x, y, vx, vy, s, d]
      json sf1 = {
        {0, 0, 0, 0, 0, 2.0, 10.0}
      };
      double cost1 = CollisionCost(tj_s, tj_d, sf1, 0.02);
      assert(cost1 < 999.0);

      json sf2 = {
        {0, 0, 0, 0, 0, 7.0, 10.0}
      };
      double cost2 = CollisionCost(tj_s, tj_d, sf2, 0.02);
      assert(cost2 == 999.0);
    }

    void TestSimpleCollisionCost() {
      double car_s = 10;
      double target_lane = 2;
      json sf = {
        {0, 0, 0, 0, 0, 12.2, 9}
      };
      assert(SimpleCollisionCost(car_s, target_lane, sf) >= 100.0);
    }

  }

}

#endif