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
      double cost = 0.0;
      for (int i = 1; i < (int)tj_s.size(); ++i) {

        double v = velocity(tj_s[i], tj_s[i-1], dt);
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
            cost += 997.0;
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

      if (cost == 0.0) {
        // cout << "-" << endl;
      }
      return cost;
    }

    double LaneCost(const vector<double> &initial_state,
                    const json &sensor_fusion,
                    double lane) {
      /**
       * Calculate the cost of a lane given current vehicle and other vehicles.
       */
      double s = initial_state[0];
      double dist = 10.0;

      return(0.0);
    }
    
  }

}

#endif