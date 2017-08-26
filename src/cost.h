#ifndef COST_H
#define COST_H

#include <fstream>
#include <tuple>
#include "base.h"

namespace {
namespace ego_cost {

/**
 * Calculate cost of given state and cars tuple when our ego car runs a trajectory.
 * the costs are weighted according to a given CostWeights object.
 */
double CalculateCost(const std::tuple<ego::State, std::vector<ego::OtherCar>> &cf_state, 
                     const ego::Trajectory &trajectory,
                     const ego::CostWeights weights,
                     bool verbose=false) {
  double cost = 10.0;
  for (int i = 0; i < trajectory.x.size(); ++i) {

  }
  if (std::get<0>(cf_state) == ego::STATE_LCL) {
    cost = 0;
  }
  return cost;
}

}
}
#endif