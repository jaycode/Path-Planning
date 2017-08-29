#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <vector>
#include "json.hpp"
#include "base.h"

namespace ego {

class Vehicle {
 public:
  Vehicle() {};
  virtual ~Vehicle() {};

  ego::World world;
  ego::Config config;
  ego::Position position;
  // Acceleration
  double a;
  ego::State state;

 protected:
  ego::State getNextState(nlohmann::json predictions);
};

class OtherCar : public Vehicle {
 public:
  OtherCar() {};
  OtherCar(World world, Position position, OtherConfig config);
  virtual ~OtherCar();

  OtherConfig config;
};

class EgoCar : public Vehicle {
 public:
  EgoCar() {};
  EgoCar(ego::World world, ego::Position position, ego::EgoConfig config);
  virtual ~EgoCar();

  /**
   * Plan the best state and trajectory of the ego car considering other cars.
   */
  ego::Trajectory PlanTrajectory(const std::vector<ego::OtherCar> &other_cars,
                                 const ego::CostWeights &weights);

  ego::Snapshot getSnapshot();
  void InitFromSnapshot(const ego::Snapshot &snap);

  ego::EgoConfig config;

 protected:
  /**
   * Get trajectory (in world coordinate) from the current state.
   * trajectory can directly be used to draw waypoints.
   */
  void CreateTrajectories(ego::State state,
                          const std::vector<ego::OtherCar> &other_cars,
                          ego::Snapshot *snap,
                          ego::Trajectory *waypoints,
                          ego::Trajectory *plan);

  ego::State ChooseBestState(const std::vector<ego::OtherCar> &other_cars,
                             const ego::CostWeights &weights,
                             ego::Trajectory *waypoints);

  /**
   * Checks if this car is behind another car.
   */
  bool is_behind(const OtherCar &car);

  /**
   * When keeping lane, make sure not to collide into the car in the same lane.
   */
  void RealizeKeepLane(const std::vector<ego::OtherCar> &other_cars,
                       const ego::Position &ref, ego::Snapshot *snap);

  void RealizeFollowCar(const std::vector<ego::OtherCar> &other_cars,
                        const ego::Position &ref, ego::Snapshot *snap);

  void RealizeLaneChange(const std::vector<ego::OtherCar> &other_cars,
                         int num_lanes, const ego::Position &ref, ego::Snapshot *snap);

  std::vector<ego::OtherCar> FindClosestCar(
    const std::vector<OtherCar> &other_cars,
    const ego::Position &ref, Snapshot *snap,
    const std::vector<int> &lanes,
    double minimum_distance /*= 5.0*/);

};

}

#endif