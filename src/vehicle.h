#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>

using namespace std;

typedef struct Trajectory {

}

class Vehicle {
 public:
  Vehicle(int lane, double s, double v, double a);

  // Get state at given time.
  // state_in()

  double s;
  double v;
  double a;
  string state;
};

#endif