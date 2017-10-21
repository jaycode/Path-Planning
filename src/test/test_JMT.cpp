#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include "../Eigen-3.3/Eigen/Dense"
#include "../json.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// To printout vector quickly
template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Mile per hour to meter per second.
double mph2mps(double x) { return x * 0.447; }

// Reversed.
double mps2mph(double x) { return x / 0.447; }

// Gets d (in meters from center of road) of a lane.
double lane2d(int lane) { return 2.0+4.0*lane; }

// Converts d to lane number
int d2lane(double d) { return (int)(((d-2.0)/4.0) + 0.5); }

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT 
  an array of length 6, each value corresponding to a coefficent in the polynomial 
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */
    
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T, 5*T*T*T*T,
       6*T,   12*T*T,  20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  cout << "Ai:" << endl;
  cout << Ai(0,0) << " " << Ai(0,1) << " " << Ai(0,2) << endl;
  cout << Ai(1,0) << " " << Ai(1,1) << " " << Ai(1,2) << endl;
  cout << Ai(2,0) << " " << Ai(2,1) << " " << Ai(2,2) << endl << endl;

  cout << "B:" << endl;
  cout << B(0,0) << endl;
  cout << B(1,0) << endl;
  cout << B(2,0) << endl << endl;
  
  MatrixXd C = Ai*B;
  // alpha 4, 5, 6 = inverse of coefficients * (final state - C of state)
  
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }
  
  return result;
    
}


vector<double> CalcCoeffs(vector<double> initial_state, vector<double> final_state, double T) {
  /*
  Calculate coefficients from given initial state, final state, and duration

  INPUTS

  initial_state - [s, v_t, a_t, d, v_n, a_n]
  final_state   - [s, v_t, a_t, d, v_n, a_n]
  T             - Duration in seconds

  OUTPUT
  An array of length 6. The first 3 for s and the next 3 for d
  */

  vector<double> s_start = {initial_state[0], initial_state[1], initial_state[2]};
  vector<double> s_end = {final_state[0], final_state[1], final_state[2]};
  cout << "s_start: " << s_start << endl;
  cout << "s_end: " << s_end << endl;
  vector<double> s_coeffs = JMT(s_start, s_end, T);
  cout << "s_coeffs: " << s_coeffs << endl;

  vector<double> d_start = {initial_state[3], initial_state[4], initial_state[5]};
  vector<double> d_end = {final_state[3], final_state[4], final_state[5]};
  vector<double> d_coeffs = JMT(d_start, d_end, T);
  cout << "d_coeffs: " << d_coeffs << endl;

  vector<double> result = {s_coeffs[0], s_coeffs[1], s_coeffs[2], s_coeffs[3], s_coeffs[4], s_coeffs[5],
                           d_coeffs[0], d_coeffs[1], d_coeffs[2], d_coeffs[3], d_coeffs[4], d_coeffs[5]};
  return result;
}


tuple<vector<double>, vector<double>> GenerateTrajectory1(vector<double> coeffs,
                                                          double T,
                                                          double dt=0.02) {
  /*
  From coefficients and final position,
  get list of s and d coordinates.

  The complete trajectory equation is visualized here:
  https://www.desmos.com/calculator/wrgdcjrz3b

  Target and initial states can be adjusted to find the perfect path
  (i.e. reached target in optimal time without breaking acceleration nor speed limits).

  INPUTS

  coeffs - Coefficients (alphas) calculated from CalcCoeffs() function.
  T      - Total duration in seconds.
  dt     - Duration of a single timeframe.

  OUTPUT
  A tuple with list of s coordinates as the first element, and list of y coordinates as the
  second one.

  */

  vector<double> s_coeffs = {coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]};
  vector<double> d_coeffs = {coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]};
  tuple<vector<double>, vector<double>> results;
  vector<double> list_s;
  vector<double> list_d;

  for (int i=0; i < T/dt; ++i) {
    double t = i * dt;
    double s = s_coeffs[0]         + s_coeffs[1] * t       + s_coeffs[2] * t*t +
               s_coeffs[3] * t*t*t + s_coeffs[4] * t*t*t*t + s_coeffs[5] * t*t*t*t*t;

    double vt = s_coeffs[1]             + 2 * s_coeffs[2] * t       + 3 * s_coeffs[3] * t*t +
                4 * s_coeffs[4] * t*t*t + 5 * s_coeffs[5] * t*t*t*t;

    double at = 2 * s_coeffs[2] + 6 * s_coeffs[3] * t + 12 * s_coeffs[4] * t*t +
                20 * s_coeffs[5] * t*t*t;

    double d = d_coeffs[0]         + d_coeffs[1] * t       + d_coeffs[2] * t*t +
               d_coeffs[3] * t*t*t + d_coeffs[4] * t*t*t*t + d_coeffs[5] * t*t*t*t*t;
    list_s.push_back(s);
    list_d.push_back(d);
  }
  get<0>(results) = list_s;
  get<1>(results) = list_d;
  return results;
}


// INCORRECT, DO NOT USE
tuple<vector<double>, vector<double>> GenerateTrajectory(vector<double> coeffs,
                                                         vector<double> initial_state,
                                                         vector<double> target_state,
                                                         double T, double dt=0.02) {
  /*
  From coefficients and final position,
  get list of s and d coordinates.

  INPUTS

  coeffs - Coefficients (alphas) calculated from CalcCoeffs() function.
  T      - Total duration in seconds.
  dt     - Duration of a single timeframe.
  */

  vector<double> s_coeffs = {coeffs[3], coeffs[4], coeffs[5]};
  vector<double> d_coeffs = {coeffs[9], coeffs[10], coeffs[11]};
  tuple<vector<double>, vector<double>> results;
  vector<double> list_s;
  vector<double> list_d;

  // Magnitudes

  MatrixXd s_M = MatrixXd(3, 1); // Magnitude
  s_M << target_state[0],
         5*target_state[0],
         10*target_state[0];

  MatrixXd d_M = MatrixXd(3, 1); // Magnitude
  d_M << target_state[3],
         5*target_state[3],
         10*target_state[3];

  // Alphas
  MatrixXd s_alphas = MatrixXd(3, 1);
  s_alphas << s_coeffs[0],
              s_coeffs[1],
              s_coeffs[2];

  MatrixXd d_alphas = MatrixXd(3, 1);
  d_alphas << d_coeffs[0],
              d_coeffs[1],
              d_coeffs[2];

  // Initial state
  MatrixXd s_C = MatrixXd(3, 1);
  s_C << initial_state[0],
         initial_state[1],
         initial_state[2];

  MatrixXd d_C = MatrixXd(3, 1);
  d_C << initial_state[3],
         initial_state[4],
         initial_state[5];

  for (int i=0; i < T/dt; ++i) {
    double t = (i * dt);

    // s
    MatrixXd s_A = MatrixXd(3, 3);
    s_A << (t/T)*(t/T)*(t/T),     (t/T)*(t/T)*(t/T)*(t/T), (t/T)*(t/T)*(t/T)*(t/T)*(t/T),
           3*((t*t)/(T*T*T)),       4*((t*t*t)/(T*T*T*T)),     5*((t*t*t*t)/(T*T*T*T*T)),
             6*((t)/(T*T*T)),        12*((t*t)/(T*T*T*T)),      20*((t*t*t)/(T*T*T*T*T));

    MatrixXd s_B = (s_A * s_alphas);

    s_B = s_B.cwiseProduct(s_M) + s_C;

    // cout << "size of s_B: " << s_B.rows() << " x " << s_B.cols() << endl;
    list_s.push_back(s_B(0, 0));

    // d
    MatrixXd d_A = MatrixXd(3, 3);
    d_A << (t/T)*(t/T)*(t/T),     (t/T)*(t/T)*(t/T)*(t/T), (t/T)*(t/T)*(t/T)*(t/T)*(t/T),
           3*((t*t)/(T*T*T)),       4*((t*t*t)/(T*T*T*T)),     5*((t*t*t*t)/(T*T*T*T*T)),
             6*((t)/(T*T*T)),        12*((t*t)/(T*T*T*T)),      20*((t*t*t)/(T*T*T*T*T));

    MatrixXd d_B = (d_A * d_alphas);

    d_B = d_B.cwiseProduct(d_M) + d_C;

    list_d.push_back(d_B(0, 0));

  }

  get<0>(results) = list_s;
  get<1>(results) = list_d;
  return results;
}

int main() {
  // Configurations
  double max_accel_t = mph2mps(9.5);
  double max_accel_n = mph2mps(9.5);
  double max_speed = mph2mps(49.5);

  // Inputs
  double car_s = 0.0;
  double car_speed = 0.0;
  double car_accel = 0.0;
  double car_d = lane2d(1);
  cout << "initial d: " << car_d << " target d: " << lane2d(2) << endl;
  double car_yaw = 0.0;

  double car_accel_t = 0.0;
  double car_accel_n = 0.0;
  double car_speed_t = 0.0;
  double car_speed_n = 0.0;

  double T = 3.6;
  double dt = 0.02;

  vector<double> initial_state = {
    car_s,
    car_speed_t,
    car_accel_t,
    car_d,
    car_speed_n,
    car_accel_n
  };

  vector<double> target_state = {
    car_s + 40,
    mph2mps(49.5),
    0,
    lane2d(2),
    0,
    0
  };

  vector<double> coeffs = CalcCoeffs(initial_state, target_state, T);
  cout << coeffs << endl;
  // tuple<vector<double>, vector<double>> traj = GenerateTrajectory(coeffs, 
  //                                                                 initial_state,
  //                                                                 target_state, T, dt);
  tuple<vector<double>, vector<double>> traj = GenerateTrajectory1(coeffs, T, dt);
  vector<double> list_s = get<0>(traj);
  vector<double> list_d = get<1>(traj);

  for (int i = 0; i < list_s.size(); ++i) {
    cout << list_s[i] << ", " << list_d[i] << endl;
  }
  return 0;
}