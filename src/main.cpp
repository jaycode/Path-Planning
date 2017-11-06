#include <chrono>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"
#include "cost.h"
#include <limits>

using namespace std;
using namespace std::chrono;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace helpers;


// for convenience
using json = nlohmann::json;

// For testing turning and slowing down.
int counter = 0;
int last_turn_counter = 0;
int prev_target_lane = 1;


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
          3*T*T, 4*T*T*T,5*T*T*T*T,
          6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
      result.push_back(C.data()[i]);
  }
  
    return result;
    
}

vector<double> CalcCoeffs(vector<double> initial_state, vector<double> final_state,
                          double T) {
  /*
  Calculate coefficients from given initial state, final state, and duration

  INPUTS

  initial_state - [(s or d), v, a]
  final_state   - [(s or d), v, a]
  T     - Total time in seconds

  OUTPUT
  An array of length 6.
  */

  vector<double> s_start = {initial_state[0], initial_state[1], initial_state[2]};
  vector<double> s_end = {final_state[0], final_state[1], final_state[2]};
  vector<double> s_coeffs = JMT(s_start, s_end, T);

  vector<double> result = {s_coeffs[0], s_coeffs[1], s_coeffs[2], s_coeffs[3], s_coeffs[4], s_coeffs[5]};
  return result;
}

vector<double> GenerateTrajectory(vector<double> coeffs,
                                  double T,
                                  double dt,
                                  int num_wp = -1) {
  /*
  From coefficients and final position,
  get list of s and d coordinates.

  The complete trajectory equation is visualized here:
  https://www.desmos.com/calculator/wrgdcjrz3b

  Target and initial states can be adjusted to find the perfect path
  (i.e. reached target in optimal time without breaking acceleration nor speed limits).

  INPUTS

  coeffs - 6-elements Coefficients (alphas) calculated from CalcCoeffs() function.
  T      - Total duration in seconds.
  dt     - Duration of a single timeframe.
  num_wp - Number of waypoints to get. -1 for all waypoints.

  OUTPUT
  A tuple with list of s coordinates as the first element, and list of y coordinates as the
  second one.

  */

  vector<double> tj;
  int wps = min((int)(T/dt), num_wp);

  for (int i=1; i <= wps; ++i) {
    double t = i * dt;
    
    double s = coeffs[0]         + coeffs[1] * t       + coeffs[2] * t*t +
               coeffs[3] * t*t*t + coeffs[4] * t*t*t*t + coeffs[5] * t*t*t*t*t;
    tj.push_back(s);
  }
  return tj;
}

void CreateWaypoints(vector<double> list_s, vector<double> list_d,
                     const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y,
                     vector<double> *next_x_vals,
                     vector<double> *next_y_vals) {

  double dt = 0.02;
  // Previous xy
  vector<double> xy0;
  for (int i = 0; i < list_s.size(); ++i) {
    vector<double> xy1 = getXY(list_s[i], list_d[i],
                              maps_s, maps_x, maps_y);
    double xf = xy1[0];
    double yf = xy1[1];

    // limiter
    if (i > 0) {
      double vt = (xy1[0] - xy0[0]) / dt;

    }
    (*next_x_vals).push_back(xf);
    (*next_y_vals).push_back(yf);
    xy0 = xy1;
  }
}

double YawFromTJ(const vector<double> &tj_s,
                 const vector<double> &tj_d,
                 const vector<double> &maps_s,
                 const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  /*
  Get car yaw from last position.
  */
  assert(tj_s.size() > 1);
  assert(tj_d.size() > 1);
  double s1 = tj_s[tj_s.size()-1];
  double s2 = tj_s[tj_s.size()-2];
  double d1 = tj_d[tj_d.size()-1];
  double d2 = tj_d[tj_d.size()-2];
  vector<double> xy1 = getXY(s1, d1, maps_s, maps_x, maps_y);
  vector<double> xy2 = getXY(s2, d2, maps_s, maps_x, maps_y);
  double dy = xy1[1] - xy2[1];
  double dx = xy1[0] - xy2[0];
  double yaw = atan2(dy, dx);
  return yaw;
}

void SetupFWPState(const vector<double> &tj_s,
                   const vector<double> &tj_d,
                   double dt,
                   vector<double> *fwp_state) {
  /*
  Setup final waypoint state from trajectories.
  Returns a 6-elements state.
  */

  vector<double> state_s = StateFromTJ(tj_s, dt);
  vector<double> state_d = StateFromTJ(tj_d, dt);

  // END - Initialize Ref variables

  (*fwp_state) = {state_s[0],
                  state_s[1],
                  state_s[2],
                  state_d[0],
                  state_d[1],
                  state_d[2]
                 };
}

void SetupFWPState(const vector<double> &tj,
                   double dt,
                   vector<double> *fwp_state) {
  /*
  Setup final waypoint state from trajectories.
  Returns a 3-elements state.
  */

  vector<double> state = StateFromTJ(tj, dt);

  // END - Initialize Ref variables

  (*fwp_state) = {state[0],
                  state[1],
                  state[2]
                 };
}

void TestFrenetDistance(const vector<double> maps_s,
                        const vector<double> maps_x,
                        const vector<double> maps_y) {
  vector<double> sd0 = {100, 0};
  vector<double> sd1 = {101, 0};
  vector<double> xy0 = getXY(sd0[0], sd0[1], maps_s, maps_x, maps_y);
  vector<double> xy1 = getXY(sd1[0], sd1[1], maps_s, maps_x, maps_y);
  double dist_sd = distance(sd0[0], sd0[1], sd1[0], sd1[1]);
  double dist_xy = distance(xy0[0], xy0[1], xy1[0], xy1[1]);
  cout << "dist sd: " << dist_sd << " dist_xy: " << dist_xy << endl;
}

double FindBestVelocity(const vector<double> &car_state,
                        const vector<double> &fwp_state,
                        const json &sensor_fusion,
                        int target_lane,
                        double max_v, bool log = false) {
  /**
   * If the target lane has a vehicle in front of us, follow that
   * vehicle's velocity, otherwise `max_v`.
   *
   * INPUTS:
   * car_state - State of the vehicle.
   * fwp_state - State of the final waypoint.
   * sensor_fusion - Gathered from sensors in the vehicle.
   *                 [ id, x, y, vx, vy, s, d]
   * target_lane - The lane the vehicle is heading towards.
   * max_v - Maximum allowed velocity.
   * log - Print log when true.
   *
   *
   * OUTPUT:
   * Best target velocity.
   */

  // double dd = fabs(d2lane(car_state[3]) - target_lane);
  double target_v = max_v;
  // if (dd >= 1) {
  //   // target_v *= 0.65;
  //   target_v = mph2mps(45.0) * 0.65;
  // }
  double closest_car_id = -1;
  double closest_car_dist = 999.0;
  double visibility_max = 30;
  double visibility_min = -3;
  // double visibility_max = 0;
  // double visibility_min = -8;

  if (d2lane(fwp_state[3]) == target_lane) {
    // Find a vehicle that is within `visibility_max` and `visibility_min` meters
    // ahead of the final waypoint (i.e. initial state).
    for (int i = 0; i < sensor_fusion.size(); ++i) {
      if (d2lane((double)sensor_fusion[i][6]) == target_lane) {
        double dist_s = ((double)sensor_fusion[i][5] - car_state[0]);
        // double dist_s = ((double)sensor_fusion[i][5] - fwp_state[0]);
        if (dist_s < visibility_max && dist_s > visibility_min && dist_s < closest_car_dist) {
          closest_car_dist = dist_s;
          closest_car_id = i;
        }
      }
    }
  }

  if (closest_car_id > -1) {
    // TODO: Use PID controller to ensure the vehicle does not crash into another.
    double vx = (double)sensor_fusion[closest_car_id][3];
    double vy = (double)sensor_fusion[closest_car_id][4];
    target_v = sqrt(vx*vx + vy*vy) - 0.2;
    if (log) {
      cout << "dist: " << closest_car_dist << " target_v: " << target_v << endl;
    }
  }

  if (log) {
    cout << "set target_v to " << target_v << endl;
  }

  return target_v;
}

void LoopTrajectories(const vector<double> &ds_params,
                      const vector<double> &dT_params,
                      const vector<double> &initial_state,
                      const vector<double> &previous_tj,
                      double target_s_or_d,
                      double target_v, double max_a, double max_jerk, int tj_wp, double dt,
                      double (*cost_function)(const vector<double>&, double, double, double, double, double),
                      vector<double> *best_tj, vector<double> *best_comb_tj,
                      double *min_cost, double *best_ds, double *best_T,
                      vector<double> *best_fwp_state) {
  /**
   * We do this by trying out various ds (distance of s) and
   * target_T (time to reach that distance), calculate the cost
   * function for each generated trajectory, and then pick the trajectory
   * with the smallest cost value.
   * 
   * INPUTS:
   * ds_params: [start, end, increment]
   * dT_params: [start, end, increment]
   * initial_state: 3-elements state
   */

  for (double ds = ds_params[0]; ds <= ds_params[1]; ds += ds_params[2]) {
    for (double target_T = dT_params[0]; target_T <= dT_params[1]; target_T += dT_params[2]) {
      // Try out various ds
      double target_s = target_s_or_d + ds;
      vector<double> target_state = {
        target_s,
        target_v,
        0
      };

      // cout << "target state (s): " << target_state_s << endl;

      int N = target_T / dt;

      vector<double> coeffs = CalcCoeffs(initial_state, target_state, target_T);
      vector<double> tj = GenerateTrajectory(coeffs, N*dt, dt, tj_wp);
      double c = 0;

      // START - Create comb_traj

      // Combine a maximum of last three waypoints of previous trajectory with the new ones.
      // This step is important so we can calculate MovementCost
      // throughout the whole combined trajectory (i.e. jerk calculation requires a minimum
      // of 4 points).

      // This is the part that fills with previous trajectory
      vector<double> comb_tj;
      int prevsize = (int)previous_tj.size();
      for (int i = min(3, prevsize); i > 0; --i) {
        double s = previous_tj[prevsize - i];
        comb_tj.push_back(s);
      }

      // This is the part that append with new trajectory
      for (int i=0; i < (int)tj.size(); ++i) {
        comb_tj.push_back(tj[i]);
      }

      // END - Create comb_traj

      // cout << "total comb_tj_s: " << comb_tj_s.size() << endl;

      vector<double> fwp_state;
      SetupFWPState(comb_tj, dt, &fwp_state);

      c += (*cost_function)(comb_tj, target_s_or_d, target_v, max_a, max_jerk, dt);

      if (c < (*min_cost)) {
        (*best_tj) = tj;
        (*best_comb_tj) = comb_tj;
        (*min_cost) = c;
        (*best_ds) = ds;
        (*best_T) = target_T;
        (*best_fwp_state) = fwp_state;
      }
    }
  }
  // END - Find best trajectory
}

void FindBestTrajectory(const vector<double> &initial_state,
                        int tj_wp,
                        int returned_wp,
                        constraints t_const,
                        double dt,
                        vector<double> *new_tj_s,
                        vector<double> *new_tj_d,
                        const vector<double> previous_tj_s = {},
                        const vector<double> previous_tj_d = {},
                        int log = 0
                        ) {

  /**
   * INPUTS:
   * initial_state - 6-element state of the origin of new trajectory.
   * tj_wp - Number of waypoints generated for a trajectory. These waypoints will
   *         be used in cost calculation.
   * returned_wp - Number of waypoints to be returned.
   * t_const - Constraints of trajectory. See `constraints` class.
   * dt - Tiem of each iteration in seconds.
   * previous_tj_s - List of s from the previous trajectory.
   * previous_tj_d - List of d from the previous trajectory.
   * log - Log verbosity. 0: no log, 1: only chosen lane, 2: +trajectory
   *
   *
   * OUTPUTS:
   * new_tj_s - Trajectory of s direction.
   * new_tj_d - Trajectory of d direction.
   */
  double min_cost_s = 99999999.9;
  double min_cost_d = 99999999.9;

  if (log >= 1) {
    cout << "tj_wp: " << tj_wp << endl;
  }
  
  double target_v = t_const.target_v;
  int target_lane = t_const.target_lane;
  double best_ds;
  double best_dd;
  double best_T_s;
  double best_T_d;
  vector<double> best_tj_s;
  vector<double> best_tj_d;
  vector<double> best_comb_tj_s;
  vector<double> best_comb_tj_d;
  vector<double> best_fwp_state_s;
  vector<double> best_fwp_state_d;

  vector<double> ds = {0.0, 60.0, 6.0};
  vector<double> dT_s = {1.5, 5.0, 0.25};
  vector<double> initial_state_s = {
    initial_state[0],
    initial_state[1],
    initial_state[2]
  };

  LoopTrajectories(ds, dT_s, initial_state_s, previous_tj_s,
                   initial_state[0], target_v,
                   t_const.max_at, t_const.max_jerk, tj_wp, dt,
                   &cost::TangentialMovementCost,
                   &best_tj_s, &best_comb_tj_s,
                   &min_cost_s, &best_ds, &best_T_s,
                   &best_fwp_state_s);

  vector<double> dd = {-1.0, 1.0, 0.25};
  vector<double> dT_d = {best_T_s, best_T_s, 0.1};
  vector<double> initial_state_d = {
    initial_state[3],
    initial_state[4],
    initial_state[5]
  };

  LoopTrajectories(dd, dT_d, initial_state_d, previous_tj_d,
                   lane2d(target_lane), 0,
                   t_const.max_an, t_const.max_jerk, 120, dt,
                   &cost::LateralMovementCost,
                   &best_tj_d, &best_comb_tj_d,
                   &min_cost_d, &best_dd, &best_T_d,
                   &best_fwp_state_d);

  if (log >= 1) {
    cout << "initial state: " << initial_state << endl;
    cout << "fwp state: " << best_fwp_state_s << ", " << best_fwp_state_d << endl;
    cout << "target_v: " << t_const.target_v << endl;
    cout << "target_lane: " << target_lane << " (d = " << lane2d(target_lane) << ")" << endl;
    cout << "ds: " << best_ds << " | Ts: " << best_T_s <<
            " | cost s: " << min_cost_s <<
            " | dd: " << best_dd << " | Td: " << best_T_d <<
            " | cost d: " << min_cost_d << endl;

  }

  assert(min_cost_s < 900);
  assert(min_cost_d < 900);

  // Get only the first `returned_wp` waypoints.
  if (returned_wp > best_tj_s.size()) {
    returned_wp = best_tj_s.size();
  }

  vector<double> fin_tj_s;
  vector<double> fin_tj_d;
  for (int i = 0; i < returned_wp; ++i) {
    fin_tj_s.push_back(best_tj_s[i]);
    fin_tj_d.push_back(best_tj_d[i]);
  }

  (*new_tj_s) = fin_tj_s;
  (*new_tj_d) = fin_tj_d;

  if (log >= 2) {
    PrintTrajectory(best_comb_tj_s, best_comb_tj_d, dt);
  }

}

int FindBestLane(const vector<double> &car_state,
                 const vector<double> &fwp_state,
                 int num_wp,
                 int num_remaining_wp,
                 constraints t_const,
                 const json &sensor_fusion,
                 double max_speed,
                 const vector<double> &prev_tj_s,
                 const vector<double> &prev_tj_d,
                 double dt) {
  /**
   * INPUTS:
   * car_state - State of the vehicle.
   * fwp_state - State of the final waypoint.
   * t_const - See constraints class.
   * sensor_fusion - Gathered from sensors in the vehicle.
   *
   *
   * OUTPUT:
   * Best target lane.
   */
  double min_cost = 999999.0;
  double target_lane = d2lane(fwp_state[3]);
  cout << "fwp_lane: " << d2lane(fwp_state[3]) << " (d = " << fwp_state[3] <<
          "), car_lane: " << d2lane(car_state[3]) << " (d = " << car_state[3] << ")" << endl;
  for (int lane = 0; lane <= 2; lane++) {
    // If there are other vehicles within a distance, try another lane.

    double c = cost::LaneChangeCost(fwp_state, lane);

    // Generate a trajectory from the vehicle's position (not final waypoint)
    // to the chosen lane, and see if it crashes into any vehicle.
    t_const.target_lane = lane;

    cout << "now checking lane " << t_const.target_lane << endl;
    double new_target_v = FindBestVelocity(car_state, fwp_state, sensor_fusion,
                                        t_const.target_lane, max_speed);

    c += max_speed - new_target_v;
    // t_const.target_v = new_target_v;

    vector<double> new_tj_s;
    vector<double> new_tj_d;
    FindBestTrajectory(fwp_state, num_wp, num_wp, t_const, dt,
                       &new_tj_s, &new_tj_d);

    // Do not check too often to avoid choosing a non-optimal lane.
    if (counter % 10 == 0) {
      // c += cost::CollisionCost(new_tj_s, new_tj_d, sensor_fusion, dt);
      c += cost::SimpleCollisionCost(car_state[0], lane, sensor_fusion);
    }

    cout << "lane " << lane << ", cost: " << c << endl;

    // if (c > 100) {
    //   PrintTrajectory(new_tj_s, new_tj_d, dt);
    // }

    if (c < min_cost) {
      min_cost = c;
      target_lane = lane;
    }
  }
  return target_lane;
}

// Trajectory for s and d.
vector<double> prev_tj_s;
vector<double> prev_tj_d;
milliseconds ms = duration_cast<milliseconds>(
  system_clock::now().time_since_epoch()
);

// string traj_log_file = "../log/traj_log.csv";
// ofstream traj_log;

int main() {

  // traj_log.open(traj_log_file);
  // traj_log << "S, Vt, At, T\n";
  // traj_log.close();

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // string map_file_ = "../data/highway_map_bosch1 - final.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  counter = 0;
  last_turn_counter = 0;

  bool first = true;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  AddWaypointsToLoop(max_s,
                     &map_waypoints_x,
                     &map_waypoints_y,
                     &map_waypoints_s,
                     &map_waypoints_dx,
                     &map_waypoints_dy);

  init(map_waypoints_s, map_waypoints_x, map_waypoints_y);

  TestFrenetDistance(map_waypoints_s, map_waypoints_x, map_waypoints_y);
  cost::TestCollisionCost();
  cost::TestSimpleCollisionCost();

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&first](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

            milliseconds new_ms = duration_cast<milliseconds>(
              system_clock::now().time_since_epoch()
            );
            // double dt = (double)(new_ms - ms).count() / 1000;
            double dt = 0.02;
            ms = new_ms;
            // cout << dt << endl;
            int num_wp = 120;

            // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            double car_speed = j[1]["speed"];

            // In initial step, ref_yaw is equal to car_yaw,
            // but on subsequent steps, it is the car's yaw at
            // the last waypoint.
            double ref_yaw;

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            int prev_size = (int)previous_path_x.size();

            int num_remaining_wp = (num_wp - prev_size);

            // Previous path's end s and d values 
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            vector<double> next_x_vals;
            vector<double> next_y_vals;
            json msgJson;

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            // [ id, x, y, vx, vy, s, d]
            auto sensor_fusion = j[1]["sensor_fusion"];

            // START - Include previous trajectory
            for (int i = 0; i < previous_path_x.size(); ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            // END - Include previous trajectory


            if (first) {
              // Set initial position. Useful to test end-of-map behavior.
              first = false;
              // next_x_vals.push_back(car_x);
              // next_y_vals.push_back(car_y);

              // vector<double> xy = getXY(6930.0, 6.0, map_waypoints_s,
              //                           map_waypoints_x, map_waypoints_y);
              // next_x_vals.push_back(xy[0]);
              // next_y_vals.push_back(xy[1]);
              // xy = getXY(6931.0, 6.0, map_waypoints_s,
              //                           map_waypoints_x, map_waypoints_y);
              // next_x_vals.push_back(xy[0]);
              // next_y_vals.push_back(xy[1]);
              // cout << car_x << ", " << car_y << endl;
              // cout << next_x_vals << endl << next_y_vals << endl;
            }
            else {
              // START - Setup initial state
              double max_accel_t = 8.0;
              double max_accel_n = 8.0;
              double max_jerk = 8.0;
              double max_speed = mph2mps(47.0);

              // State of the final waypoint
              // [s, vt, at, d, vn, an]
              vector<double> fwp_state = {};

              // State of the car
              // [s, vt, at, d, vn, an]
              vector<double> car_state = {car_s, car_speed, 0, car_d, 0, 0};

              if ((int)prev_tj_s.size() <= 2) {
                fwp_state = car_state;
                ref_yaw = car_yaw;
              }
              else {
                SetupFWPState(prev_tj_s, prev_tj_d,
                              dt, &fwp_state);
                ref_yaw = YawFromTJ(prev_tj_s, prev_tj_d,
                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
              }

              assert(fwp_state.size() > 0);

              if (num_remaining_wp > 3) {
                // To ensure we always have at least 3 prev_tj_s and prev_tj_d.

                constraints t_const = {};
                t_const.max_at = max_accel_t;
                t_const.max_an = max_accel_n;
                t_const.max_jerk = max_jerk;
                t_const.target_v = max_speed;
                double target_lane = prev_target_lane;

                // Prevents slipping from turning too often in a short period of time.
                // This code only allows the car to change target lane after several steps.
                if (counter - last_turn_counter > 40) {
                  target_lane = FindBestLane(car_state, fwp_state, num_wp, num_remaining_wp,
                                             t_const, sensor_fusion, max_speed,
                                             prev_tj_s, prev_tj_d, dt);
                }

                if (target_lane != prev_target_lane) {
                  last_turn_counter = counter;
                  prev_target_lane = target_lane;
                }

                // if (counter < 30) {
                //   target_lane = 1;
                // }
                // else {
                //   target_lane = 2;
                // }

                t_const.target_v = FindBestVelocity(car_state, fwp_state, sensor_fusion,
                                                    target_lane, max_speed, false);
                t_const.target_lane = target_lane;

                // END - Setup initial state

                // START - Find best trajectory

                vector<double> new_tj_s;
                vector<double> new_tj_d;

                FindBestTrajectory(fwp_state, num_remaining_wp, num_remaining_wp, t_const, dt,
                                   &new_tj_s, &new_tj_d,
                                   prev_tj_s, prev_tj_d, 0);

                // cout << "Best trajectory:" << endl;

                // Erase previous trajectory
                prev_tj_s.clear();
                prev_tj_d.clear();

                for (int i = 0; i < new_tj_s.size(); ++i) {
                  prev_tj_s.push_back(new_tj_s[i]);
                  prev_tj_d.push_back(new_tj_d[i]);
                  // cout << new_tj_s[i] << ", " << new_tj_d[i] << endl;
                }
                
                for (int i = 0; i < new_tj_s.size(); ++i) {
                  // cout << new_tj_s[i] << ", " << new_tj_d[i];
                  if (i > 0) {
                    double dist = distance(new_tj_s[i], new_tj_d[i],
                                           new_tj_s[i-1], new_tj_d[i-1]);
                    // cout << " dist: " << dist << endl;
                  }
                }
                
                // END - Find best_trajectory

                CreateWaypoints(new_tj_s, new_tj_d,
                                map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                &next_x_vals, &next_y_vals);

                // Prints out new tjs and xy distances.
                //
                // vector<double> prev_xy = {};
                // for (int i = 0; i < new_tj_s.size(); ++i) {
                //   double s = new_tj_s[i];
                //   double d = new_tj_d[i];
                //   vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                //   if (prev_xy.size() > 0) {
                //     cout << "sd: " << s << ", " << d << " xy: " << xy[0] << ", " << xy[1] << 
                //             " dist: " << distance(xy[0], xy[1], prev_xy[0], prev_xy[1]) << endl;
                //   }
                //   prev_xy = xy;
                // }

                cout << endl;

              }
            }

            // cout << "size of next trajectory: " << next_x_vals.size() << endl;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            ++counter;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


// Deprecated: default getXY function from Udacity
// ======================================================================================

// // Transform from Frenet s,d coordinates to Cartesian x,y
// vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
// {
//   int prev_wp = -1;

//   while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
//   {
//     prev_wp++;
//   }

//   int wp2 = (prev_wp+1)%maps_x.size();

//   double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
//   // the x,y,s along the segment
//   double seg_s = (s-maps_s[prev_wp]);

//   double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
//   double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

//   double perp_heading = heading-pi()/2;

//   double x = seg_x + d*cos(perp_heading);
//   double y = seg_y + d*sin(perp_heading);

//   return {x,y};

// }
// ======================================================================================