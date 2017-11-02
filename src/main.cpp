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

  initial_state - [s, v_t, a_t, d, v_n, a_n]
  final_state   - [s, v_t, a_t, d, v_n, a_n]
  T     - Total time in seconds

  OUTPUT
  An array of length 6. The first 3 for s and the next 3 for d
  */

  vector<double> s_start = {initial_state[0], initial_state[1], initial_state[2]};
  vector<double> s_end = {final_state[0], final_state[1], final_state[2]};
  vector<double> s_coeffs = JMT(s_start, s_end, T);

  vector<double> d_start = {initial_state[3], initial_state[4], initial_state[5]};
  vector<double> d_end = {final_state[3], final_state[4], final_state[5]};
  vector<double> d_coeffs = JMT(d_start, d_end, T);

  vector<double> result = {s_coeffs[0], s_coeffs[1], s_coeffs[2], s_coeffs[3], s_coeffs[4], s_coeffs[5],
                           d_coeffs[0], d_coeffs[1], d_coeffs[2], d_coeffs[3], d_coeffs[4], d_coeffs[5]};
  return result;
}

tuple<vector<double>, vector<double>> GenerateTrajectory(vector<double> coeffs,
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

  coeffs - Coefficients (alphas) calculated from CalcCoeffs() function.
  T      - Total duration in seconds.
  dt     - Duration of a single timeframe.
  num_wp - Number of waypoints to get. -1 for all waypoints.

  OUTPUT
  A tuple with list of s coordinates as the first element, and list of y coordinates as the
  second one.

  */

  vector<double> s_coeffs = {coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]};
  vector<double> d_coeffs = {coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]};
  tuple<vector<double>, vector<double>> results;
  vector<double> list_s;
  vector<double> list_d;
  int wps = min((int)(T/dt), num_wp);

  for (int i=1; i <= wps; ++i) {
    double t = i * dt;
    
    double s = s_coeffs[0]         + s_coeffs[1] * t       + s_coeffs[2] * t*t +
               s_coeffs[3] * t*t*t + s_coeffs[4] * t*t*t*t + s_coeffs[5] * t*t*t*t*t;
    // double vt = s_coeffs[1]             + 2 * s_coeffs[2] * t       + 3 * s_coeffs[3] * t*t +
    //             4 * s_coeffs[4] * t*t*t + 5 * s_coeffs[5] * t*t*t*t
    //             + adj_vt;

    // double at = 2 * s_coeffs[2] + 6 * s_coeffs[3] * t + 12 * s_coeffs[4] * t*t +
    //             20 * s_coeffs[5] * t*t*t
    //             + adj_at;

    double d = d_coeffs[0]         + d_coeffs[1] * t       + d_coeffs[2] * t*t +
               d_coeffs[3] * t*t*t + d_coeffs[4] * t*t*t*t + d_coeffs[5] * t*t*t*t*t;
    list_s.push_back(s);
    list_d.push_back(d);
  }
  get<0>(results) = list_s;
  get<1>(results) = list_d;
  return results;
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

void SetupInitialState(const vector<double> &tj_s,
                       const vector<double> &tj_d,
                       vector<double> *initial_state,
                       double dt=0.02) {
  /*
  Setup initial state from trajectories.
  */

  vector<double> state_s = StateFromTJ(tj_s, dt);
  vector<double> state_d = StateFromTJ(tj_d, dt);

  // END - Initialize Ref variables

  (*initial_state) = {state_s[0],
                      state_s[1],
                      state_s[2],
                      state_d[0],
                      state_d[1],
                      state_d[2]
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

void FindBestTrajectory(const vector<double> &initial_state,
                        int num_wp,
                        constraints t_const,
                        const json &sensor_fusion,

                        vector<double> *new_tj_s,
                        vector<double> *new_tj_d,
                        double dt,
                        const vector<double> previous_tj_s = {},
                        const vector<double> previous_tj_d = {}
                        ) {

  /**
   * INPUTS:
   * 
   * initial_state - State of the origin of waypoints.
   * num_wp - Number of waypoints of the resulting trajectory.
   * t_const - Constraints of trajectory.
   *           Contains maximum allowed speed and accelerations.
   * sensor_fusion - Gathered from sensors in the vehicle.
   * previous_tj_s - List of s from the previous trajectory.
   * previous_tj_d - List of d from the previous trajectory.
   * dt - Tiem of each iteration in seconds.
   *
   *
   * OUTPUTS:
   * new_tj_s - Trajectory of s direction.
   */
  double min_cost = 99999999.9;
  tuple<vector<double>, vector<double>> best_traj;

  cout << "num_wp: " << num_wp << endl;
  
  int target_lane = 1;
  double best_ds;
  double best_T;
  tuple<vector<double>, vector<double>> best_comb_traj;

  // Find best lane
  for (int lane = 0; lane <= 2; lane++) {
    // If there are other vehicles within a distance, try another lane.
    cost::LaneCost(initial_state, sensor_fusion, lane);
  }

  // START - Find best trajectory.
  // We do this by trying out various ds (distance of s) and
  // target_T (time to reach that distance), calculate the cost
  // function for each generated trajectory, and then pick the trajectory
  // with the smallest cost value.

  vector<double> act_initial_state = {
    0,
    initial_state[1],
    initial_state[2],
    0,
    initial_state[4],
    initial_state[5],
  };
  for (double ds = 5.0; ds <= 50.0; ds += 5.0) {
    for (double target_T = 1.5; target_T <= 20.0; target_T += 0.5) {
      // Try out various ds
      double target_s = initial_state[0] + ds;
      // double target_s = ds;
      double target_v = t_const.max_v;

      if (d2lane(initial_state[3]) != target_lane) {
        // Slower speed when changing lane
        // target_v = mph2mps(40.0);
      }
 
      vector<double> target_state = {
        target_s,
        target_v,
        0,
        lane2d(target_lane),
        0,
        0
      };

      // cout << "target state: " << target_state << endl;

      int N = target_T / dt;

      vector<double> coeffs = CalcCoeffs(initial_state, target_state, target_T);
      tuple<vector<double>, vector<double>> traj = GenerateTrajectory(coeffs, N*dt, dt, num_wp);
      double c = 0;

      // START - Create comb_traj

      // Combine a maximum of last three waypoints of previous trajectory with the new ones.
      // This step is important so we can calculate MovementCost
      // throughout the whole combined trajectory (i.e. jerk calculation requires a minimum
      // of 4 points).
      tuple<vector<double>, vector<double>> comb_traj;

      // This is the part that fills with previous trajectory
      vector<double> comb_tj_s;
      vector<double> comb_tj_d;
      int prevsize = (int)previous_tj_s.size();
      for (int i = min(3, prevsize); i > 0; --i) {
        double s = previous_tj_s[prevsize - i];
        comb_tj_s.push_back(s);
        double d = previous_tj_d[prevsize - i];
        comb_tj_d.push_back(d);
      }

      // This is the part that append with new trajectory
      for (int i=0; i < (int)get<0>(traj).size(); ++i) {
        // get<0>(traj)[i] += initial_state[0];
        // get<1>(traj)[i] += initial_state[3];

        comb_tj_s.push_back(get<0>(traj)[i]);
        comb_tj_d.push_back(get<1>(traj)[i]);
      }
      get<0>(comb_traj) = comb_tj_s;
      get<1>(comb_traj) = comb_tj_d;

      // END - Create comb_traj

      // cout << "total comb_tj_s: " << comb_tj_s.size() << endl;

      // cout << "l: " << target_lane << " ds: " << ds << " ";
      // c += cost::OrientationCost(traj);
      c += cost::MovementCost(comb_traj, target_v, t_const.max_at, t_const.max_jerk, dt);
      // c += cost::CollisionCost();
      // cout << "cost: " << c << " min_cost: " << min_cost << endl;

      if (c < min_cost) {
        best_traj = traj;
        best_comb_traj = comb_traj;
        min_cost = c;
        best_ds = ds;
        best_T = target_T;
        // cout << "Set best_traj to traj (size " << get<0>(traj).size() << ")" << endl;
      }
    }
  }
  // END - Find best trajectory

  cout << "initial state: " << initial_state << endl;
  cout << "target_v: " << t_const.max_v << endl;
  cout << "chosen lane: " << target_lane << 
          " | ds: " << best_ds << " | T: " << best_T <<
          " | cost: " << min_cost << endl;
  // Get only the first `num_wp` waypoints.

  int nwp = num_wp;
  if (nwp > get<0>(best_traj).size()) {
    nwp = get<0>(best_traj).size();
  }

  (*new_tj_s) = get<0>(best_traj);
  (*new_tj_d) = get<1>(best_traj);

  // cout << "set new_tj_s with tj_s with size " << (*new_tj_s).size() << endl;
  vector <double> comb_traj_s = get<0>(best_comb_traj);
  vector <double> comb_traj_d = get<1>(best_comb_traj);
  for (int i=0; i < comb_traj_s.size(); ++i) {
    cout << comb_traj_s[i] << ", " << comb_traj_d[i];

    if (i > 0) {
      cout << " v: " << velocity(comb_traj_s[i],
                                 comb_traj_s[i-1], dt);
    } 
    if (i > 1) {
      cout << " a: " << acceleration(comb_traj_s[i],
                                     comb_traj_s[i-1],
                                     comb_traj_s[i-2], dt);
    }
    if (i > 2) {
      cout << " j: " << jerk(comb_traj_s[i],
                             comb_traj_s[i-1],
                             comb_traj_s[i-2],
                             comb_traj_s[i-3], dt);
    }
    cout << endl;
  }
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

  cost::testIsWrongDirection();

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

  init(map_waypoints_s, map_waypoints_x, map_waypoints_y);

  TestFrenetDistance(map_waypoints_s, map_waypoints_x, map_waypoints_y);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // START - Include previous trajectory
            for (int i = 0; i < previous_path_x.size(); ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            // END - Include previous trajectory

            // START - Setup initial state
            double max_accel_t = 8.0;
            // double max_accel_n = 3.0;
            double max_jerk = 8.0;
            double max_speed = mph2mps(45.0);
            if (counter > 200) {
              max_speed = mph2mps(30.0);
            }

            constraints t_const = {};
            t_const.max_v = max_speed;
            t_const.max_at = max_accel_t;
            t_const.max_jerk = max_jerk;

            vector<double> initial_state = {};

            if ((int)prev_tj_s.size() <= 2) {
              initial_state = {car_s, car_speed, 0,
                               car_d, 0, 0};
              ref_yaw = car_yaw;
            }
            else {
              SetupInitialState(prev_tj_s, prev_tj_d,
                                &initial_state,
                                dt);
              ref_yaw = YawFromTJ(prev_tj_s, prev_tj_d,
                                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
            }

            assert(initial_state.size() > 0);

            // END - Setup initial state

            // START - Find best trajectory

            vector<double> new_tj_s;
            vector<double> new_tj_d;

            int num_remaining_wp = (num_wp - prev_size);

            if (num_remaining_wp > 2) {
              // To ensure we always have at least 3 prev_tj_s and prev_tj_d.

              FindBestTrajectory(initial_state, num_remaining_wp, t_const,
                                 sensor_fusion,
                                 &new_tj_s, &new_tj_d, dt,
                                 prev_tj_s, prev_tj_d);

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

              cout << endl;

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