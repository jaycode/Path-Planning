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
#include <limits>

using namespace std;
using namespace std::chrono;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// for convenience
using json = nlohmann::json;

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

// Remove N elements from a vector.
void RemoveN(int N, vector<double> *myvector) {
  std::vector<double>((*myvector).begin()+N,
                      (*myvector).end()).swap((*myvector));
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// getXY function with spline implementation
// We do not use the default Udacity functions for getXY as it created gaps between
// some portions of the map. See at the bottom for original getXY function.
// To use this function, `init()` function needs to be called first to initialize the splines.
//
// Credit to Guy Pavlov:
// https://github.com/gpavlov2016/CarND-Path-Planning-Project/blob/master/src/utils.cpp
//=============================================================

// x, y, and headings
tk::spline sx;
tk::spline sy;
tk::spline sh;

void init(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) 
{
  /*
  Initialize x, y, and heading splines for use in getXY and getFrenet functions.
  */
  vector<double> maps_h;
  double prev_heading = 0;
  for (int i=0; i<maps_s.size(); i++) 
  {
    int next_i = (i+1)%maps_x.size();
    double dy = maps_y[next_i]-maps_y[i];
    double dx = maps_x[next_i]-maps_x[i];
    double heading = atan2(dy,dx);
    if (fabs(heading-prev_heading) > pi()/2) {
      heading += 2*pi();
    }
    prev_heading = heading;
    maps_h.push_back(heading);
  }

  sx.set_points(maps_s, maps_x);
  sy.set_points(maps_s, maps_y);
  sh.set_points(maps_s, maps_h);
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  double heading = sh(s);

  double perp_heading = heading-pi()/2;


  double x = sx(s) + d*cos(perp_heading);
  double y = sy(s) + d*sin(perp_heading);
  return {x,y};

}
//=============================================================

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}


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

  for (int i=1; i <= T/dt; ++i) {
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

vector<double> StateFromTJ(const vector<double> &tj,
                           double dt) {
  /*
  Calculate position, speed, and acceleration from the trajectory's last three positions.
  */
  assert(tj.size() > 2);
  double s1 = tj[tj.size()-1];
  double s2 = tj[tj.size()-2];
  double s3 = tj[tj.size()-3];
  double v1 = (s1-s2) / dt;
  double v2 = (s2-s3) / dt;
  double a = (v1-v2) / dt;
  vector<double> state = {
    s1,
    v1,
    a
  };
  return state;
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
  cout << "dy: " << dy << " dx: " << dx << endl;
  double yaw = atan2(dy, dx);
  cout << "yaw from tj: " << yaw << endl;
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
                        double max_speed,
                        
                        vector<double> *new_tj_s,
                        vector<double> *new_tj_d) {

  double target_T = 4.5;
  for (int target_lane = 0; target_lane <= 2; target_lane++) {
    double target_s = initial_state[0] + 50;
    double target_v = max_speed;

    vector<double> target_state = {
      target_s,
      target_v,
      0,
      lane2d(target_lane),
      0,
      0
    };

    // cout << "initial state: " << initial_state << endl;
    // cout << "target state: " << target_state << endl;

    int N = target_T / dt;
    first = false;

    vector<double> coeffs = CalcCoeffs(initial_state, target_state, target_T);
    tuple<vector<double>, vector<double>> traj = GenerateTrajectory(coeffs, N*dt, dt);

    (*new_tj_s) = get<0>(traj);
    (*new_tj_d) = get<1>(traj);
  }
}

// Trajectory for s and d.
vector<double> tj_s;
vector<double> tj_d;
// Time of created trajectory
vector<double> tj_t;

bool first = true;

milliseconds ms = duration_cast<milliseconds>(
  system_clock::now().time_since_epoch()
);

int main() {
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
            cout << dt << endl;
            double num_wp = 120;
          
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
            int prev_size = previous_path_x.size();

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            double max_accel_t = mph2mps(9.5);
            double max_accel_n = mph2mps(9.5);
            double max_speed = mph2mps(45.0);

            vector<double> initial_state = {};

            if (tj_s.size() <= 2) {
              initial_state = {car_s, 0, 0,
                               car_d, 0, 0};
              ref_yaw = car_yaw;
            }
            else {
              SetupInitialState(tj_s, tj_d,
                                &initial_state,
                                dt);
              ref_yaw = YawFromTJ(tj_s, tj_d,
                                  map_waypoints_s, map_waypoints_x, map_waypoints_y);
            }

            assert(initial_state.size() > 0);

            if (first == true) {
              // Find best trajectory

              vector<double> new_tj_s;
              vector<double> new_tj_d;

              FindBestTrajectory(initial_state, max_speed, &new_tj_s, &new_tj_d)

              for (int i = 0; i < new_tj_s.size(); ++i) {
                tj_s.push_back(new_tj_s[i]);
                tj_d.push_back(new_tj_d[i]);
              }

            }
            else {
              N = num_wp - prev_size;
              double vt = max_speed;
              double s0 = initial_state[0];
              double vn = initial_state[4];
              double d0 = initial_state[3];
              
              for (int i = 1; i <= N; ++i) {
                // cout << "ref_yaw: " << ref_yaw << endl;
                double s1 = s0 + (vt * i * dt);
                double d1 = d0 + (vn * i * dt);

                tj_s.push_back(s1);
                tj_d.push_back(d1);
              }
            }

            // END - Find best_trajectory

            CreateWaypoints(tj_s, tj_d,
                            map_waypoints_s, map_waypoints_x, map_waypoints_y,
                            &next_x_vals, &next_y_vals);

            cout << endl;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

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