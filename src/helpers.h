#ifndef EGO_HELPERS_H
#define EGO_HELPERS_H

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>
#include "json.hpp"
#include "spline.h"
#include "base.h"


// Reason for unnamed namespace:
// https://google.github.io/styleguide/cppguide.html#Unnamed_Namespaces_and_Static_Variables
namespace {

namespace ego_help {

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Mile per hour to meter per second.
double mph2mps(double x) { return x * 1609.34 / 3600.0; }

// Gets d (in meters from center of road) of a lane.
double lane2d(int lane) { return 2.0+4.0*lane; }

// Converts d to lane number
int d2lane(double d) { return (int)(((d-2.0)/4.0) + 0.5); }

// Finds standard deviation. Useful for debuggging trajectory positions.
double stdev(vector<double> v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / v.size());
  return stdev;
}

double mean(vector<double> v) {
  double mean = accumulate( v.begin(), v.end(), 0.0)/v.size(); 
  return mean;
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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
/**
 * WARNING!!! The conversion is inaccurate, do not use for critical functions.
 * DO NOT USE especially for global distance calculation e.g. to find out trajectory
 * distance.
 */
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

template <typename T>
vector<size_t> SortIndexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

/**
 * Sort by x and remove similar points.
 */
void PruneWaypoint(vector<double> *wp_x, vector<double> *wp_y) {
  vector<double> wp_x_sorted;
  vector<double> wp_y_sorted;

  for (auto i: SortIndexes((*wp_x))) {
    wp_x_sorted.push_back((*wp_x)[i]);
    wp_y_sorted.push_back((*wp_y)[i]);
  }
  (*wp_x) = wp_x_sorted;
  (*wp_y) = wp_y_sorted;
}

void PruneMaps(vector<double> *maps_s, vector<double> *maps_x,
               vector<double> *maps_y, vector<double> *maps_h) {
  vector<double> maps_s_sorted;
  vector<double> maps_x_sorted;
  vector<double> maps_y_sorted;
  vector<double> maps_h_sorted;

  double prev = 0.0;
  int prev_i = -1;
  for (auto i: SortIndexes((*maps_s))) {
    if (prev_i > -1 && (*maps_s)[i] != (*maps_s)[prev_i]) {
      cout << (*maps_s)[i] << " vs " << (*maps_s)[prev_i] << endl;
      maps_s_sorted.push_back((*maps_s)[i]);
      maps_x_sorted.push_back((*maps_x)[i]);
      maps_y_sorted.push_back((*maps_y)[i]);
      maps_h_sorted.push_back((*maps_h)[i]);
    }
    prev_i = i;
  }
  (*maps_s) = maps_s_sorted;
  (*maps_x) = maps_x_sorted;
  (*maps_y) = maps_y_sorted;
  (*maps_h) = maps_h_sorted;
}


tk::spline sx;
tk::spline sy;
tk::spline sh;


void init(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) 
{
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

  PruneMaps(&maps_s, &maps_x, &maps_y, &maps_h);
  for (int i = 0; i < maps_s.size(); ++i) {
    cout << maps_s[i] << endl;
  }

  sx.set_points(maps_s, maps_x);    // currently it is required that X is already sorted
  sy.set_points(maps_s, maps_y);    // currently it is required that X is already sorted
  sh.set_points(maps_s, maps_h);    // currently it is required that X is already sorted
}

vector<double> getXY2(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  double heading = sh(s);

  double perp_heading = heading-pi()/2;

  double x = sx(s) + d*cos(perp_heading);
  double y = sy(s) + d*sin(perp_heading);

  return {x,y};
}

vector<double> getFrenet2(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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

void TestFrenetConversion(ego::World world) {
  vector<double> test_xy = getXY(28.7929058074951, 0.01159054, *world.map_waypoints_s,
                                 *world.map_waypoints_x, *world.map_waypoints_y);

  vector<double> test_xy2 = getXY2(28.7929058074951, 0.01159054, *world.map_waypoints_s,
                                   *world.map_waypoints_x, *world.map_waypoints_y);
  cout << "Test XY" << endl;
  cout << test_xy[0] << ", " << test_xy[1] << endl;
  cout << test_xy2[0] << ", " << test_xy2[1] << endl;

  vector<double> test_sd = getFrenet(129.1392, 500.8578, 0.0,
                                 *world.map_waypoints_x, *world.map_waypoints_y);
  vector<double> test_sd2 = getFrenet2(129.1392, 500.8578, 0.0,
                                 *world.map_waypoints_x, *world.map_waypoints_y);
  cout << "Test SD" << endl;
  cout << test_sd[0] << ", " << test_sd[1] << endl;
  cout << test_sd2[0] << ", " << test_sd2[1] << endl;
}

void TestFrenetConversion1(ego::World world) {
  double dist = 100.0;
  vector<double> test_xy = getXY(28.7929058074951, 0.01159054, *world.map_waypoints_s,
                                 *world.map_waypoints_x, *world.map_waypoints_y);

  vector<double> test_xy2 = getXY((28.7929058074951 + dist), (0.01159054 + 0), *world.map_waypoints_s,
                                   *world.map_waypoints_x, *world.map_waypoints_y);
  cout << "Test XY" << endl;
  cout << test_xy[0] << ", " << test_xy[1] << endl;
  cout << test_xy2[0] << ", " << test_xy2[1] << endl;
  cout << "XY distance: " << distance(test_xy[0], test_xy[1], test_xy2[0], test_xy2[1])
    << endl;

  vector<double> test_xy2_1 = getXY2(28.7929058074951, 0.01159054, *world.map_waypoints_s,
                                 *world.map_waypoints_x, *world.map_waypoints_y);

  vector<double> test_xy2_2 = getXY2((28.7929058074951 + dist), (0.01159054 + 0), *world.map_waypoints_s,
                                   *world.map_waypoints_x, *world.map_waypoints_y);
  cout << "Test XY" << endl;
  cout << test_xy[0] << ", " << test_xy2_1[1] << endl;
  cout << test_xy2[0] << ", " << test_xy2_2[1] << endl;
  cout << "XY2 distance: " << distance(test_xy2_1[0], test_xy2_1[1], test_xy2_2[0], test_xy2_2[1])
    << endl;
}

}
}

#endif