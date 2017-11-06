#ifndef HELPERS_H
#define HELPERS_H

#include <fstream>
#include <tuple>
#include <math.h>

namespace {
  namespace helpers {

    using namespace std;
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

    // Constraints for trajectories.
    struct constraints {
      double target_v;
      double target_lane;
      double max_at; // tangential acceleration
      double max_an; // normal acceleration
      double max_jerk;
    };

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

    double velocity(double x1, // double y1,
                    double x2, // double y2,
                    double dt) {
      /**
       * Calculate acceleration from three points.
       * Earlier points are closer to final position.
       */
      // double v = distance(x1, y1, x2, y2) / dt;
      double v = (x1 - x2) / dt;
      // cout << "v1: " << v1 << ", v2: " << v2 << ", at: " << ((v1-v2)/dt) << endl;
      return v;
    }

    double acceleration(double x1, // double y1,
                        double x2, // double y2,
                        double x3, // double y3,
                        double dt) {
      /**
       * Calculate acceleration from three points.
       * Earlier points are closer to final position.
       */
      // double v1 = distance(x1, y1, x2, y2) / dt;
      // double v2 = distance(x2, y2, x3, y3) / dt;

      double v1 = velocity(x1, x2, dt);
      double v2 = velocity(x2, x3, dt);
      // cout << "v1: " << v1 << ", v2: " << v2 << ", at: " << ((v1-v2)/dt) << endl;
      return ((v1-v2)/dt);
    }

    double jerk(double x1,
                double x2,
                double x3,
                double x4,
                double dt) {
      /**
       * Calculate jerk from four points.
       * Earlier points are closer to final position.
       */

      double a1 = acceleration(x1, x2, x3, dt);
      double a2 = acceleration(x2, x3, x4, dt);
      return ((a1-a2)/dt);
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
      double max_s = 6945.554;

      s = fmod(s, max_s);

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

    void PrintTrajectory(const vector<double> &tj_s,
                         const vector<double> &tj_d,
                         double dt) {
      for (int i = 0; i < tj_s.size(); ++i) {
        cout << tj_s[i] << ", " << tj_d[i];

        if (i > 0) {
          cout << " v: " << velocity(tj_s[i],
                                     tj_s[i-1], dt);
        } 
        if (i > 1) {
          cout << " a: " << acceleration(tj_s[i],
                                         tj_s[i-1],
                                         tj_s[i-2], dt);
        }
        if (i > 2) {
          cout << " j: " << jerk(tj_s[i],
                                 tj_s[i-1],
                                 tj_s[i-2],
                                 tj_s[i-3], dt);
        }

        cout << endl;
      }

    }

    void AddWaypointsToLoop(double max_s,
                            vector<double> *map_waypoints_x,
                            vector<double> *map_waypoints_y,
                            vector<double> *map_waypoints_s,
                            vector<double> *map_waypoints_dx,
                            vector<double> *map_waypoints_dy) {
      /**
       * Add extra waypoint to loop back to the beginning with overlap...
       * Credit to Denise James's:
       * https://github.com/DeniseJames/CarND-Path-Planning-Project/blob/master/src/main.cpp
       */
      (*map_waypoints_x).push_back((*map_waypoints_x)[0]);
      (*map_waypoints_y).push_back((*map_waypoints_y)[0]);
      (*map_waypoints_s).push_back(max_s);
      (*map_waypoints_dx).push_back((*map_waypoints_dx)[0]);
      (*map_waypoints_dy).push_back((*map_waypoints_dy)[0]);
      (*map_waypoints_x).push_back((*map_waypoints_x)[1]);
      (*map_waypoints_y).push_back((*map_waypoints_y)[1]);
      (*map_waypoints_s).push_back(max_s + (*map_waypoints_s)[1]);
      (*map_waypoints_dx).push_back((*map_waypoints_dx)[1]);
      (*map_waypoints_dy).push_back((*map_waypoints_dy)[1]);

      (*map_waypoints_x).push_back((*map_waypoints_x)[2]);
      (*map_waypoints_y).push_back((*map_waypoints_y)[2]);
      (*map_waypoints_s).push_back(max_s + (*map_waypoints_s)[2]);
      (*map_waypoints_dx).push_back((*map_waypoints_dx)[2]);
      (*map_waypoints_dy).push_back((*map_waypoints_dy)[2]);
      (*map_waypoints_x).push_back((*map_waypoints_x)[3]);
      (*map_waypoints_y).push_back((*map_waypoints_y)[3]);
      (*map_waypoints_s).push_back(max_s + (*map_waypoints_s)[3]);
      (*map_waypoints_dx).push_back((*map_waypoints_dx)[3]);
      (*map_waypoints_dy).push_back((*map_waypoints_dy)[3]);

      (*map_waypoints_x).push_back((*map_waypoints_x)[4]);
      (*map_waypoints_y).push_back((*map_waypoints_y)[4]);
      (*map_waypoints_s).push_back(max_s + (*map_waypoints_s)[4]);
      (*map_waypoints_dx).push_back((*map_waypoints_dx)[4]);
      (*map_waypoints_dy).push_back((*map_waypoints_dy)[4]);
      (*map_waypoints_x).push_back((*map_waypoints_x)[5]);
      (*map_waypoints_y).push_back((*map_waypoints_y)[5]);
      (*map_waypoints_s).push_back(max_s + (*map_waypoints_s)[5]);
      (*map_waypoints_dx).push_back((*map_waypoints_dx)[5]);
      (*map_waypoints_dy).push_back((*map_waypoints_dy)[5]);
    }

  }

}

#endif