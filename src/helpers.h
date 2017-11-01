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
      double max_v;
      double max_at; // tangential acceleration
      double max_an; // normal acceleration
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

    double acceleration(double x1, double y1,
                        double x2, double y2,
                        double x3, double y3,
                        double dt) {
      /**
       * Calculate acceleration from three points.
       * Earlier points are closer to final position.
       */
      double v1 = distance(x1, y1, x2, y2) / dt;
      double v2 = distance(x2, y2, x3, y3) / dt;
      // cout << "v1: " << v1 << ", v2: " << v2 << ", at: " << ((v1-v2)/dt) << endl;
      return ((v1-v2)/dt);
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

  }

}

#endif