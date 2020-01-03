#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  static int iteration = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        // set variables
        iteration++;
        bool debug = 1;
        double velocity_max = 49.5/2.24;
        double lane = 1; // TODO
        int max_points = 50;

        // parse string
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int prev_size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Print debug information
          if( debug )
          {
            std::cout << "=======================================================================================================" << std::endl; 
            std::cout << "Iteration:" << iteration << std::endl; 
            std::cout << "CarX:" << car_x << std::endl; 
            std::cout << "CarY:" << car_y << std::endl; 
            std::cout << "CarS:" << car_s << std::endl; 
            std::cout << "CarD:" << car_d << std::endl; 
            std::cout << "CarYAW:" << car_yaw << std::endl; 
            std::cout << "CarSPEED:" << car_speed << std::endl; 
            std::cout << "GOT:" << prev_size << std::endl; 
          }

          vector<double> pts_x;
          vector<double> pts_y;
          double ref_x = car_x;
          double ref_y = car_yaw;
          double ref_yaw = deg2rad(car_yaw);

          if ( prev_size < 2){
            // Processing first 2 points; 

            // Making first points tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);

            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
          }else{
            // Processing consecutive points...

            // Use 2 end points of last path to define new reference point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Make first two points tangent to last path
            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);

            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
          }

          // Waypoints parameters
          int num_waypoints  = 3;
          double waypoint_increment = 30;
          double lane_size = 4;

          // Prepare watpoints in Frenet coordinates 
          for(int w=0;w<num_waypoints;++w)
          {
            vector<double> waypoint = getXY(car_s+(w+1)*waypoint_increment,(lane_size/2+lane_size*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            pts_x.push_back(waypoint[0]);
            pts_y.push_back(waypoint[1]);
          }

          // shift & rotate coordinates
          for(int p=0; p<pts_x.size();++p)
          {
            double shift_x = pts_x[p]-ref_x;
            double shift_y = pts_y[p]-ref_y;

            std::cout << "CHK:" << pts_x[p] <<","<< pts_y[p] << std::endl;

            pts_x[p] = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
            pts_y[p] = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);
          }

          // create a spline and set points
          tk::spline spline;
          spline.set_points(pts_x, pts_y);

          // points for planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;  

          // reuse points from last path
          for(int p=0;p<prev_size;++p)
          {
            next_x_vals.push_back(previous_path_x[p]);
            next_y_vals.push_back(previous_path_y[p]);
          }      

          double target_x = waypoint_increment;
          double target_y = spline(target_x);
          double target_dist = distance_by_diff(target_x, target_y);

          // generate more points from spline calculation
          double current_x = 0;
          double frame_time = 0.02;
          double N = (target_dist/(frame_time*velocity_max));
          double x_increment = target_x/N;
          for(int i=1;i<=max_points-prev_size;++i)
          {
            current_x += x_increment;
            double y = spline(current_x);

            std::cout << current_x <<","<< y-1128<< std::endl;

            // transform back to original coords
            next_x_vals.push_back(current_x*cos(ref_yaw)-y*sin(ref_yaw) + ref_x);
            next_y_vals.push_back(current_x*sin(ref_yaw)+y*cos(ref_yaw) + ref_y);
          }

          /*
          // Sensor fusion data
          for(int s=0;s<sensor_fusion.size();s++)
          {
            // Order of sensor fusion data is [ id, x, y, vx, vy, s, d]
            double other_id = sensor_fusion[s][0];
            double other_x = sensor_fusion[s][1];
            double other_y = sensor_fusion[s][2];
            double other_vx = sensor_fusion[s][3];
            double other_vy = sensor_fusion[s][4];
            double other_s = sensor_fusion[s][5];
            double other_d = sensor_fusion[s][6];

            double dx = car_x - other_x;
            double dy = car_y - other_y;
            double dist = sqrt(dx*dx+dy*dy);

            if(debug)
            {
              std::cout << "Distance of " << other_id << " is:" << dist << std::endl; 
            }
          }*/

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          DELAY(500);
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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