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
  double max_s = 6945.554; // TODO handle cyclic values

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
        // set variables & parameters
        iteration++;

        int num_waypoints  = 3;
        double waypoint_increment = 30;
        double lane_size = 4;
        double max_acc = 6; // m/s^2

        double speed_conv_ratio = 2.24;
        double velocity_max = 49.5/speed_conv_ratio;
        double lane = 1; 
        double frame_time = 0.02;
        double lane_clearance_dist = 30;
        int max_points = 50;
        double max_v_diff = max_acc * frame_time; // m/s

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
          double last_speed = 0;
          double target_speed = car_speed;

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

            last_speed = distance(ref_x,ref_y, ref_x_prev, ref_y_prev) / frame_time;
          }

         // Sensor fusion data
          bool slow_down = false;
          double lane_clearance_front[3];
          double lane_clearance_back[3];
          for(int i=0;i<3;++i)
          {
            lane_clearance_front[i] = 100;
            lane_clearance_back[i] = 100;
          }
          for(int s=0;s<sensor_fusion.size();s++)
          {
            // Order of sensor fusion data is [ id, x, y, vx, vy, s, d]
            double other_id = sensor_fusion[s][0]; // TODO remove not needed values
            double other_x = sensor_fusion[s][1];
            double other_y = sensor_fusion[s][2];
            double other_vx = sensor_fusion[s][3];
            double other_vy = sensor_fusion[s][4];
            double other_s = sensor_fusion[s][5];
            double other_d = sensor_fusion[s][6];

            double dx = car_x - other_x;
            double dy = car_y - other_y;
            double dist_front = other_s-car_s;
            double dist_back = car_s-other_s;

            // Slow down logic
            if (other_s>car_s && other_s-car_s<waypoint_increment && abs(other_d-car_d)<lane_size/2 )
            {
              double other_v_magnitude = sqrt(other_vx*other_vx+other_vy*other_vy);
              if(target_speed>other_v_magnitude)
              {
                target_speed = other_v_magnitude;
              }

              slow_down = true;
            }

            // Lane clerance update
            for(int i=0;i<3;++i)
            {
              double lane_left_boundary = i*4;
              double lane_right_boundary = (i+1)*4;

              if(other_d>lane_left_boundary && other_d<lane_right_boundary)
              {
                if(dist_front>0 && dist_front<lane_clearance_front[i])
                  lane_clearance_front[i] = dist_front;
                if(dist_back>0 && dist_back<lane_clearance_back[i])
                  lane_clearance_back[i] = dist_back;

              }
            }
          }

          int lane_int = int(car_d / lane_size);
          double minimal_distance_to_change = 5;

          std::cout << "Lane clearance(front): "<< lane_clearance_front[0] << "," <<lane_clearance_front[1]<<","<<lane_clearance_front[2]<<std::endl;
          std::cout << "Lane clearance(back): "<< lane_clearance_back[0] << "," <<lane_clearance_back[1]<<","<<lane_clearance_back[2]<<std::endl;
          std::cout << "car_d : "<< car_d <<std::endl;
          std::cout << "Current lane float : "<< (car_d-lane_size) <<std::endl;
          std::cout << "Current lane : "<< lane_int <<std::endl;
          std::cout << "Current lane clearance (front): "<< lane_clearance_front[lane_int] <<std::endl;
          std::cout << "Current lane clearance (back): "<< lane_clearance_back[lane_int] << std::endl;

          // Lane switching
          if( lane_clearance_front[lane_int] < lane_clearance_dist )
          {
            if(lane-1>=0 && lane_clearance_back[lane_int-1]> lane_clearance_dist && 
                lane_clearance_front[lane_int-1]>lane_clearance_front[lane_int]+minimal_distance_to_change)
            {
              lane -= 1;
            }else{
              if(lane+1>=2 && lane_clearance_back[lane_int+1]> lane_clearance_dist && 
                  lane_clearance_front[lane_int+1]>lane_clearance_front[lane_int]+minimal_distance_to_change)
              {
                lane +=1;
              }
            }
          }

          // Prepare waypoints in Frenet coordinates 
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

          // Speed up logic
          bool speed_up = true;
          if (!slow_down && target_speed<velocity_max)
          {
            target_speed = velocity_max;
          }

          // Velocity constraints checks
          if(target_speed<0)
            target_speed = 0;
          if(target_speed> velocity_max)
            target_speed = velocity_max;

          // Handle acceleration and jerk violations
          double velocity_diff = target_speed-last_speed;
          double time_diff = frame_time * (max_points-prev_size);
          double mean_acc = velocity_diff/time_diff;

          bool end = false;
          double current_x = 0;
          double desired_frame_increment = target_speed * frame_time;
          int counter = 0;
          while(!end)
          {
            counter++;
            // Calculate max & min velocity not to violate acc constraints
            double v_start = last_speed;
            double v_max = v_start + max_v_diff;
            double v_min = v_start - max_v_diff;
            if (v_min<0)
              v_min=0;

            double s_max = v_max * frame_time;
            double s_min = v_min * frame_time;

            double increment = desired_frame_increment;
            if(increment>s_max){
              increment=s_max;
            } 

            if(increment<s_min){
              increment=s_min;
            } 

            last_speed = increment/frame_time;

            current_x += increment;

            if(counter > max_points-prev_size)
            {
              end = true;
              continue;
            }

            if( current_x > target_dist){
              end=true;
            }else{
              // Add path planning points to output with applying reverse transform of coordinates
              double y = spline(current_x);
              next_x_vals.push_back(current_x*cos(ref_yaw)-y*sin(ref_yaw) + ref_x);
              next_y_vals.push_back(current_x*sin(ref_yaw)+y*cos(ref_yaw) + ref_y);  
            }
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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