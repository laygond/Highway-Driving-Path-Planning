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
    map_waypoints_dx.push_back(d_x); // Frenets d unit vector x component
    map_waypoints_dy.push_back(d_y); // Frenets d unit vector y component
  }

  double max_s = 6945.554;    // meters. The max s value before wrapping around the track back to 0
  int lane = 1;               // lanes are defined as (leftmost,middle,rightmost)=(0,1,2)
  double max_speed = 22.0;    // [m/s] (max should be close but under speed limit)
  double speed_limit = 22.352;// [m/s]. Equivalent to 50mph
  double safe_distance = 30.0;// meters. Minimum front and back distance for car to to take decisions.
  double ref_speed = 0.0;     // [m/s]. End point speed in trajectory vector. Zero since it starts from rest.

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &max_speed,&lane,&speed_limit,&safe_distance,&ref_speed,&max_s]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data from Simulator
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data from Simulator 
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road. (3 right side lanes)
          auto sensor_fusion = j[1]["sensor_fusion"];
          

          // Analyze Sensor Fusion Data (to raise flags)
          bool too_close = false; //to front car
          double front_car_speed;
          double front_car_distance =  safe_distance +1; // greater (farther) than safe_distance
          bool left_lane_free  = true;
          bool right_lane_free = true; 
          for(int i=0; i<sensor_fusion.size(); i++)
          { 
            // Read other car's data 
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            double check_distance = check_car_s-car_s;
            // Check for cars in front of my lane
            if( d >(2+4*lane-2) && d<(2+4*lane+2) )
            { 
              if ( check_car_s>car_s && check_distance<safe_distance )
              {
                too_close = true;  //assumes one car within safety distance range(not multiple)
                front_car_speed = check_speed;
                front_car_distance = check_distance; 
              }
            }
            // Check for cars in right lane (front and back)
            if (lane == 2) right_lane_free = false; 
            else if (lane < 2)
            {
              if( d >(2+4*(lane+1)-2) && d<(2+4*(lane+1)+2) ) //lanewidth=4m
              { 
                if ( check_distance<1.5*safe_distance && check_distance>-safe_distance )
                {
                  right_lane_free = false;  
                }
              }
            }
            // Check for cars in left lane (front and back)
            if (lane == 0) left_lane_free = false;
            else if (lane > 0)
            {
              if ( d>(2+4*(lane-1)-2) && d<(2+4*(lane-1)+2) )
              { 
                if ( check_distance<1.5*safe_distance && check_distance>-safe_distance )
                {
                  left_lane_free = false;  
                }
              }
            }
          }

          // Evaluate Flags
          double target_speed; //the speed the ego car wants to reach
          if (too_close)
          {
            target_speed = front_car_speed;
            if (right_lane_free)
            {
              lane+=1;
              target_speed = max_speed;
            }
            else if (left_lane_free)
            {
              lane-=1;
              target_speed = max_speed;
            }
          }
          else
          {
            target_speed = max_speed;
          }

          /**
          * Define a path made up of (x,y) points that the car will visit
          * sequentially every .02 seconds
          *
          * Problem: map waypoints are too far apart therefore create function to interpolate 
          * Steps - Procedure: 
          * 1. Create vector of 'pts(x,y)' waypoints: {previous, current, 30m, 60m, 90m)
          * 2. Interpolate these waypoints by creating a spline function 's()'
          * 3. Fill vector 'next_vals' with points from 's()' spaced appropiately to control speed
          */

          vector<double> ptsx;  //vector of x points for Step Procedure
          vector<double> ptsy;  //vector of y points for Step Procedure

          // reference as where the car currently is or previous paths end point (map perspective)
          double ref_x;
          double ref_y;
          double ref_x_prev;
          double ref_y_prev;
          double ref_yaw; 

          // Collect previous and current 'pts(x,y)' waypoints
          int prev_size = previous_path_x.size(); 
          if(prev_size < 1) 
          {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);

            ref_x_prev = ref_x - cos(car_yaw); //go backwards in time with the angle to generate 'fake' previous
            ref_y_prev = ref_y - sin(car_yaw); // times the length of the car ??????

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x); 
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y); 

            end_path_s = car_s; //assign to current position since there is no last position yet
          } 
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1]; 

            ref_x_prev = previous_path_x[prev_size-2];
            ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev); 

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x); 
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y); 
          } 

          // Collect {30m,60m,90m} 'pts(x,y)' waypoints
          vector<double> next_wp0 = getXY(end_path_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(end_path_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(end_path_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]); 

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]); 

          // Transform 'pts(x,y)' waypoints into car's end point reference (to prevent problems with spline)
          for (int i = 0; i < ptsx.size(); i++ ) 
          { 
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y; 

            ptsx[i] = (shift_x *cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // Define and set up spline
          tk::spline s;
          s.set_points(ptsx,ptsy);

          // path vector that will be sent back to simulator
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // the simulator always makes use of some of the previous path points, therefore alway less than 50
          // transfer remaining path points from previous iteration
          for (int i=1; i<previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          } 

          // Create Target/Goal Point (from car's end point reference)
          double target_x = 30.0; //meters
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y)); 
          
          // Refill empty 'next_val' slots with new points that the car will visit every .02 seconds
          double x_local = 0.0; // (x,y) end point of trajectory in local car's end point reference
          double y_local;
          for (int i=1; i<=(50-previous_path_x.size()); i++)
          { 
            if ( ref_speed > target_speed || front_car_distance < safe_distance/2 )
            {
              ref_speed -= 0.1;  //[m/s] equivalent to acceleration of -5 [m/s^2]
            }
            else if ( ref_speed < target_speed )
            {
              ref_speed += 0.1;  //[m/s] equivalent to acceleration of 5 [m/s^2]
            }
            ref_speed = std::max(0.00001,ref_speed);   // prevents from going backwards
            double N = (target_dist/(0.02*ref_speed)); // spacing between points so that car travels at ref_speed
            x_local = x_local + (target_x)/N;
            y_local = s(x_local); 
            double x_map = x_local*cos(ref_yaw) - y_local*sin(ref_yaw) + ref_x; //rotate back to map
            double y_map = x_local*sin(ref_yaw) + y_local*cos(ref_yaw) + ref_y; 

            next_x_vals.push_back(x_map);
            next_y_vals.push_back(y_map); 
          }

          // Send data back to Simulator
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if
      } 
      else
      {
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