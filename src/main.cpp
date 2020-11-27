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
//#include "vehicle.h"
#include <cmath>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double getlane(double car_d)
{
    double lane = 1;  // default CENTER lane
    if (car_d > 0 && car_d < 4)
    {
        lane = 0;  // LEFT
    }
    else if (car_d >= 4 && car_d <= 8)
    {
        lane = 1;  // CENTER
    }
    else if (car_d > 8 && car_d <= 12)
    {
        lane = 2;  // RIGHT
    }
    return lane;
}

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

  /*
  target speed: 49.5mph (22.12m/sec)
  goal s: 6945.554m (4.316 mile)
  e.t.a: about 5.23 minutes.
  */
  int lane = 1;//initial lane
  double ref_vel = 0.;//initial vel. in mph.

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode){
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);

      if (s != "") {
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
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side
            //   of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];


          /*
           * Behavior planner:
           *    Objective is to suggest an appropriate maneuver in time
           *    such as target lane, target vehicle to follow, target speed, time to reach targets, etc.,
           *
           * General steps are,
           *    1.predict on vehicles around
           *    2.check my sucessor states
           *    3.for each next state, generate trajectories
           *    4.evaluate each traj. w/ cost function
           *    5.execute traj. w/ optimal cost
           */
            
            //variables to calc. time-to-collision
            double car_s0 = car_s;
            double ttc;
            
            //variables for danger in safety
            bool too_close = false;
            bool left_safe = true;
            bool right_safe = true;
            int too_close_idx = -1;
            
            //A simple finite state machine(FSM) is used
            //state 0:keep lane, 1:change lane left, 2:change lane right
            int state = 0;
            
            //variables for calc. cost functions
            vector<double> emptyahead= {99999.,99999.,99999.};
            vector<double> freeway = {0.,0.,0.};
            vector<int>  waycount = {0,0,0};
            
            //my car s coord. in the future horizon.
            int prev_size = previous_path_x.size();
            if (prev_size > 0){
                car_s = end_path_s;
            }
            
            //get sensor fusion data for each near-by car. [id,x,y,vx,vy,s,d]
            for (int i = 0; i<sensor_fusion.size(); i++)
            {
                float  id = sensor_fusion[i][0];
                float  d  = sensor_fusion[i][6];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
                
                //near-by car s coord. in the future horizon.
                check_car_s += ((double)prev_size*.02*check_speed);
                
                //near-by car d coord. in the future.
                //d += ((double)prev_size*.02*vx);
                
                //If a car in my lane is predicted to be less than 30m ahead, that car is too close.
                if((2+4*lane-2) < d && d < (2+4*lane+2)){
                    if((check_car_s > car_s) && (check_car_s - car_s) < 30){
                        too_close = true;
                        too_close_idx = i;
                    }
                }
                //If a car in my left is predicted within 20m, lane change is not safe
                else if((2+4*lane-2-4) < d && d < (2+4*lane-2))
                {
                    if(abs(check_car_s - car_s) < 20){
                        left_safe = false; //not safe to change to the left
                    }
                    else if(check_car_s > car_s && check_car_s-car_s < emptyahead[getlane(d)]){
                        emptyahead[getlane(d)] = (check_car_s - car_s);
                    }
                }
                else if((2+4*lane+2) < d && d < (2+4*lane+2+4))
                {
                    if(abs(check_car_s - car_s) < 20){
                        right_safe = false; //not safe to change to the right
                    }
                    else if(check_car_s > car_s && check_car_s-car_s < emptyahead[getlane(d)]){
                        emptyahead[getlane(d)] = (check_car_s - car_s);
                    }
                }
                
                freeway[getlane(d)]+=check_speed;
                waycount[getlane(d)]+=1;

            }
            
            //The cost function for a lane:
            //Each non ego vehicles in a lane has a different speed, so to get the speed limit for a lane,
            //i used the avg. of vehicles in that lane.
            //Although that lane may be slow, ego car can take a chance to select it if there's vast space ahead.
            //Therefore, the cost is avg. lane speed plus amount of space ahead.
            for (int k=0; k<3; k++){
                if(waycount[k] != 0){
                    freeway[k] /= waycount[k];
                    freeway[k] = (49.5-freeway[k])/49.5;
                    
                    if(emptyahead[k] == 99999.){
                        emptyahead[k] = 0.;
                    }
                    freeway[k] = (49.5-freeway[k])/49.5 + 3*exp(-(emptyahead[k]));
                }
            }
            
            if(state == 0){
 
                if(too_close){
                    
                    //get too close car information
                    float  id = sensor_fusion[too_close_idx][0];
                    double vx = sensor_fusion[too_close_idx][3];
                    double vy = sensor_fusion[too_close_idx][4];
                    double check_car_s0 = sensor_fusion[too_close_idx][5];
                    float d_f = sensor_fusion[too_close_idx][6];
                    
                    //calc. too-close car s coord. in the future.
                    double check_speed = sqrt(vx*vx+vy*vy);
                    double check_car_s = check_car_s0 + ((double)prev_size*.02*check_speed);

                    //calc. time to collision based on the current position
                    double ttc = (check_car_s0 - car_s0)/abs(check_speed - car_speed);
                    double ltc = ttc*car_speed;
                    double n = ttc/.02; //n points move
                    
                    // divide relative speed by n moves, and deccelerate until equal speed.
                    if (ref_vel > check_speed*2.24){
                        ref_vel -= (ref_vel-check_speed*2.24)/n ;
                    }
                    
                    //std::cout<<" ttc:"<<ttc<<" ltc:"<<ltc<<" n:"<<n<<" car_s0:"<<car_s0<<" car_s:"<<car_s<<std::endl;
                    std::cout<<"c0:"<<freeway[0]<<" c1:"<<freeway[1]<<" c2:"<<freeway[2]<<std::endl;
                    
                    // check least cost lane, execute lance change if safe.
                    vector<double>::iterator best_cost = min_element(freeway.begin(), freeway.end());
                    int best_lane = distance(freeway.begin(), best_cost);
                    if(best_lane != lane)
                    {
                        //if(left_safe && lane > 0 && lane-1 == best_lane){
                        if(left_safe && lane > 0 && lane-1 == best_lane){
                            lane -=1;
                            std::cout<<"[to the left]"<< lane << std::endl;
                            state = 1;
                        }
                        else if(right_safe && lane < 3 && lane+1 == best_lane){
                            lane +=1;
                            std::cout<<"[to the right]"<< lane << std::endl;
                            state = 2;
                        }
                    }
                    
                }
                
                if(!too_close && ref_vel < 49.5){
                    ref_vel += .224;
                }
            }
            else if(state == 1){
                //if change completed, go back to kl state
                //if(getlane(car_d) == lane && abs(car_d - 2+4*lane) < 0.5){
                if(getlane(car_d) == lane){
                    state = 0;
                }
            }
            else if(state == 2){
                //if change completed, go back to kl state
                if(getlane(car_d) == lane){
                     state = 0;
                 }
            }
            
          /*
           * Trajectory planner:
           *    Objective is to create path that smoothly changes lanes.
           *    Namely, the path is made up of (x,y) points that the car will visit sequentially every .02 seconds
           * General rules are,
           *    to keep safe distance from other near-by cars and the comfort of low acc./jerk.
           * Given conditions are,
           *    a simulator cycle: 1 move / 0.02 sec.
           *    horizon: 50 move (1 sec).
           *    therefore, when my speed is 50mph(25m/ sec), each move is 0.5m long.
           */

            //variables to hold final x,y coord. of path
            vector<double> ptsx;
            vector<double> ptsy;
            
            //keep the current state of my car as reference state
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            
            if (prev_size < 2){
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);
                
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            // use the previous path's end point as starting reference
            else{
                //redefine reference state as previous path end point
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];
                
                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
                
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);
                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }
            
            //In Frenet, add evenly 30m spaced points ahead of the starting reference
            //these 3 are anchor points to create interpolated spline.
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
            
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            //change from global x,y coord. to vehicle coord.
            for (int i = 0; i < ptsx.size(); i++){
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                
                ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
                ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));
            }
            
            //create a spline using cubic spline interpolation function(spline.h)
            //as in https://kluge.in-chemnitz.de/opensource/spline/
            tk::spline s;
            s.set_points(ptsx,ptsy);
            
            //define the actual (x,y)points we will use for the planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            //start with all of the previous path points from last time
            for (int i=0; i< previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
        
            //caculate how to break up spline points so that we travel at our desired reference velocity.
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
            double x_add_on = 0;

            //get points that are spaced along the spline
            //and spaced in such a way that the car will go at the desired speed
            //v = d/t. where t is N * .02
            //therefore, N = d /(.02 * vel)
            for (int i =0; i<= 50-previous_path_x.size(); i++)
            {
                double N = (target_dist/(.02*ref_vel/2.24)); //to convert mph to mps, devide it by 2.24
                double x_point = x_add_on+(target_x)/N;
                double y_point = s(x_point);
                
                x_add_on = x_point;
                
                double x_ref = x_point;
                double y_ref = y_point;
                
                //rotate back from vehicle coord. to global coord.
                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
                
                x_point += ref_x;
                y_point += ref_y;
                
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }
            
            json msgJson;
            
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      }
      else {
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
