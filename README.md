
# **Path-Planning-Project**

[//]: # (Image References)

[image1]: ./data/simulation_for20min.png "simulation for 20 minutes"


---

![alt text][image1]

---

### Simulator.
This project is a part of Self-Driving Car Engineer Nanodegree Program. You can download the simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

**3. Behavior planner block implementation:** 

The objective is to suggest an appropriate maneuver in time, such as target lane, target vehicle to follow, target speed, time to reach targets, etc., General steps are as following.
    
   - **predictions on vehicles around**
    Simulator provided sensor fusion data for each near-by car[id,x,y,vx,vy,s,d]. Predictions were made on near-by cars' s coord. in the future.
    
    ```
    car_s = end_path_s; //my car s coord. in the future horizon.
    :
    check_car_s += ((double)prev_size*.02*check_speed); 
    ```
    
   - **checking sucessor states**
    A simple finite state machine(FSM) was used(state 0:keep lane, 1:change lane left, 2:change lane right). Keep lane state changed to Lane Change Left(LCL) or Lane Change Right(LCR) if ego car found a better cost lane. And LCL/LCR state went back to KL only when ego car arrived at the new lane.
    
    ```
    if(state == 0){
        if(too_close){
          :
        }
    else if(state == 1){
        if(getlane(car_d) == lane){
          state = 0;
        }
    }
    ```
    
   - **generating trajectories**
   Time to collision(TTC) was calculated for each possible trajectory based on the current position. And only non-collision trajectories survive. TTC is distance delta divided by relative speed delta. To get the minimum decceleration, relative speed divided by n moves is subtracted. n is calculated by dividing TTC by simulation cycle 0.02 second. In simulation test, calculating TTC and decceleration by n points helped the max. acceleration/decceleration exceeding problems.  
   
    ```
    double ttc = (check_car_s0 - car_s0)/abs(check_speed - car_speed);
    double ltc = ttc*car_speed;
    double n = ttc/.02; //n points move
    :
    if (ref_vel > check_speed*2.24){
        ref_vel -= (ref_vel-check_speed*2.24)/n ;
    }
    ```
    
   - **evaluating each trajectory with cost function**
    Each non ego vehicles in a lane has a different speed, so to get the speed limit for a lane, i used the avg. of vehicles in that lane. Although that lane may be slow, ego car can take a chance to select it if there's vast space ahead(emptyahead cost). Therefore, the cost is avg. lane speed plus amount of space ahead in the neighbor lanes. This cost function allowed travel time to the goal to be shorter. The weight ratio of 1:3 worked quite well in that ego vehicle achieved shorter travel time and made a sharp lane change when another car suddenly cut in the lane.
    
    ```
    for (int k=0; k<3; k++){
    if(waycount[k] != 0){
        freeway[k] /= waycount[k];
        freeway[k] = (49.5-freeway[k])/49.5;
        :
        freeway[k] = (49.5-freeway[k])/49.5 + 3*exp(-(emptyahead[k]));
    }
    ```
    
   - **enforcing optimal cost trajectory**
    The least cost trjectory was monitored, and a lane change was executed when possible and safe. If a car in my lane was predicted to be less than 30m ahead, the car was deemed to be too close and not safe. If a car in my left was predicted within 20m, lane change was deemed not safe. 
    
    ```c++
    vector<double>::iterator best_cost = min_element(freeway.begin(), freeway.end());
    int best_lane = distance(freeway.begin(), best_cost);
    if(best_lane != lane)
    {
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
    ```
    
   The log below shows that the ego car waited until LCR became safe. Then the optimal lane changed to the lane 2. And then the ego car executed LCR again with little wait this time.
   
    ```
    c0:3.98754 c1:0.987319 c2:3.98688
    c0:3.98754 c1:0.987334 c2:3.98689
    c0:3.98754 c1:0.987348 c2:3.9869
    c0:3.98754 c1:0.987361 c2:3.98691
    c0:3.98754 c1:0.98737 c2:3.98691
    c0:3.98754 c1:0.987382 c2:3.98692
    c0:3.98754 c1:0.987398 c2:3.98694
    c0:3.98754 c1:0.987406 c2:3.98694
    c0:3.98754 c1:0.987417 c2:3.98695
    c0:3.98754 c1:0.987428 c2:3.98696
    c0:3.98754 c1:0.987439 c2:3.98697
    c0:3.98754 c1:0.987449 c2:3.98697
    c0:3.98754 c1:0.987462 c2:3.98698
    c0:3.98754 c1:0.987469 c2:3.98699
    c0:3.98754 c1:0.987478 c2:3.987
    c0:3.98754 c1:0.987484 c2:3.987
    c0:3.98754 c1:0.987496 c2:3.98693
    [to the right]1
    c0:3.98771 c1:3.98706 c2:0.986769
    [to the right]2
    c0:3.98765 c1:3.98679 c2:3.98684
    [to the left]1
    ```
 
**4. Trajectory planner block implemenation:** 

The objective is to create a path that smoothly changes lanes. Namely, the path is made up of (x,y) points that the car will visit sequentially every .02 seconds. General rule of trajectory planner is to keep safe distance from other near-by cars and the comfort of low acc./jerk. Conditions given by simulator were, simulator cycle 1 move / 0.02 sec and horizon 50 move(1 sec). Therefore, when my speed was 50mph(25m/ sec), each move was around 0.5m long.
       
   - **cubic spline interpolation**
   
   A spline was generate by the cubic spline interpolation function in [spline.h](https://kluge.in-chemnitz.de/opensource/spline/). In Frenet coord., evenly 30m spaced points were added ahead of the starting reference. These 3 points were anchor points to create interpolated spline points. Those points were transformed to x,y coord using getXY function.

    ```
    vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
    :
    tk::spline s;
    s.set_points(ptsx,ptsy);
    ```
    
   - **seamless smooth trajectory**
   
   On every websocket message arrival, the simulator returned the path points that were not consumed yet to my C++ program. So, for seamless and smooth trajectory generation, the previous path's end points were used as starting reference. Leaving all of the previous path points from last time, new path points were added on top of them, in order to make 1 sec long trajectory(50 moves). Then spline points were broken up such that we traveled at our desired reference velocity. N points were spaced along the spline at the desired speed. N = d /(.02 * vel) because v = d/t. where t is N * .02. 
   
```
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
    double x_add_on = 0;
    :
    for (int i =0; i<= 50-previous_path_x.size(); i++)
    {
        double N = (target_dist/(.02*ref_vel/2.24)); //to convert mph to mps, devide it by 2.24
        double x_point = x_add_on+(target_x)/N;
        double y_point = s(x_point);
        :
```
---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
