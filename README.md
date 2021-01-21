
# **Path-Planning-Project**

[//]: # (Image References)

[image1]: ./data/simulation_for20min.png "simulation for 20 minutes"
[image2]: ./data/input_output.png "behavior function inputs/outputs"
[image3]: ./data/speed.png "speed penaly"
[image4]: ./data/lane_change.png "lane change penaly"
[image5]: ./data/compute_time.png "timing schedule"

![alt text][image1]

---

## Goals

In this project the goal is to implement a path planner(FSM behavior planner + local planner) that safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The planner is provided the car's localization and sensor fusion data and there are also a sparse map list of waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible and other cars will try to change lanes too. The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Behavior planner

The objective is to suggest an appropriate maneuver in time, such as target lane, target vehicle to follow, target speed, time to reach targets, etc., 

* Lane Keep
    - d: Stay near center line for lane
    - s: Drive at target speed when feasible, othewise drive at whatever speed is safest for the lane
* Lane Change Left/Right
    - d: Move left or right
    - s: Drive at target speed when feasible, othewise drive at whatever speed is safest for the lane
* Prepare Lane Change Left/Right
    - d: Stay near center line for lane
    - s: Attempt to match position and speed of "gap" in lane
    
#### State transition function

In the sate transition function, we first consider states that can be reached from current FSM state to keep track of the total cost of each state. For the cost, we first generate a rough idea of what trajectory we would follow if we chose this state and calculate the "cost" associated with that trajectory. We apply each cost function to the generated trajectory and multiply the cost by the associated weight to find the minimum cost state. The least cost trjectory is monitored, and a lane change is executed whenever possible and safe.

```
def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights):

    possible_successor_states = successor_states(current_fsm_state)
    
    costs = []
    for state in possible_successor_states:
        
        trajectory_for_state = generate_trajectory(state, current_pose, predictions)

        cost_for_state = 0
        for i in range(len(cost_functions)) :
            
            cost_function = cost_functions[i]
            cost_for_cost_function = cost_function(trajectory_for_state, predictions)

            weight = weights[i]
            cost_for_state += weight * cost_for_cost_function
         costs.append({'state' : state, 'cost' : cost_for_state})

    best_next_state = None
    min_cost = 9999999
    for i in range(len(possible_successor_states)):
        state = possible_successor_states[i]
        cost  = costs[i]
        if cost < min_cost:
            min_cost = cost
            best_next_state = state 

    return best_next_state
```

### Cost functions

Behavior planner gets inputs from multiple sources to plan its path. The simulator provides sensor fusion data for each near-by car[id,x,y,vx,vy,s,d]. Predictions are made not only on ego car's s coordinate but also on near-by cars' s coordinate in the future.

#### Speed panelty

![alt text][image3]

#### Lane change panelty

We want a cost function that penalizes large |\Delta d|∣Δd∣ and we want that penalty to be bigger when \Delta sΔs is small.
Furthermore, we want to make sure that the maximum cost of this cost function never exceeds one and that the minimum never goes below zero. The cost increases with both the distance of intended lane from the goal and the distance of the final lane from the goal. The cost of being out of the goal lane also becomes larger as the vehicle approaches the goal.

![alt text][image4]


## Local planner

#### Compute scheduling

The objective is to create a path that smoothly changes lanes. Namely, the path is made up of (x,y) points that the car will visit sequentially every .02 seconds. General rule of trajectory planner is to keep safe distance from other near-by cars and the comfort of low acc./jerk. Conditions given by simulator are, simulator cycle 1 move / 0.02 sec and horizon 50 moves(1 sec). Therefore, when my speed is 50mph(25m/ sec), each move is around 0.5m long.

There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long(1-3 time steps). During this delay, the simulator will continue using points that it is last given. Because of this, its a good idea to store the last points we have used so we can have a smooth transition. Previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator. We would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

![alt text][image5]

#### Cubic spline interpolation
   
A spline is generated by the cubic spline interpolation function in [spline.h](https://kluge.in-chemnitz.de/opensource/spline/). In Frenet coord., evenly 30m spaced points are added ahead of the starting reference. These 3 points play the role of anchor points to create interpolated spline points. Those points are first transformed to x,y coord using getXY function.

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
    
#### Smooth trajectory

On every websocket message arrival in the C++ program, there are the path points returned, which are not yet consumed by the simulator. So, for smooth trajectory generation, the previous path's end points has to be used as starting reference. Leaving all of the previous path points from last time, new path points are added ahead of them, in order to make 1 sec long trajectory(50 moves). Then spline points are broken up such that we travel at our desired reference velocity. N points are spaced along the spline at the desired speed. N = d /(.02 * vel) because v = d/t. where t is N * .02. 
   
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

#### Sharp decceleration

(x,y) points are in meters and the spacing inbetween determines the speed of the car. The vector from a point to the next dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total acceleration. (x,y) points in a path should not have a total acceleration that goes over 10 m/s^2. The jerk also should not go over 50 m/s^3. Note that these requirements might change for BETA version simulator. For instance, jerk is measured over a .02 second interval and is the average total acceleration over 1 second.

Time to collision(TTC) is calculated for each possible trajectory based on the current positions. And only non-collision trajectories survive. TTC is distance delta divided by relative speed delta. To get the minimum decceleration, relative speed divided by n moves is subtracted from relative speed delta. n is calculated by dividing TTC by simulation cycle 0.02 second. In simulation test, calculating TTC and decceleration by n points help avoid passing the maximum acceleration/decceleration.  
   
```
    double ttc = (check_car_s0 - car_s0)/abs(check_speed - car_speed);
    double ltc = ttc*car_speed;
    double n = ttc/.02; //n points move
    :
    if (ref_vel > check_speed*2.24){
        ref_vel -= (ref_vel-check_speed*2.24)/n ;
    }
```
---

## Simulator

This project is a part of Self-Driving Car Engineer Nanodegree Program. You can download the simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

Here is the data provided from the Simulator to the C++ Program.

#### Main car's localization Data (No Noise)

* ["x"] The car's x position in map coordinates
* ["y"] The car's y position in map coordinates
* ["s"] The car's s position in frenet coordinates
* ["d"] The car's d position in frenet coordinates
* ["yaw"] The car's yaw angle in the map
* ["speed"] The car's speed in MPH

#### Previous path data given to the Planner

Note that it returns the previous list but with processed points removed, whch can be a nice tool to show how far along
the path has processed since last time. 

* ["previous_path_x"] The previous list of x points previously given to the simulator
* ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

* ["end_path_s"] The previous list's last point's frenet s value
* ["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

A 2d vector of cars, ["sensor_fusion"] includes:
* [car's unique ID]
* [car's x position in map coordinates, car's y position in map coordinates]
* [car's x velocity in m/s, car's y velocity in m/s]
* [car's s position in frenet coordinates, car's d position in frenet coordinates]


## Map

#### The map of the highway is in data/highway_map.txt
The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.


---
## Build

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`


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
