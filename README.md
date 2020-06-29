# Highway Path Planning
The goal is to safely navigate around a virtual highway with other vehicles which are changing lanes while driving +-10 MPH of the 50 MPH speed limit. The ego car is provided a sparse map, and its location along with sensor fusion data to estimate the location of all the vehicles on the same side of the road. Behavioral planning and trajectory planning are used to create smooth, safe trajectories. This involves driving as close as possible to the 50 MPH speed limit, passing slower traffic when possible, avoid hitting other cars, driving inside of the marked road lanes at all times, and not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3. This repo uses [Udacity's CarND-Path-Planning-Project repo](https://github.com/udacity/CarND-Path-Planning-Project) as a base template and guide.

<p align="center"> 
  <img src="./README_images/path_planning.gif">

  The car has driven autonomously for more than 15 min without any incidents.
</p>

[//]: # (List of Images used in this README.md)
[image1]: ./README_images/highway_sim.png "Simulator"
[image2]: ./README_images/subsystems.png "Subsystems"
[image3]: ./README_images/sensor_fusion.png "Prediction"
[image4]: ./README_images/behavior.png "Behavior"
[image5]: ./README_images/match_speed.gif "Match Speed"
[image6]: ./README_images/spline.jpg "spline"

## Directory Structure
```
.Highway-Driving-Path-Planning
├── CMakeLists.txt        # Compiler Instructions
├── cmakepatch.txt        # Sub Dependency for Mac
├── install-mac.sh        # Dependency for Mac
├── install-ubuntu.sh     # Dependency for Linux
├── .gitignore            # git prevents unnecessary uploads
├── README.md
├── README_images         # Images used by README.md
|   └── ...
├── data                  # waypoints of center of highway
|   └── highway_map.csv
├── build.sh              # Compiles to create a build
├── run.sh                # Runs the build
└── src                   # C++ code
    ├── json.hpp          # json helper functions
    ├── main.cpp          
    ├── helpers.h         # helper functions for main.cpp
    ├── aaron.cpp         # extra: helped as base code for main 
    ├── spline.h          # spline library
    └── Eigen-3.3         # more math libraries
        └── ...
```
Note: cmakepatch.txt is used by install-mac.sh. The [spline library](http://kluge.in-chemnitz.de/opensource/spline/) for creating smooth trajectories and [Aaron's starting code](https://www.youtube.com/watch?time_continue=4974&v=7sI3VHFPP0w&feature=emb_logo) were really helpful resources for doing this project.

## Installation
Open your terminal and type:
```sh
git clone https://github.com/laygond/Highway-Driving-Path-Planning.git
cd Highway-Driving-Path-Planning
sudo ./install-ubuntu.sh # (or './install-mac.sh')
```
This shell file will install the dependencies for our project
- cmake >= 3.5
- make >= 4.1 (Linux, Mac), 3.81 (Windows)
- gcc/g++ >= 5.4
- uWebSocketIO  # Allows communication bewtween c++ code and simulator

Note: you might need to grant executable permission to the install shell files before running them: `chmod +x install-ubuntu.sh`. Running the install may also need the root permission prefix `sudo`. For Windows set-up and more details go to the 'Dependencies section' from [here](https://github.com/udacity/CarND-Path-Planning-Project)

## Udacity's Simulator

![alt text][image1]

The simulator can be downloaded [here (Choose latest Term 3 Version)](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). After the simulator is downloaded, make sure it has executable permision: `chmod +x term3_sim.x86_64` (in the case of linux). 

## Run Project
You can either run the following or the simulator first. The order is irrelevant. In terminal go to your Highway-Driving-Path-Planning repo and type:
```sh
./build.sh
./run.sh
```
Note: you might need to `chmod +x` the build and run shell files.

## Project Analysis
### Overview
![alt text][image2]

In this project we will deal with the Planning Subsystem. This means that information from the Perception Subsystem is already provided. For the Control subsystem the car uses a perfect controller and will visit every (x,y) point it receives from the Planning subsystem.

### Perception Subsystem
The ego car is provided a sparse map, its location, the previous path data from simulator, and sensor fusion data.

<b>Highway Waypoint Map</b>

The map of the circular highway is in `data/highway_map.csv`.  The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway. The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway. The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. Each waypoint [x,y,s,dx,dy] has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component). The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point. The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. 

<b>Main car's localization Data </b>

Variable | Description
--- | --- 
"x"     | The car's x position in map coordinates 
"y"     | The car's y position in map coordinates 
"s"     | The car's s position in frenet coordinates 
"d"     | The car's d position in frenet coordinates 
"yaw"   | The car's yaw angle in the map 
"speed" | The car's speed in MPH 

NOTE: No noise is assumed in the data given.

<b>Previous path data from Simulator</b>

Variable | Description
--- | ---
"previous_path_x" | The previous list of x points previously given to the simulator 
"previous_path_y" | The previous list of y points previously given to the simulator 
"end_path_s" | The previous list's last point's frenet s value 
"end_path_d" | The previous list's last point's frenet d value 

NOTE: before the ego car sends its trajectory to the control subsytem (back to simulator), it makes sure its trajectory vector has 50 points to travel. These 50 points are supposed to be travelled within 1 second (.02 seconds each point), however, a loop in a simulator takes less than a second. Hence, just a portion of those 50 points are crossed. The remaining points in the vector are sent back from the simulator; these points are referred as 'the previous path data from simulator.'

<b> Sensor Fusion Data</b>

"sensor_fusion" is a 2d vector that contains all the information about the cars on the right-hand side of the road. The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car. No Noise is assumed in the data given.

### Planning subsystem
The Planning subsystem is conformed by several modules
- Route or Mission Planning
- Prediction
- Behavioral Planning
- Trajectory Planning (Path Planner + Scheduler)

<b>Route Planning</b> is the high level path of the vehicle between two points on a map. Since the map is a circular highway there is no reason to specify the final goal. The highway's waypoints loop around.Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.

<b>Prediction</b> is what identifies which maneuver other objects on the road might take. For instance vx and vy from the senor fusion data can be useful for predicting where the cars will be in the future. However, in this repo I place conditions that do not lead me to look so much into the future and just use the data I get in real time. The information I seek is whether there are vehicles in front of my lane and whether I am at a safe distance from it. I also check if there are cars in the right and left lane and if there are at a safe distance for a lane change. In code it looks like

![alt text][image3] 

<b>Behavior Planning</b> decides what maneuver our vehicle should take. In this project our car goes at the maximum permitted speed (near speed limit) until it encounters a vehicle in front in which case it matches the front car speed. The ego vehicle (our car) will continue to match the front car speed until either the right or left lane are free to make a lane change at max speed once again. In code it looks like

![alt text][image4]

![alt text][image5]

The ego vehicle matching the front car speed after a lane change.

<b>Path Planner:</b> Path planning is an ambigous term that requires clarification for incomers. Technically, it refers only to the geometrical waypoints we would like our vehicle to follow with no information about time. A velocity profile or scheduler is needed to tell the ego car when it should be at each waypoint. Combining the path planner with the scheduler makes up the trajectory planner. Due to historical reason, however, the term path planning is used interchangeably to relate to the entire planning subsystem, hence, causing confusion. 

In this project the scheduler is fixed to .02 seconds. The car will visit every (x,y) point it receives in the list every .02 seconds. Therefore the objective in this module is to only focus on the path creation for the car to navigate safely and the distancing between waypoints to control the speed and prevent jerk (the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3).

The ego car makes sure its trajectory vector has always 50 points to travel. These 50 points are supposed to be travelled within 1 second (.02 seconds each point), however, a loop in a simulator takes less than a second. Hence, just a portion of those 50 points are crossed. From these remaining points the last two points from this trajectory vector are used as a starting guidance to generate the new points that will fill the 50-point trajectory vector. Once the behavioral planner agrees on what lane the car should be in and at what speed. The path planner will create a spline function 's()' based on these last two points plus 3 additional points spaced 30m apart in the s frenet direction and in the lane the behavior planner has stated to go: {'previous', 'current', 30m, 60m, 90m}. The next step is to select from this spline function points appropiately spaced so that it matches once again the speed the behavior planner has stated to go. These appropiately spaced chosen points should be added to the trajectory vector in (x,y) map format. To make things easier, instead of spacing the curvy path of the spline directly, the linear distance from the 'current' (x,y) point to a target point, say (30,s(30)), is spaced to the correct speed. This linear distance spacing or number of divisions, N, is applied to the x component of the target point: 30/N. Therefore the next three points after 'current' (x,y) that are along the spline function and follow the desired speed will be: {(x+30/N, s(x+30/N)),(x+2\*30/N, s(x+2\*30/N)),(x+3\*30/N, s(x+3\*30/N)),...} 

![alt text][image6]

The 2 green dots (the last two points from the previous trajectory vector) represent the previous and current points needed to calculate the red spline function. d is the distance between the current and the target point. From d the number of divisions N can be derived to generate the purple waypoints along the spline.  

NOTE: If the number of divisions N is small then trajectory waypoints are far apart which means very high speed. If N is big then trajectory waypoints are very close to each other which means very low speed.

## Extra Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

3. If you want to switch lanes just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint. If you would like to be in the middle lane, add the waypoint's coordinates to the d vector multiplied by 6 = (2+4), since the center of the middle lane is 4 m from the center of the left lane. A helper function, getXY,  has been included which takes in Frenet (s,d) coordinates and transforms them to (x,y) coordinates.

4. For this project speed increments are in ±0.1 [m/s]. This is equivalent to an acceleration of ±5 [m/s2] and jerk of ±10 [m/s3]