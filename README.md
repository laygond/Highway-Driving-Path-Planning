# Highway Path Planning
The goal is to safely navigate around a virtual highway with other vehicles which are changing lanes while driving +-10 MPH of the 50 MPH speed limit. The ego car is provided a sparse map, and its location along with sensor fusion data to estimate the location of all the vehicles on the same side of the road. Behavioral planning and trajectory planning are used to create smooth, safe trajectories. This involves driving as close as possible to the 50 MPH speed limit, passing slower traffic when possible, avoid hitting other cars, driving inside of the marked road lanes at all times, and not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3. This repo uses [Udacity's CarND-Path-Planning-Project repo](https://github.com/udacity/CarND-Path-Planning-Project) as a base template and guide.

<p align="center"> 
  <img src="./README_images/path_planning.gif">
</p>

[//]: # (List of Images used in this README.md)
[image1]: ./README_images/highway_sim.png "Simulator"
[image2]: ./README_images/pid_graph.png "PID Diagram"
[image3]: ./README_images/p_control.gif "P Controller"
[image4]: ./README_images/pd_control.gif "PD Controller"
[image5]: ./README_images/drift.png "Drift"
[image6]: ./README_images/pd_drift.gif "PD + Drift"
[image7]: ./README_images/pid_drift.gif "PID + Drift"

## Directory Structure
```
.PID-Controller
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

Path planning is an ambigous term that requires clarification for incomers. In this project we will deal with the Planning Subsystem. This means that information from the Perception Subsystem is already provided. For the Control subsystem the car uses a perfect controller and will visit every (x,y) point it recieves from the Planning subsystem.

The Planning subsystem is formed by other modules
- Route or Mission Planning
- Prediction
- Behavioral Planning
- Trajectory Planning (Path Planner + Scheduler)

<b>Path Planner:</b> refers only to the geometrical waypoints of a path with no information about time. A velocity profile or scheduler is needed to tell the ego car when it should be at each waypoint. Combining the path planner with the scheduler makes up the trajectory planner. Due to historical reason the term path planning is used interchangeably to relate to the entire planning subsystem, hence, causing confusion. 

In this project the scheduler is fixed to .02 seconds. The car will visit every (x,y) point it receives in the list every .02 seconds. Therefore the objective in this module is to only focus on the path creation for the car to safely navigate and the distancing between between waypoints to control the speed and prevent jerk.


### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 

The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Perception Subsystem 
#### The map of the highway is in data/highway_map.csv
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

Every turn the car sets a goal position and a ref_vel.
To reach that goal at that ref_vel, the path to the goal should be spaced accordingly.
The distance to target may be divided into just a few points(very high speed) or several points (very low speed).
In general there will always be more than 50 point divisions to target (say 200) since we make sure we do not go above speed limit.
Regardless of number of point divisions to target, the car makes sure it has 50 points in its vector to travel.
This 50 points are supposed to be travelled within 1 second (.02 seconds each point), however, a loop in a simulator takes less than a second.
Hence, just a portion of those 50 points are crossed. 
The remaining points in the vector are transferred to the vector of the next loop.
The next loop vector then gets filled until it reaches 50 with new points that obey this next loop's goal position and a ref_vel.
The process then repeats.

