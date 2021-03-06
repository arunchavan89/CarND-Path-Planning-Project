# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
![](attachments/result.gif)

[image1]: ./attachments/behaviour_control.PNG "behaviour_control Image"
[image2]: ./attachments/frenet1.PNG "frenet1 Image"
[image3]: ./attachments/frenet3.PNG "frenet3 Image"
[image4]: ./attachments/poly_eq_1.PNG "formula_1 Image"
[image5]: ./attachments/polyequ.PNG "polyequ Image"
[image6]: ./attachments/spline.PNG "spline Image"
[image7]: ./attachments/homo_transformation.PNG "homo_transformation Image"
[image8]: ./attachments/input.PNG "quintic Image"

## Goal
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Introduction 

### Input
* The map of the highway is in data/highway_map.txt
* Each waypoint in the list contains  [x,y,s,dx,dy] values. 
* x and y are the waypoint's map coordinate position.
* The s value is the distance along the road to get to that waypoint in meters.
* The dx and dy values define the unit normal vector pointing outward of the highway loop.
* The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

# Fundamentals

## Behaviour Control 
* An overview of the behavior control is as shown in the following Figure.
<p align="center">
  <img width="500" height="300" src="attachments/behaviour_control.PNG">
</p>

* The module is typically decomposed into the following set of sub-modules:
* Predictions: Predicting the trajectories of the surrounding detected objects.
* Behaviour Planner: Defining a set of candidate high level targets for the vehicle to follow (lane changes, slow down, emergency braking etc)
* Trajectory: Finding a percise path to drive the ego vehicle.

## Frenet and Cartesian Coordinates
* Frenet Coordinates are a way of representing position on a road in a more intuitive way than traditional '(x,y)' Cartesian Coordinates.
* The coordinates use variables 's' and 'd' to describe a vehicle's position on the road. 
* The 's' coordinate represents distance along the road also known as longitudinal displacement.
* The 'd' coordinate represents side-to-side position on the road also known as lateral displacement.

<p align="center">
  <img width="300" height="300" src="attachments/frenet3.PNG">
</p>

## Spline

* A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Polynomial solver for jerk minimization (A quintic polynomial solver) 

*  A quintic polynomial solver takes boundary conditions as input and generate a polynomial trajectory which matches those conditions with minimal jerk.

* The inputs for polynomial solver are shown in the following image. 
<p align="center">
  <img width="500" height="300" src="attachments/input.PNG">
</p>

* The following six coefficients are to be calculated
<p align="center">
  <img width="500" height="300" src="attachments/poly_eq_1.PNG">
</p>

* The equation to solve is as follows:
<p align="center">
  <img width="500" height="400" src="attachments/polyequ.PNG">
</p>

## Vehicle and Map (World) coordinates transformation
* From the world to the robot:

<p align="center">
  <img width="500" height="200" src="attachments/homo_transformation.PNG">
</p>




## Sensor Fusion

#### Main car's localization Data (No Noise)

* 'x' : car's x position in map coordinates

* 'y' : car's y position in map coordinates

* 's' : car's s position in frenet coordinates

* 'd' : car's d position in frenet coordinates

* 'yaw' : car's yaw angle in the map

* 'speed' : car's speed in MPH

* 'previous_path_x' : previous list of x points previously given to the simulator

* 'previous_path_y' : previous list of y points previously given to the simulator

* 'end_path_s' : previous list's last point's frenet s value

* 'end_path_d' : previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Algorithm :

* The following code snippet implements the logic for detecting a vehicle ahead of the ego vehicle.
* It also sets a flag for the emergency braking in case if the distance between the ego vehicle and the vehicle in front is very close.  
```
/* Check if any vehicle is ahead of the ego vehicle */
if ((d < (2 + 4 * lane + 2)) && (d > (2 + 4 * lane - 2)))
{
    if ((check_car_s > car_s) && (check_car_s - car_s) < 30)
    {
        car_ahead = true;
        speed_car_ahead = check_speed;
        if ((check_car_s > car_s) && (check_car_s - car_s) < 10)
        {
            emergency_brake = true;
        }
    }
}
```

* The following logic is implemented for changing lanes if a vehicle is ahead of the ego vehicle.
```
if (car_ahead)
{
    if ((lane == 1) && !car_lane_0)
    {
        lane--;
    }
    else if ((lane == 1) && !car_lane_2)
    {
        lane++;
    }
    else if ((lane == 2) && !car_lane_1)
    {
        lane--;
    }
    else if ((lane == 0) && !car_lane_1)
    {
        lane++;
    }
    else
    {
        if (emergency_brake)
        {
            /* Possibility of getting jerk but good for safety */
            std::cout << "Emergency Brake:" << std::endl;
            ref_vel -= emergency_braking;
        }
        else
        {
            if (ref_vel != speed_car_ahead)
            {
                ref_vel -= accleration_rate;
            }
        }
    }
}

```

* For lane changing trajectories the current implemetation uses 'spline' library.
* This library is single header file that helps to get the corresponding 'y' coordinate on the spline if 'x' coordinate is provided.
* The following diagram helps to understand the trajectory generation procedure.
<p align="center">
  <img width="500" height="400" src="attachments/spline.PNG">
</p>

* The idea is to find 'N' number of points on the spline which are spaced in a way that the vehicle moves with desired speed.
* The length of the spline can be linearlized to make it easy.
* A spline trajectory of length 30 meters is to be genearated. This 30 meter path is segmented'N' number of parts with the formula mentioned in the image.
* In addition to the previous path points the new points on the spline are calculated using the function implemented in spline.h
* Having previous path points is important to make the spline trajectory smooth.


## Result
* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes

<p align="center">
  <img width="500" height="400" src="attachments/final_result.PNG">
</p>

## Future work
* Implementation of th quintic polynomial solver
* participation in Bosch challenge after completion of the course.

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
## Installation:

### Windows:
* [step by step guide](https://medium.com/@fzubair/udacity-carnd-term2-visual-studio-2015-17-setup-cca602e0b1cd)

* One of the newest features to Windows 10 users is an Ubuntu Bash environment that works great and is easy to setup and use. Here is a nice [step by step guide](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up utilities.
