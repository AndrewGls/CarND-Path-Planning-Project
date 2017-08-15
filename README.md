# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[image1]: ./report/img/clr-2.png "IMG-CLR-2"

The goal of this project is to safely navigate a car around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit using a path planning algorithm. The autonomous car should drive passengers with comfort meaning with limited maximum acceleration and jerk. In this project, I decided to follow the approach described in here: [Optimal Trajectory Generation For Dynamic Street Scenarious in A Frenet Frame](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/595fd482_werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame/werling-optimal-trajectory-generation-for-dynamic-street-scenarios-in-a-frenet-frame.pdf)

A Path-Planner generates a set of trajectories with different target speed, duration, and target driving lanes. Then, a cost function is calculated for every trajectory and the trajectory with the smallest cost is passed to a vehicle controller for execution. The Path-Planner contains stack machine with two states Lane-Keeping and Lane-Changing used to control the process of trajectory generation.

![change-lane][image1]

### Coordinate System for Motion Planning ###
Motion Planning is implemented in the FRENET´ FRAME. All transformations from FRENET´ FRAME coordinates to Cartesian coordinates are supported by special class Waypoints, which loads waypoint coordinates as [x,y,s,dx,dy] values from special file, supplied to the project, and fits two bsplines x(s) and y(s) - the x and y coordinates as functions of longitudinal s-parameter.

Transformation from FRENET coordinates (s,d) to Cartesian coordinates is implemented in two steps: evaluation of splines Fx(s) and Fy(s) at the s-coordinate and adding shift on the d-value along normal to the splines:

                        [x  y]^t = [Fx(s)  Fy(s)]^t + d * [Fx'(s)  Fy'(s)]        (1)

During the first run, the position of the vehicle in Frenet space is initialized by the simulator as a point (x, y) in Cartesian space, which is then transformed to a point (s, d) Frenet space in two steps. During the first step, the s-coordinate is computed using gradient descent optimization solver:

                          Jxy(s) = sqrt( (x – Fx(s))^2 + (y – Fy(s))^2 )
                          
After that, during the second step, the (x, y) points is computed using formula (1), using d-coordinate. This Inverse transformation is added because the simulator doesn't provide the vehicle with the accurate position in Frenet space.



### Jerk Minimizing Trajectories ###
#### 1	Velocity Keeping Trajectories ####
Velocity Keeping Trajectory is the common state where the car spends most of the time, trying to follow the leading car as close as possible with keeping speed limit and safety distance to it. The path for this state is generated as jerk minimizing trajectories using quintic polynomials [Optimal Trajectory Generation For Dynamic Street Scenarious in A Frenet Frame](https://www.researchgate.net/publication/224156269):

                          s(t) = a0+ a1*t + a2*t^2 + a3*t^3 +a4*t^4
                          s’(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3               (2)
                          s’’(t) = 2*a2 + 6*a3*t +12*a4*t^2
                          
The boundary conditions for the current state at the star t=0 of the trajectory are the longitudinal position s0, velocity s0_dot and acceleration s0_dot_dot. Using the start condition, the following three parameters can be found as:

                          s0 = a0
                          s0_dot = a1                                             (3)
                          s0_dot_dot = 2*a2
                          
The boundary conditions at the end t=T of the trajectory are the longitudinal velocity s1_dot and acceleration s1_dot_dot, which is commonly zero. Using (2) and (3), the other a3 and a4 parameters can be calculated using the following linear equation system:

                          s1_dot =s0_dot + s0_dot_dot * T + 3 * a3 * T^2 + 4*a4*T^3
                          s1_dot_dot = s0_dot_dot + 6 * a3 * T + 12 * a4 * T^2
Or using matrix form:
                          | s1_dot – s0_dot – s0_dot_dot * T |   | 3T^2   4T^3 |   | a3 |
                          |                                  | = |             | x |    |
                          |    s1_dot_dot – s0_dot_dot       |   | 6T    12T^2 |   | a4 |

The boundary conditions at the start and end states for lateral component are: d0=d1=d, d0_dot=d1_dot=0 and d0_dot_dot=d1_dot_dot=0

#### 2 Lane Changing Trajectories ####
This type of trajectory is generated as a combination of  quartic polynomial trajectory, for the longitudinal component s, using Velocity Keeping Trajectories, with  a quintic polynomial for the lateral component d, using the following linear equation system:

      | d1 – d0 – d0_dot – 0.05 * d0_dot_dot * T^2 |   |  T^3    T^4    T^5 |   | a3 |
      | d1_dot – d0_dot – d0_dot_dot * T           | = | 3T^2   4T^3   5T^4 | x | a4 |
      | d1_dot_dot – d0_dot_dot                    |   | 6T    12T^2  20T^3 |   | a5 |



                        Report is in process...
                      


### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
