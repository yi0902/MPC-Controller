# Model Predictive Control Project
Self-Driving Car Engineer Nanodegree Program

---

## Description

This project aimes to implement a Model Predictive Controller (MPC) to drive a car around a track in a simulator. The simulator takes as input the steering angle and throttle to drive the car, and gives as output a bunch of car states' values including the positions of the car, its speed, throttle and heading direction, as well as the coordinates of a reference trafectory (waypoints) that the car should follow. More details of the simulator's data can be found in DATA.md.

Below the final output video. The car drove with an average speed of 50 mph.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/MELCv99jTBY&t=6s/hqdefault.jpg)](https://www.youtube.com/watch?v=MELCv99jTBY&t=6s)

## The Vehicle Model

In model used in this project is the Global Kinematic Model. It's a simplification of dynamic model that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the model, but it makes model more tractable. At low and moderate speeds, Kinematic Model often approximate the actual vehicle dynamics.

In Kinematic Model, vehicle state is represented by six variables which contain also error variables: 

- **x,y** denote the position of the vehicle
- **psi** the heading direction
- **v** the velocity of the vehicle
- **cte** the cross track error 
- **epsi** the orientation error

The update equations can be described as following:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

**Lf** measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.

**a** and **delta** denote the actuators of the vehicle model:

- **a** the acceleration/throttle of the vehicle which can take values between -1 and 1 in this project
- **delta** the steering angle of the vehicle which can take values between -25 and 25 degrees.
 
The vehicle model can be found in the class **FG_eval**.

## Timestep Length and Elapsed Duration (N & dt)

For every state value provided by the simulator, an optimal trajectory for the next N time steps is computed that minimizes a cost function. dt denotes the duration between time steps. 

The cost function is quadratic in the cross-track error, the error in the heading direction, the difference to the reference velocity, the actuator values and the difference of actuator values in adjacent time steps. The weights of each component of the cost function need to be fine tuned to smooth the driving transitions, so as the values of N and dt.

I started with (N, dt) values as (20, 0.05), the car oscillated a lot and quickly went out of the track. As there is a latency of 0.1 second in the project, I then tried dt values bigger or equal to the latency, and through lots of trial and error I got best results with dt as 0.1. And it seemed that N = 10 worked better than N = 20 for my implementation.

## Polynomial Fitting and MPC Preprocessing

I used a 3 order polynomial to fit waypoints in order to approximates the road. Roads always have curves and turn to the right and left, a polynomial of 3rd order could be able to approximate the curve of any road. 

As the wayspoints and vehicule's positions were given in global coordinates, while the simulator need vehicle's local coordinates (in the perspective of the vehicle) to plot reference trajectory and predicted path on the video, I first transformed the global coordiantes into vehicle's local coordiantes. This step also help a lot to simplify the MPC model as the new state becomes (0, 0, 0, v, cte, epsi) in vehicle's perspective.

The transformation of coordiantes and polynomial fitting would be found in **main.cpp**.

```
auto waypoints = Eigen::MatrixXd(2, len);
for (auto i = 0; i < len ; ++i){
  double x_diff = ptsx[i] - px;
  double y_diff = ptsy[i] - py;
  waypoints(0,i) = cos(psi) * x_diff + sin(psi) * y_diff;
  waypoints(1,i) = -sin(psi) * x_diff + cos(psi) * y_diff;  
} 
// Fit the polynomial to a 3 order polynomial
auto coeffs = polyfit(waypoints.row(0), waypoints.row(1), 3);
```

## Model Predictive Control with Latency

To handle with the latency, I took the approach of leaving vehicule run from actual state for the duration of latency, predicting the new state with Kinematic Model, then taking the predicted state as the new initial state for MPC. 

```
// Compute initial state for MPC by predicting the car's future state after the latency 
state[0] = v * cos(0) * latency;
state[1] = v * sin(0) * latency;
state[2] = (-v / Lf) * steer * latency;
state[3] = v + a * latency;
state[4] = cte + v * sin(epsi) * latency;
state[5] = epsi - (v / Lf) * steer * latency;
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
