# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Reflections

#### UPDATE AFTER 1st SUBMISSION
For the next video, I didn't modify my code  from previous submission and the car is able to finish the track without leaving drivable portion of the surface. https://www.youtube.com/watch?v=i0a8GYFxHoo&feature=youtu.be

Also, I added state correction for latency as suggested by previous reviewer. It didn't seem to increase performance, so I disabled it by default and didn't record a video. It can be switched using flag `IS_LATENCY_CORRECTION` in `main.cpp` line 127.  
As I wrote in previous submission, latency didn't affect driving, so I didn't implement special correction for it in my previous solution.
As `dt` in my solution equals to `latency_t`, I approximated steering value and throttle value by previous values that I keep in `mpc.prev_throttle` and `mpc.prev_steer` , instead of extraction from json data as suggested by previous reviewer.
 
#### The Model
I implemented Kinematic model for driving the car in Udacity Simulator.
 The model keeps track of and update state vector, which contains: current global coordinates (`x` and `y`), vehicle global orientation `psi`, speed `v` as well as CTE `cte` and error of orientation `epsi`.
 These values are updated every 0.1 seconds (I set dt = 0.1) according to constraints and kinematic equations implemented using Ipopt and CppAD libraries.
 
The car moves according to actuators values (steering angle `delta` and acceleration `a`) which are also updated each timestep.

The simulator provides waypoints which are used to calculate optimal trajectory.

Optimal actuators' values are calculated in `MPC::Solve` function in `MPC.cpp` file.

Update state equations I used:

```
x_1   = x_0 + v_0 * cos(psi) * dt
y_1   = y_0 + v_0 * sin(psi) * dt
psi_1 = psi_0 + v_0 / Lf * delta * dt
v_1   = v_0 + a_0 * dt
```
where `Lf` is the distance between the front of the car and its center of gravity. The larger the vehicle, the slower its turn rate.

#### Timestep Length and Elapsed Duration (N & dt)
I used 0.1 instead of 0.05 (like in Lessons) to make computation less expensive, because I thought that such a resolution can give sufficient results. I also chose number of timesteps `N = 10` as a starting point. During optimisation of the cost these values allowed me to create the model that performs well, so I kept these values.
These values allow to model next `N * dt = 10 * 0.1 = 1` second of future movement during each optimisation step.
This was enough for the model to perform well. 
 
More timesteps is not necessarily better, because the situation can change very fast and further prediction are probably much less relevant but require computational resources.

On the other hand, smaller `dt` could increase performance in more complicated road situation, but to compensate for smaller elapsed duration between timestamps, we would need to increase `N` which would computation more expensive.
   
#### Polynomial Fitting and MPC Preprocessing

Before fitting a polynomial to waypoints provided by simulator, I converted them to vehicle's coordinate system. After that transformed arrays were used to fit the third degree polynomial. 3 degrees of freedom are generally enough for most roads. 
Since I converted `x`, `y` and `psi` to vehicle coordinate system, all these values (which represent car position) became 0.

#### Model Predictive Control with Latency
Latency is a delay between execution of actuator and response from the car. Real car's latency can be as big as 100 ms. The same latency was modeled in this project. In my implementation the latency didn't really affect driving. So I didn't address this issue.

If latency is the issue, then one can use vehicle's movement model and predict vehicle's state starting from the current time for the duration of the latency, an use this new state as initial state for MPC.

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