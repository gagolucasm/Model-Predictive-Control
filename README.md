# Model Predictive Controller

## Model

A simple Kinematic model (ignores tire forces, gravity, mass, etc) was used for the Controller. The state variables are:

#### State Variables
1. `X` The X position of the car.
2. `Y` The Y position of the car (Z in Unity).
3. `ψ` The current steering angle in radians.
4. `v` The current velocity in mph.

The state vector is: [x,y,ψ,v]

#### Actuator Variables

1. `δ` The steering angle. 
2. `a` This is the acceleration, in range `[-1,1]`.

Actuators: [δ,a]

#### Error Variables
Note: We also added the following to our state representation (although technically not part of the state)

1. `cte` The cross track error is the difference between our desired position and actual position.
2. `eψ` The orientation error is the difference beween our desired heading and actual heading.


#### Update equations

The state and errors update equations are below.
```
X' = X + v * cos(psi) * dt
Y' = Y + v * sin(psi) ( dt)
ψ' = ψ + v / Lf * delta * dt
v' = v + a * dt

cte' = cte - v * sin(epsi) * dt
eψ' = eψ +  v / Lf * delta * dt
```
Lf measures the distance between the front of the vehicle and its center of gravity and was provided by Udacity.


## Timestep Length and Elapsed Duration (N & dt)

* `N` The number of timesteps to predict
* `dt` The time between actuations. 
* `T = N*dt` The total time into the future to predict.

Just like in the office hours video, `N` was set to 10 and `dt` to  0.1. `T` is 1 second. A longer value was tried without great results, and reducing `dt` increase the computational power in a way that latency was a problem. Previous values were `N=20 dt=.05` and  `N=20 dt=.1`. Tuning was manual.


## Polynomial Fitting and MPC Preprocessing

The simulator sends global values, but transforming them to local reduces the complexity of the problem:
```
          // transform way points to be from car's perspective
          for (int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            waypoints_x.push_back(dx * cos(-psi) - dy * sin(-psi));
            waypoints_y.push_back(dx * sin(-psi) + dy * cos(-psi));
          }

```


Then a third order polynomial was fitted.

## Model Predictive Control with Latency

Adding latency will consist in shifting the first state of all the calculation that value in the future. It helps to reduce negative effects of the latency and increase stability of the controller. The latency was introduced to simulate the real delay of physical actuators.

## Tunning
The most difficult part of this project was the cost, weight tunning for each value. With fine tuning the car could go faster without going out the road. In this control, higher weights were set to avoid too much action and reduce the position and orientation error, but the speed was not a priority. This gives us a very secure behaviour, following the trajectory almost perfectly at a constant speed of 45 MPH.


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
