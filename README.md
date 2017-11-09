# CarND2-P5 MPC Control

## Description

**This my 5th project result of Udacity self-driving car nanodegree (CarND) term 2. It's required to implement a model predictive control (MPC) of a vehicle. A vehicle simulator is provided to validate performance of MPC.**

**The following demonstrates designed MPC controlled vehicle moving in simulator :** 

![alt text][image1]

* Udacity self-driving car nanodegree (CarND) :

  https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
  
* Udacity self-driving car nanodegree term 2 simulator :

  https://github.com/udacity/self-driving-car-sim/releases/

[//]: # (Image References)
[image1]: ./images/mpc.gif


**File structure:**

* The MPC setup and interaction between simulator and MPC are implemented in C++ file `./src/main.cpp`.

* The MPC controller is implemented in C++ file `./src/MPC.cpp`.

## Usage
* `mkdir build` 
* `cd build`
* `cmake ..`
* `make`
* `./mpc`
* Download simulator `term2_sim.app` (if in OSX) and open it. Click `play!` bottom, select Project 5: MPC Controller to start.

## MPC control

The model predictive control (MPC) reframes the task of following a trajectory as an optimization problem. It simulates different actuator inputs, predicts resulting trajectory based on vehicle model and finally selects the trajectory with minimum cost.

The vehicle model used in this project is kinematic bicycle model :

    x(t+1) = x(t) + v(t) * cos(psi(t)) * dt
    y(t+1) = y(t) + v(t) * sin(psi(t)) * dt
    psi(t+1) = psi(t) + v(t) / Lf * delta(t) * dt
    v(t+1) = v(t) + a(t) * dt
    cte(t+1) = [f(x(t)) - y(t)] + v(t) * sin(epsi(t)) * dt
    epsi(t+1) = [psi(t) - psides(t)] + v(t) / Lf * delta(t) * dt

Where `x` and `y` are position of vehicle, `psi` is heading, `v` is velocity, `cte` is cross track error, `epsi` is heading error, `delta` is steering, `a` is throttle, `Lf` is the distance between vehicle front and ceter of gravity.

The MPC cost is firstly set by values used in Self-Driving Car Project Q&A | MPC controller Youtube video and tuned by trial and error : 

    fg[0] = 0;  //cost

    for (int t = 0; t < N; t++){
      fg[0] += 2000 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += 2000 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += 1 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    //minimize steer, throttle
    for (int t = 0; t < N - 1; t++){
      fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2); //5
      fg[0] += 100 * CppAD::pow(vars[a_start + t], 2); //5
    }

    //minimize change of steer, throttle
    for (int t = 0; t < N - 2; t++){
      fg[0] += 500000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);  //200
      fg[0] += 5000 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2); //100
    }

## Polynomial Fitting and MPC Preprocessing

For simplification, coordinates of waypoints are transformed in vehicle coordinates by shifting origin to vehicle current position and rotate heading to x axis:

    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;
    ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
    ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));

Where `ptsx[i]` and `ptsy[i]` are coordinates of i-th waypoints 

The MPC trajectory fitting model used is 3rd order polynomial :

    AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
    AD<double> psides0 = CppAD::atan(3 * coeffs[3] * CppAD::pow(x0, 2) + 2 * coeffs[2] * x0 + coeffs[1]);


## Model Predictive Control with Latency

In real case, actuators act with delay when actuation signal take action because of mechanism or electrical response delay. If the control system doesn't take it into consideration, this will result in late response to the environmental change.

To deal with latency, future predicted state based on vehicle model is estimated : 

    double delay = 0.1; //delay for 100ms
    double x_d = x_i + ( v * cos(psi_i) * delay );
    double y_d = y_i + ( v * sin(psi_i) * delay );
    double psi_d = psi_i - ( v * delta / Lf * delay );
    double v_d = v + a * delay;
    double cte_d = cte_i + ( v * sin(epsi_i) * delay );
    double epsi_d = epsi_i - ( v * atan(coeffs[1]) / Lf * delay);
    Eigen::VectorXd state(6);
    state << x_d, y_d, psi_d, v_d, cte_d, epsi_d;

Then MPC actuator value is solved based on the predicted state : 

    auto vars = mpc.Solve(state, coeffs);
    // vars[0] : "optimal steer value solved in MPC"
    // vars[1] : "optimal throttle value solved in MPC"

By applying actuators input that based on future predicted state, the delay problem can be lessened.

## Timestep Length and Elapsed Duration (N & dt) Setup

Firstly, I choose N = 20, dt = 0.1 because it can model a longer time span at high speed (100km/h). Besides the elapsed duration dt is chosen 0.1 because it's the same as given delay and seems to be a good starting point. After testing, I found this setting corresponding to too long MPC trajectory and caused some unstable oscillation behavior. Then I try to reduce N to 10 and also tune the cost, especially cost of steer and and change of steer. After several tests, the vehicle can drive a lap around the track successfully.



