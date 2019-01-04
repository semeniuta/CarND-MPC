# Model Predictive Control Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Model Predictive Control Project in Udacity's Self-Driving Car Nanodegree. The original project template can be found [here](https://github.com/udacity/CarND-MPC-Project).

## Dependencies

The C++ code depends on the following libraries:

* uWebSockets
* Ipopt
* CppAD
* Eigen [bundled in the project]
* Matplotlibcpp [bundled in the project]

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). 

## The model

The model used in this project constitutes a kinematic model of a vehicle. It operates on three classes of variables:

* state: `x, y, psi, v`
* controls: `delta, a`
* errors: `cte, epsi`

The model updates the state and errors based on the values on controls, the time increment `dt`, and constant `Lf` (specific to the vehicle in the simulator). 

```
(1) x_next = x + v * cos(psi) * dt
(2) y_next = y + v * sin(psi) * dt
(3) psi_next = psi + (v / Lf) * delta * dt
(4) v_next = v + a * dt
(5) cte_next = [f(x) - y] + v * sin(epsi) * dt
(6) epsi_next = [psi - psides] + (v / Lf) * delta * dt
```

`f(x)` constitutes a polynomial estimated from waypoints. In this project, the 3rd degree polynomial is considered:

```
f(x) = a3 * x**3 + a2 * x**2 + a1 * x + a0
```

On each telemetry message from the simulator, the state of the vehicle is read (`x, y, psi, v`). Given (`x, y`) and a list of waypoints, the errors are measured with respect to a line formed by the first two waypoints (see function `errorsFromLine` in `geometry.cpp`). 

After receiving the original state of the vehicle, its position and orientation after 100 ms is predicted using equations (1)-(4) of the kinematic model (to account for latency). These position and orientation (`x, y, psi`) are used to define the vehicle's coordinate frame, with respect to which all waypoints are transformed (see functions `createPose` and `invertPose` in `geometry.cpp`).

## Tuning the controller

The MPC controller (`MPCController`) is configured with `MPCConfig` structure. It is instantiated with three required parameters: 

* `N` - number of iterations
* `dt` - time increment
*  `vref` - reference velocity

Additional fields of `MPCConfig` can be set:

* `change_a_penalty_`
* `change_delta_penalty_`

They are used in the optimization routine in `MPC.cpp` to penalize the change of throttle and steering respectively:

```cpp
fg[0] += conf_.change_delta_penalty_ * 
         CppAD::pow(vars[conf_.delta_start_ + t + 1] - vars[conf_.delta_start_ + t], 2);
fg[0] += conf_.change_a_penalty_ * 
         CppAD::pow(vars[conf_.a_start_ + t + 1] - vars[conf_.a_start_ + t], 2);
```

These parameters are set to the following values: `change_delta_penalty_=100, change_a_penalty_=500`. 

The controller exerts the best behavior with the following required parameters: `(N=50, dt=0.2, vref=5.)`. Other workable,  albeit less stable, configuration is `(N=30, dt=0.1, vref=10.)`. 

It is observed that for larger `vref`, it is important that the total prediction time (product `N * dt`)  is not too big. In addition, `dt`  should not be too small. Setting the penalizing factors stablizes the driving behavior.