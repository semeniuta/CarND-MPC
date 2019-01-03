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

## Tuning the controller

The MPC controller (`MPCController`) is configured with `MPCConfig` structure. It is instantiated with three required parameters: 

* `N` - number of iterations
* `dt` - time increment
*  `vref` - reference velocity

Additional fields of `MPCConfig` can be set:

* `change_a_penalty_`
* `change_delta_penalty_`

They are used in `MPCController` to penalize the change of throttle and steering respectively:

```cpp
fg[0] += conf_.change_delta_penalty_ * 
         CppAD::pow(vars[conf_.delta_start_ + t + 1] - vars[conf_.delta_start_ + t], 2);
fg[0] += conf_.change_a_penalty_ * 
         CppAD::pow(vars[conf_.a_start_ + t + 1] - vars[conf_.a_start_ + t], 2);
```

These parameters are set to the following values: `change_delta_penalty_=100, change_a_penalty_=500`. 

The controller exerts the best behavior with the following required parameters: `(N=50, dt=0.2, vref=5.)`. Other workable,  albeit less stable, configuration is `(N=30, dt=0.1, vref=10.)`. 

It is observed that for larger `vref`, it is important that the total prediction time (product `N * dt`)  is not too big. In addition, `dt`  should not be too small. Setting the penalizing factors stablizes the driving behavior.

## Latency

The 100 ms latency is dealt with in `client.cpp` by predicting the state of the vehicle using the kinematic model.