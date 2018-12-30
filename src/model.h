//
// Created by Oleksandr Semeniuta on 2018-12-30.
//

#ifndef MODEL_H
#define MODEL_H

#include "Eigen-3.3/Eigen/Core"

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

Eigen::VectorXd kinematicModel(const Eigen::VectorXd& state, const Eigen::VectorXd& actuators, double dt);

#endif //MPC_MODEL_H
