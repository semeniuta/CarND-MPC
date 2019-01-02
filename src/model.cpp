//
// Created by Oleksandr Semeniuta on 2018-12-30.
//

#include "model.h"

Eigen::VectorXd kinematicModel(const Eigen::VectorXd& state, const Eigen::VectorXd& actuators, double dt) {

  double x = state(0);
  double y = state(1);
  double psi = state(2);
  double v = state(3);

  double delta = actuators(0);
  double a = actuators(1);

  Eigen::VectorXd next_state{state.size()};

  next_state << x + v * cos(psi) * dt,
                y + v * sin(psi) * dt,
                psi + (v / Lf) * delta * dt, // NOTE: In Unity coordinates: psi - ...
                v + a * dt;

  return next_state;

}

