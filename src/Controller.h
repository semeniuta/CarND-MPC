//
// Created by Oleksandr Semeniuta on 2018-12-29.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>

struct ControllerResult {

  double steer_value;
  double throttle_value;

  std::vector<double> mpc_x_vals;
  std::vector<double> mpc_y_vals;

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

};

class Controller {

public:

  virtual ControllerResult activate(double x, double y, double psi, double v) = 0;
  virtual void setWaypoints(std::vector<double> ptsx, std::vector<double> ptsy) = 0;

};

#endif //CONTROLLER_H
