#ifndef MPC_H
#define MPC_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "Controller.h"
#include "Localizer.h"

// Timestep length and duration
const size_t MPC_N = 25;
const double MPC_dt = 0.05;

const size_t MPC_nvars = MPC_N * 6 + (MPC_N - 1) * 2;
const size_t MPC_nconstraints = MPC_N * 6;

// Start indices in the variables' vector
const size_t x_start = 0;
const size_t y_start = x_start + MPC_N;
const size_t psi_start = y_start + MPC_N;
const size_t v_start = psi_start + MPC_N;
const size_t cte_start = v_start + MPC_N;
const size_t epsi_start = cte_start + MPC_N;
const size_t delta_start = epsi_start + MPC_N;
const size_t a_start = delta_start + MPC_N - 1;

const double MCP_vref = 0.2;

struct OptResult {
  double cost;
  std::vector<double> variables;
};

OptResult solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);

class MPCContoller : public Controller {

public:

  explicit MPCContoller(const Localizer& localizer) :localizer_{localizer} { }

  ControllerResult activate(double x, double y, double psi, double v) override {

    localizer_.activate(x, y, psi, v);

    OptResult opt_res = solve(localizer_.state_, localizer_.poly_coeffs_);

    ControllerResult res{};
    res.steer_value = opt_res.variables[delta_start + 1];
    res.throttle_value = opt_res.variables[a_start + 1];

    return res;

  }

private:

  Localizer localizer_;

};

class NewMPC : public Controller {

public:

  void setWaypoints(std::vector<double> ptsx, std::vector<double> ptsy) override {

    waypoints_.x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
    waypoints_.y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

  }

  ControllerResult activate(double x, double y, double psi, double v) override {

    auto vehicle_pose = createPose(x, y, psi);
    auto t_map_to_vehicle = invertPose(vehicle_pose);
    auto wp = waypoints_.transformAll(t_map_to_vehicle);

    Eigen::VectorXd coeffs = polyfit(wp.x, wp.y, 3);
    std::cout << "Coefficients:\n" << coeffs << std::endl;

    auto closest = findClosestWaypoint(wp);
    Line closest_line = closestLine(wp, closest);

    Errors err = errorsFromLine(closest_line);
    std::cout << "cte=" << err.cte << std::endl;
    std::cout << "epsi=" << err.epsi << std::endl;

    Eigen::VectorXd state{6};
    state << 0, 0, 0, v, err.cte, err.epsi;

    OptResult opt_res = solve(state, coeffs);

    ControllerResult res{};
    res.steer_value = opt_res.variables[delta_start + 1];
    res.throttle_value = opt_res.variables[a_start + 1];

    return res;

  }

private:

  WaypointsMap waypoints_;

};

#endif /* MPC_H */
