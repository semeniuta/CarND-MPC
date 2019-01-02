#ifndef MPC_H
#define MPC_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "Controller.h"
#include "Localizer.h"

struct OptResult {
  double cost;
  std::vector<double> variables;

};

struct MPCConfig {
  size_t N_;
  double dt_;
  double vref_;

  size_t nvars_;
  size_t nconstraints_;

  size_t x_start_;
  size_t y_start_;
  size_t psi_start_;
  size_t v_start_;
  size_t cte_start_;
  size_t epsi_start_;
  size_t delta_start_;
  size_t a_start_;

  MPCConfig() = delete;

  MPCConfig(size_t N, double dt, double vref)
      : N_{N},
        dt_{dt},
        vref_{vref} {

    nvars_ = N_ * 6 + (N_ - 1) * 2;
    nconstraints_ = N_ * 6;

    x_start_ = 0;
    y_start_ = x_start_ + N_;
    psi_start_ = y_start_ + N_;
    v_start_ = psi_start_ + N_;
    cte_start_ = v_start_ + N_;
    epsi_start_ = cte_start_ + N_;
    delta_start_ = epsi_start_ + N_;
    a_start_ = delta_start_ + N_ - 1;

  }

  std::vector<double> getNextVariables(const OptResult res) const {

    std::vector<double> next{res.variables[x_start_ + 1],   res.variables[y_start_ + 1],
                             res.variables[psi_start_ + 1], res.variables[v_start_ + 1],
                             res.variables[cte_start_ + 1], res.variables[epsi_start_ + 1],
                             res.variables[delta_start_],   res.variables[a_start_]};

    return next;
  }

};

OptResult solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const MPCConfig& conf);

class MPCContoller : public Controller {

public:

  explicit MPCContoller(const Localizer& localizer) :localizer_{localizer} { }

  ControllerResult activate(double x, double y, double psi, double v) override {

    localizer_.activate(x, y, psi, v);

    MPCConfig conf{25, 0.05, 0.2};
    OptResult opt_res = solve(localizer_.state_, localizer_.poly_coeffs_, conf);

    ControllerResult res{};
    res.steer_value = opt_res.variables[conf.delta_start_ + 1];
    res.throttle_value = opt_res.variables[conf.a_start_ + 1];

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

    MPCConfig conf{25, 0.05, 0.2};
    OptResult opt_res = solve(state, coeffs, conf);

    ControllerResult res{};
    res.steer_value = opt_res.variables[conf.delta_start_ + 1];
    res.throttle_value = opt_res.variables[conf.a_start_ + 1];

    return res;

  }

private:

  WaypointsMap waypoints_;

};

#endif /* MPC_H */
