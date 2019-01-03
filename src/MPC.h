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

  std::vector<double> getNextVariables(const OptResult& res) const {

    std::vector<double> next{res.variables[x_start_ + 1],   res.variables[y_start_ + 1],
                             res.variables[psi_start_ + 1], res.variables[v_start_ + 1],
                             res.variables[cte_start_ + 1], res.variables[epsi_start_ + 1],
                             res.variables[delta_start_],   res.variables[a_start_]};

    return next;
  }

  std::vector<double> getX(const OptResult& res) const {

    std::vector<double> x;
    for (size_t i = x_start_; i < x_start_ + N_; i++) {
      x.push_back(res.variables[i]);
    }

    return x;

  }

  std::vector<double> getY(const OptResult& res) const {

    std::vector<double> y;
    for (size_t i = y_start_; i < y_start_ + N_; i++) {
      y.push_back(res.variables[i]);
    }

    return y;

  }

};

class MPCController : public Controller {

public:

  MPCController() = delete;

  explicit MPCController(const MPCConfig& conf) : conf_{conf} { };

  void setWaypoints(std::vector<double> ptsx, std::vector<double> ptsy) override;

  ControllerResult activate(double x, double y, double psi, double v) override;

private:

  MPCConfig conf_;
  WaypointsMap waypoints_;

};

OptResult solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const MPCConfig& conf);

#endif /* MPC_H */
