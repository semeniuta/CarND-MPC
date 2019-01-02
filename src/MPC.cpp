#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "model.h"

using CppAD::AD;

struct FG_eval {

  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs_;
  MPCConfig conf_;

  FG_eval(const Eigen::VectorXd& coeffs, const MPCConfig& conf)
  : coeffs_{coeffs},
    conf_{conf}
    { }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {

    // `fg` is a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // The cost is stored is the first element of `fg`
    // Any additions to the cost should be added to `fg[0]`
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < conf_.N_; t++) {
      fg[0] += CppAD::pow(vars[conf_.cte_start_ + t], 2);
      fg[0] += CppAD::pow(vars[conf_.epsi_start_ + t], 2);
      fg[0] += CppAD::pow(vars[conf_.v_start_ + t] - conf_.vref_, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < conf_.N_ - 1; t++) {
      fg[0] += CppAD::pow(vars[conf_.delta_start_ + t], 2);
      fg[0] += CppAD::pow(vars[conf_.a_start_ + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < conf_.N_ - 2; t++) {
      fg[0] += CppAD::pow(vars[conf_.delta_start_ + t + 1] - vars[conf_.delta_start_ + t], 2);
      fg[0] += CppAD::pow(vars[conf_.a_start_ + t + 1] - vars[conf_.a_start_ + t], 2);
    }

    fg[1 + conf_.x_start_] = vars[conf_.x_start_];
    fg[1 + conf_.y_start_] = vars[conf_.y_start_];
    fg[1 + conf_.psi_start_] = vars[conf_.psi_start_];
    fg[1 + conf_.v_start_] = vars[conf_.v_start_];
    fg[1 + conf_.cte_start_] = vars[conf_.cte_start_];
    fg[1 + conf_.epsi_start_] = vars[conf_.epsi_start_];

    for (unsigned int t = 1; t < conf_.N_; t++) {

      AD<double> x1 = vars[conf_.x_start_ + t];
      AD<double> y1 = vars[conf_.y_start_ + t];
      AD<double> psi1 = vars[conf_.psi_start_ + t];
      AD<double> v1 = vars[conf_.v_start_ + t];
      AD<double> cte1 = vars[conf_.cte_start_ + t];
      AD<double> epsi1 = vars[conf_.epsi_start_ + t];

      AD<double> x0 = vars[conf_.x_start_ + t - 1];
      AD<double> y0 = vars[conf_.y_start_ + t - 1];
      AD<double> psi0 = vars[conf_.psi_start_ + t - 1];
      AD<double> v0 = vars[conf_.v_start_ + t - 1];
      AD<double> a0 = vars[conf_.a_start_ + t - 1];
      AD<double> delta0 = vars[conf_.delta_start_ + t - 1];
      AD<double> epsi0 = vars[conf_.epsi_start_ + t - 1];

      fg[1 + conf_.x_start_ + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * conf_.dt_);
      fg[1 + conf_.y_start_ + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * conf_.dt_);
      fg[1 + conf_.psi_start_ + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * conf_.dt_);  // NOTE: In Unity coordinates: psi0 - ...
      fg[1 + conf_.v_start_ + t] = v1 - (v0 + a0 * conf_.dt_);

      AD<double> f0 = coeffs_[0] + coeffs_[1] * x0 + coeffs_[2] * x0 * x0 + coeffs_[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs_[1] + 2 * coeffs_[2] * x0 + 3 * coeffs_[3] * x0 * x0);

//      AD<double> f0 = coeffs_[0] + coeffs_[1] * x0;
//      AD<double> psides0 = CppAD::atan(coeffs_[1]);

//      for (unsigned int i = 1; i < coeffs_.size(); i++) {
//
//      }

      fg[1 + conf_.cte_start_ + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * conf_.dt_);
      fg[1 + conf_.epsi_start_ + t] = epsi1 - (psi0 - psides0 + (v0 / Lf) * delta0 * conf_.dt_);
      // NOTE: In Unity coordinates: psides0 - ...

    }

  }

};

OptResult solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, const MPCConfig& conf) {

  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars{conf.nvars_};
  for (int i = 0; i < conf.nvars_; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[conf.x_start_] = x;
  vars[conf.y_start_] = y;
  vars[conf.psi_start_] = psi;
  vars[conf.v_start_] = v;
  vars[conf.cte_start_] = cte;
  vars[conf.epsi_start_] = epsi;

  Dvector vars_lowerbound(conf.nvars_);
  Dvector vars_upperbound(conf.nvars_);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < conf.delta_start_; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = conf.delta_start_; i < conf.a_start_; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = conf.a_start_; i < conf.nvars_; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(conf.nconstraints_);
  Dvector constraints_upperbound(conf.nconstraints_);
  for (int i = 0; i < conf.nconstraints_; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[conf.x_start_] = x;
  constraints_lowerbound[conf.y_start_] = y;
  constraints_lowerbound[conf.psi_start_] = psi;
  constraints_lowerbound[conf.v_start_] = v;
  constraints_lowerbound[conf.cte_start_] = cte;
  constraints_lowerbound[conf.epsi_start_] = epsi;

  constraints_upperbound[conf.x_start_] = x;
  constraints_upperbound[conf.y_start_] = y;
  constraints_upperbound[conf.psi_start_] = psi;
  constraints_upperbound[conf.v_start_] = v;
  constraints_upperbound[conf.cte_start_] = cte;
  constraints_upperbound[conf.epsi_start_] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval{coeffs, conf};

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  OptResult result{};
  result.cost = cost;
  for (unsigned int i = 0; i < conf.nvars_; i++) {
    result.variables.push_back(solution.x[i]);
  }

  return result;

}
