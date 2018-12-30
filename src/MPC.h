#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// Timestep length and duration
const size_t MPC_N = 25;
const double MPC_dt = 0.05;

// Start indices in the variables' vector
const size_t x_start = 0;
const size_t y_start = x_start + MPC_N;
const size_t psi_start = y_start + MPC_N;
const size_t v_start = psi_start + MPC_N;
const size_t cte_start = v_start + MPC_N;
const size_t epsi_start = cte_start + MPC_N;
const size_t delta_start = epsi_start + MPC_N;
const size_t a_start = delta_start + MPC_N - 1;

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
const double MPC_Lf = 2.67;

const double MCP_vref = 40;

struct OptResult {
  double cost;
  std::vector<double> variables;
};

OptResult solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);

#endif /* MPC_H */
