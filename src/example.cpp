#include "MPC.h"
#include "geometry.h"
#include "Eigen-3.3/Eigen/Core"
#include "plotsim.h"

int main() {

  int n_iters = 50;

  Eigen::VectorXd ptsx(9);
  Eigen::VectorXd ptsy(9);
  ptsx << -100, -70, -50, 25,  0, 25, 50, 70, 100;
  ptsy << -1,    -1,  -1, -1, -1, -1, -1, -1,  -1;

  auto coeffs = polyfit(ptsx, ptsy, 3);

  double x = -1;
  double y = 10;
  double psi = 0;
  double v = 10;
  double cte = polyeval(coeffs, x) - y;
  double epsi = psi - atan(coeffs[1]);

  Eigen::VectorXd state(6);
  state << x, y, psi, v, cte, epsi;

  MPCConfig conf{25, 0.05, 40.};
  History h = simulateMPC(state, coeffs, n_iters, conf);

  plot_mpc(h);

}