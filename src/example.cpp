#include "MPC.h"
#include "geometry.h"
#include "Eigen-3.3/Eigen/Core"
#include "plotsim.h"

void example_line() {

  int n_iters = 50;

  Eigen::VectorXd ptsx(9);
  Eigen::VectorXd ptsy(9);
  ptsx << -100, -70, -50, 25,  0, 25, 50, 70, 100;
  ptsy <<   -1,  -1,  -1, -1, -1, -1, -1, -1,  -1;

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

  plot_mpc(h, coeffs);

}

void example_track() {

  int n_iters = 100;

  Eigen::VectorXd ptsx(6);
  Eigen::VectorXd ptsy(6);
  ptsx << -32.16173,-43.49173,-61.09,-78.29172,-93.05002,-107.7717;
  ptsy << 113.361,105.941,92.88499,78.73102,65.34102,50.57938;
  WaypointsMap pts{ptsx, ptsy};

  double x = -40.62;
  double y = 108.73;
  double psi = 3.733651;

  auto vehicle_pose = createPose(x, y, psi);
  auto t_map_to_vehicle = invertPose(vehicle_pose);
  auto wp = pts.transformAll(t_map_to_vehicle);

  Eigen::VectorXd coeffs = polyfit(wp.x, wp.y, 3);
  std::cout << "Coefficients:\n" << coeffs << std::endl;

  auto closest = findClosestWaypoint(wp);
  Line closest_line = closestLine(wp, closest);

  Errors err = errorsFromLine(closest_line);
  std::cout << "cte=" << err.cte << std::endl;
  std::cout << "epsi=" << err.epsi << std::endl;

  Eigen::VectorXd state{6};
  state << 0, 0, 0, 0, err.cte, err.epsi;

  MPCConfig conf{50, 0.2, 5.};
  History h = simulateMPC(state, coeffs, n_iters, conf);

  plot_mpc(h, coeffs);

}

int main() {

  example_track();

}