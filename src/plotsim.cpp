//
// Created by Oleksandr Semeniuta on 2019-01-02.
//

#include "plotsim.h"
#include "geometry.h"
#include "matplotlibcpp.h"
#include <algorithm>

namespace plt = matplotlibcpp;

History simulateMPC(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, int n_iters, const MPCConfig& conf) {

  History h;
  h.init(state);

  Eigen::VectorXd current_state{6};
  current_state = state;

  for (size_t i = 0; i < n_iters; i++) {

    auto res = solve(current_state, coeffs, conf);
    auto vars = conf.getNextVariables(res);

    h.update(vars);

    current_state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
  }

  return h;

}

void plot_mpc(const History& h, const Eigen::VectorXd& coeffs) {

  plt::subplot(3, 2, 1);
  plt::title("cte");
  plt::plot(h.cte);

  plt::subplot(3, 2, 2);
  plt::title("epsi");
  plt::plot(h.epsi);

  plt::subplot(3, 2, 3);
  plt::title("a");
  plt::plot(h.a);

  plt::subplot(3, 2, 4);
  plt::title("delta");
  plt::plot(h.delta);

  plt::subplot(3, 2, 5);
  plt::title("v");
  plt::plot(h.v);

  plt::subplot(3, 2, 6);
  plt::title("x, y");
  plt::plot(h.x, h.y);

  std::vector<double> fx;
  for (const double& val : h.x) {
    fx.push_back( polyeval(coeffs, val) );
  }

  plt::plot(h.x, fx, "c-");

  plt::save("dynamics.png");


}
