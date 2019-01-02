//
// Created by Oleksandr Semeniuta on 2019-01-02.
//

#ifndef PLOTSIM_H
#define PLOTSIM_H

#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"

struct History {

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> psi;
  std::vector<double> v;
  std::vector<double> cte;
  std::vector<double> epsi;
  std::vector<double> delta;
  std::vector<double> a;

  void init(const Eigen::VectorXd& state) {

    x.push_back(state(0));
    y.push_back(state(1));
    psi.push_back(state(2));
    v.push_back(state(3));
    cte.push_back(state(4));
    epsi.push_back(state(5));

  }

  void update(const std::vector<double>& vars) {

    x.push_back(vars[0]);
    y.push_back(vars[1]);
    psi.push_back(vars[2]);
    v.push_back(vars[3]);
    cte.push_back(vars[4]);
    epsi.push_back(vars[5]);
    delta.push_back(vars[6]);
    a.push_back(vars[7]);

  }

};

void plot_mpc(const History& h);

History simulateMPC(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, int n_iters, const MPCConfig& conf);


#endif //PLOTSIM_H
