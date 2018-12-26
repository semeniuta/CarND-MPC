//
// Created by Oleksandr Semeniuta on 2018-12-26.
//

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double polyeval(const Eigen::VectorXd& coeffs, double x);

Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);

#endif //GEOMETRY_H
