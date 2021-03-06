//
// Created by Oleksandr Semeniuta on 2018-12-26.
//

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"


struct WaypointsMap {

  Eigen::VectorXd x;
  Eigen::VectorXd y;

  Eigen::VectorXd transformPoint(unsigned int i, const Eigen::MatrixXd& pose) {

    Eigen::VectorXd pt{3};
    pt << x[i], y[i], 1.;

    Eigen::VectorXd pt_transformed = pose * pt;

    Eigen::VectorXd res{2};
    res << pt_transformed(0) / pt_transformed(2),
           pt_transformed(1) / pt_transformed(2);

    return res;

  }

  WaypointsMap transformAll(const Eigen::MatrixXd& pose) {

    auto n = x.size();

    WaypointsMap t_map;
    t_map.x = Eigen::VectorXd{n};
    t_map.y = Eigen::VectorXd{n};

    for (unsigned int i = 0; i < n; i++) {

      Eigen::VectorXd v = this->transformPoint(i, pose);

      t_map.x[i] = v(0);
      t_map.y[i] = v(1);

    }

    return t_map;
  }

};

struct ClosestWaypoint {
  unsigned int index;
  double distanceToOrigin;
};

struct Line {
  double slope;
  double intercept;
};

struct Errors {
  double cte;
  double epsi;
};

double deg2rad(double x);

double rad2deg(double x);

double polyeval(const Eigen::VectorXd& coeffs, double x);

Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);

WaypointsMap readWaypoints(const std::string& filename);

Eigen::MatrixXd invertPose(const Eigen::MatrixXd& pose);

Eigen::MatrixXd createPose(double x, double y, double theta);

double distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);

ClosestWaypoint findClosestWaypoint(const WaypointsMap& map);

WaypointsMap getNextWaypoints(const WaypointsMap& points, const ClosestWaypoint& closest, int n);

Line closestLine(const WaypointsMap& points, const ClosestWaypoint& closest);

Errors errorsFromLine(const Line& line, double x0=0., double y0=0.);

Line createLine(double x1, double y1, double x2, double y2);

#endif //GEOMETRY_H
