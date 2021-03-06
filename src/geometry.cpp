//
// Created by Oleksandr Semeniuta on 2018-12-26.
//

#include "geometry.h"
#include <string>
#include <fstream>
#include <vector>
#include <limits>

double deg2rad(double x) {
  return x * M_PI / 180;
}

double rad2deg(double x) {
  return x * 180 / M_PI;
}

// Evaluate a polynomial.
double polyeval(const Eigen::VectorXd& coeffs, double x) {

  double result = 0.0;

  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }

  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order) {

  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;

}

WaypointsMap readWaypoints(const std::string& filename) {

  std::ifstream in_file(filename.c_str(), std::ifstream::in);

  if (!in_file.is_open()) {
    throw std::runtime_error("Cannot open input file");
  }

  std::vector<double> x_vec;
  std::vector<double> y_vec;

  std::string line;
  double x;
  double y;

  while(getline(in_file, line)){

    std::istringstream iss{line};
    iss >> x;
    iss >> y;

    x_vec.push_back(x);
    y_vec.push_back(y);

  }

  WaypointsMap map;
  map.x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(x_vec.data(), x_vec.size());
  map.y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(y_vec.data(), y_vec.size());

  return map;

}

Eigen::MatrixXd createPose(double x, double y, double theta) {

  double ct = cos(theta);
  double st = sin(theta);

  Eigen::MatrixXd pose{3, 3};

  pose << ct, -st, x,
          st,  ct, y,
           0,   0, 1;

  return pose;

}

Eigen::MatrixXd invertPose(const Eigen::MatrixXd& pose) {

  //Eigen::MatrixXd pose_inv = pose.inverse();

  Eigen::MatrixXd pose_inv{3, 3};

  double ct = pose(0, 0);
  double st = pose(1, 0);

  double x = pose(0, 2);
  double y = pose(1, 2);

  pose_inv << ct, st, -x*ct - y*st,
             -st, ct,  x*st - y*ct,
               0,  0,            1;

  return pose_inv;

}

double distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) {

  auto n = x1.size();

  double ss{0.};

  for (unsigned int i = 0; i < n; i++) {

    double diff = x1(i) - x2(i);

    ss += diff * diff;
  }

  return sqrt(ss);

}

ClosestWaypoint findClosestWaypoint(const WaypointsMap& map) {

  Eigen::VectorXd origin{2};
  origin << 0, 0;

  auto n = map.x.size();
  double min_dist = std::numeric_limits<double>::max();
  unsigned int closest_idx = 0;
  Eigen::VectorXd current{2};

  for (unsigned int i = 0; i < n; i++) {

    current << map.x[i], map.y[i];
    double dist = distance(current, origin);

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return ClosestWaypoint{closest_idx, min_dist};

}

WaypointsMap getNextWaypoints(const WaypointsMap& points, const ClosestWaypoint& closest, int n) {

  int idx = closest.index;

  if (closest.distanceToOrigin < 0) {
    idx++;
  }

  WaypointsMap nextWaypoints;

  nextWaypoints.x = Eigen::VectorXd{n};
  nextWaypoints.y = Eigen::VectorXd{n};

  unsigned int i = 0;
  while (i < n) {

    nextWaypoints.x(i) = points.x(idx);
    nextWaypoints.y(i) = points.y(idx);

    idx++;
    if (idx == n) {
      idx = 0;
    }

    i++;

  }

  return nextWaypoints;
}

Line closestLine(const WaypointsMap& points, const ClosestWaypoint& closest) {

  double x1, x2, y1, y2;

  if (closest.distanceToOrigin > 0) {

    x1 = points.x(closest.index - 1);
    y1 = points.y(closest.index - 1);

    x2 = points.x(closest.index);
    y2 = points.y(closest.index);

  } else {

    x1 = points.x(closest.index);
    y1 = points.y(closest.index);

    x2 = points.x(closest.index + 1);
    y2 = points.y(closest.index + 1);

  }

  Line line = createLine(x1, y1, x2, y2);

  return line;

}

Line createLine(double x1, double y1, double x2, double y2) {

  Line line{};

  line.slope = (y2 - y1) / (x2 - x1);
  line.intercept = y2 - line.slope * x2;

  return line;

}

Errors errorsFromLine(const Line& line, double x0, double y0) {

  Errors err{};

  err.cte = abs(line.intercept + line.slope * x0 - y0) / sqrt(1 + line.slope * line.slope);
  err.epsi = atan(line.slope);

  return err;

}
