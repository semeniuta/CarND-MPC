//
// Created by Oleksandr Semeniuta on 2018-12-29.
//

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include "geometry.h"

class Localizer {

public:

  Eigen::VectorXd state_;
  Eigen::VectorXd poly_coeffs_;
  bool never_activated_;

  Localizer() = delete;
  Localizer(const WaypointsMap& map, int n_waypoints, int poly_order);

  ~Localizer() = default;

  void activate(double x, double y, double psi, double v);


private:

  int n_waypoints_;
  int poly_order_;
  WaypointsMap map_;

  Eigen::MatrixXd t_map_to_vehicle_;
  WaypointsMap waypoints_from_vehicle_;
  ClosestWaypoint closest_;

};


#endif //LOCALIZER_H
