//
// Created by Oleksandr Semeniuta on 2018-12-29.
//

#include "Localizer.h"

Localizer::Localizer(const WaypointsMap& map, int n_waypoints, int poly_order)
: never_activated_{true},
  n_waypoints_{n_waypoints},
  map_{map},
  poly_order_{poly_order} {

  state_ = Eigen::VectorXd{6};
  poly_coeffs_ = Eigen::VectorXd{poly_order_ + 1};
  vehicle_pose_ = Eigen::MatrixXd{3, 3};

}

void Localizer::activate(double x, double y, double psi, double v) {

  vehicle_pose_ = createPose(x, y, psi);
  t_map_to_vehicle_ = invertPose(vehicle_pose_);

  waypoints_from_vehicle_ = map_.transformAll(t_map_to_vehicle_);
  closest_ = findClosestWaypoint(waypoints_from_vehicle_);

  auto next_waypoints = getNextWaypoints(waypoints_from_vehicle_, closest_, n_waypoints_);
  poly_coeffs_ = polyfit(next_waypoints.x, next_waypoints.y, poly_order_);

  Line closest_line = closestLine(waypoints_from_vehicle_, closest_);
  Errors err = errorsFromLine(closest_line);
  state_ << 0, 0, 0, v, err.cte, err.epsi;

}
