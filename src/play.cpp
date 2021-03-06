//
// Created by Oleksandr Semeniuta on 2018-12-26.
//

#include "geometry.h"
#include <iostream>

void printMap(const WaypointsMap& map) {

  for (unsigned int i = 0; i < map.x.size(); i++) {
    std::cout << map.x[i] << ", " << map.y[i] << std::endl;
  }

}

int main() {

  WaypointsMap map = readWaypoints("../data/waypoints.txt");

  double car_x = -40.62;
  double car_y = 108.73;
  double car_psi = 3.733651;

  auto car_pose = createPose(car_x, car_y, car_psi);
  auto t_map_to_car = invertPose(car_pose);

  WaypointsMap map_t = map.transformAll(t_map_to_car);

  auto closest = findClosestWaypoint(map_t);

  std::cout << "Closest index: " << closest.index << ". Distance: " << closest.distanceToOrigin << std::endl;

  auto nextWP = getNextWaypoints(map_t, closest, 10);
  printMap(nextWP);

  Line cl = closestLine(map_t, closest);

  std::cout << "Closest line: " << cl.slope << ", " << cl.intercept << std::endl;

  Eigen::VectorXd coeffs = polyfit(nextWP.x, nextWP.y, 3);
  std::cout << "Coefficients:\n" << coeffs << std::endl;

  auto err = errorsFromLine(cl);
  std::cout << "cte: " << err.cte << ", epsi:" << err.epsi << std::endl;

}