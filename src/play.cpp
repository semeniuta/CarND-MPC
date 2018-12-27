//
// Created by Oleksandr Semeniuta on 2018-12-26.
//

#include "geometry.h"
#include <iostream>

int main() {

  WaypointsMap map = readWaypoints("../data/waypoints.txt");

  for (unsigned int i = 0; i < map.x.size(); i++) {
    std::cout << map.x[i] << ", " << map.y[i] << std::endl;
  }

}