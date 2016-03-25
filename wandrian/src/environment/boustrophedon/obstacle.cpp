/*
 * obstacle.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/environment/boustrophedon/obstacle.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

Obstacle::Obstacle(PointPtr center, double size_x, double size_y) :
    center(center), size_x(size_x), size_y(size_y) {
  points.insert(points.end(),
      PointPtr(new Point(center->x - size_x / 2, center->y + size_y / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size_x / 2, center->y + size_y / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size_x / 2, center->y - size_y / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x - size_x / 2, center->y - size_y / 2)));
  build();
}

PointPtr Obstacle::get_center() {
  return center;
}

double Obstacle::get_size_x() {
  return size_x;
}

double Obstacle::get_size_y() {
  return size_y;
}

}
}
}
