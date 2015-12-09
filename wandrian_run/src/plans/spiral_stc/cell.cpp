/*
 * square.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/spiral_stc/cell.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {

Cell::Cell(PointPtr center, double size) :
    current_quadrant(IV), center(center), size(size) {
  for (int i = I; i <= IV; i++)
    quadrants[i] = NEW;
  points.insert(points.end(),
      PointPtr(new Point(center->x - size / 2, center->y + size / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size / 2, center->y + size / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size / 2, center->y - size / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x - size / 2, center->y - size / 2)));
  build();
}

PointPtr Cell::get_center() const {
  return center;
}

double Cell::get_size() {
  return size;
}

CellPtr Cell::get_parent() {
  return parent;
}

PointPtr Cell::get_current_position() {
  switch (current_quadrant) {
  case I:
    return PointPtr(new Point(center->x + size / 4, center->y + size / 4));
  case II:
    return PointPtr(new Point(center->x - size / 4, center->y + size / 4));
  case III:
    return PointPtr(new Point(center->x - size / 4, center->y - size / 4));
  case IV:
    return PointPtr(new Point(center->x + size / 4, center->y - size / 4));
  default:
    return PointPtr(new Point(center->x, center->y));
  }
}

void Cell::set_parent(CellPtr parent) {
  this->parent = parent;
}

}
}
}
