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
    quadrant1(NEW), quadrant2(NEW), quadrant3(NEW), quadrant4(NEW), center(
        center), size(size) {
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

PointPtr Cell::get_center() {
  return center;
}

double Cell::get_size() {
  return size;
}

void Cell::set_parent(CellPtr parent) {
  this->parent = parent;
}

CellPtr Cell::get_parent() {
  return parent;
}

}
}
}
