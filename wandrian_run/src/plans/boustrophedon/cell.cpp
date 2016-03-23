/*
 * square.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/boustrophedon/cell.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon{

Cell::Cell(PointPtr center, double size) :
    center(center), size(size), parent(CellPtr()) {
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

