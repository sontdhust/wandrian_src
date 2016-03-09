/*
 * cell.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: manhnh
 */

#include "../../../include/plans/mstc_online/cell.hpp"

namespace wandrian {
namespace plans {
namespace mstc_online {

Cell::Cell(PointPtr center, double size, std::string robot_name) :
    center(center), size(size), robot_name(robot_name) {
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

Cell::~Cell() {
}

PointPtr Cell::get_center() const {
  return center;
}

double Cell::get_size() const {
  return size;
}

std::string Cell::get_robot_name() const {
  return robot_name;
}

CellPtr Cell::get_parent() {
  return parent;
}

void Cell::set_parent(CellPtr parent) {
  this->parent = parent;
}

}
}
}
