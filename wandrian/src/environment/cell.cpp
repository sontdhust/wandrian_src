/*
 * cell.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#include "../../include/environment/cell.hpp"

namespace wandrian {
namespace environment {

Cell::Cell(PointPtr center, double size) :
    Rectangle(center, size, size) {
}

Cell::~Cell() {
}

double Cell::get_size() const {
  return (get_width() + get_height()) / 2;
}

CellPtr Cell::get_parent() {
  return parent;
}

void Cell::set_parent(CellPtr parent) {
  this->parent = parent;
}

}
}
