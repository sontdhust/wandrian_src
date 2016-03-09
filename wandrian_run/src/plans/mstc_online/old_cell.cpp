/*
 * old_cell.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: manhnh
 */

#include "../../../include/plans/mstc_online/old_cell.hpp"

namespace wandrian {
namespace plans {
namespace mstc_online {

OldCell::OldCell(CellPtr old_cell, std::string robot_name) :
    Cell(old_cell->get_center(), old_cell->get_size()) {
  this->old_cell = old_cell;
  this->robot_name = robot_name;
}

OldCell::~OldCell() {
}

const std::string& OldCell::get_robot_name() const {
  return robot_name;
}

void OldCell::set_robot_name(const std::string& robot_name) {
  this->robot_name = robot_name;
}

const CellPtr& OldCell::get_old_cell() const {
  return old_cell;
}

void OldCell::set_old_cell(const CellPtr& old_cell) {
  this->old_cell = old_cell;
}

}
}
}
