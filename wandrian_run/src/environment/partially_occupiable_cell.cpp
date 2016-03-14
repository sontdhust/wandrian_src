/*
 * partially_occupiable_cell.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#include "../../include/environment/partially_occupiable_cell.hpp"

namespace wandrian {
namespace environment {

PartiallyOccupiableCell::PartiallyOccupiableCell(PointPtr center, double size) :
    Cell(center, size) {
}

PartiallyOccupiableCell::~PartiallyOccupiableCell() {
}

PointPtr PartiallyOccupiableCell::get_current_position() {
  return current_position(center, size);
}

}
}
