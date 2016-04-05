/*
 * identifiable_cell.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#include "../../../include/environment/mstc/identifiable_cell.hpp"

namespace wandrian {
namespace environment {
namespace mstc {

IdentifiableCell::IdentifiableCell(PointPtr center, double size,
    std::string robot_name) :
    Cell(center, size), Identifiable(robot_name) {
}

IdentifiableCell::~IdentifiableCell() {
}

}
}
}
