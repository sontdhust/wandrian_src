/*
 * identifiable_cell.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#include "../../../include/environment/mstc_online/identifiable_cell.hpp"

namespace wandrian {
namespace environment {
namespace mstc_online {

IdentifiableCell::IdentifiableCell(PointPtr center, double size,
    std::string robot_name) :
    Cell(center, size), Identifiable(robot_name) {
}

IdentifiableCell::~IdentifiableCell() {
}

}
}
}
