/*
 * identifiable.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: sontd
 */

#include "../../../include/environment/mstc/identifiable.hpp"

namespace wandrian {
namespace environment {
namespace mstc {

Identifiable::Identifiable(std::string robot_name) :
    robot_name(robot_name) {
}

Identifiable::~Identifiable() {
}

std::string Identifiable::get_robot_name() const {
  return robot_name;
}

}
}
}
