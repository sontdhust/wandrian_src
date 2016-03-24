/*
 * identifiable.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: sontd
 */

#include "../../../include/environment/mstc_online/identifiable.hpp"

namespace wandrian {
namespace environment {
namespace mstc_online {

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
