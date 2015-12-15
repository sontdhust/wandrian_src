/*
 * environment.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: manhnh
 */

#include "../../include/common/environment.hpp"

namespace wandrian {
namespace common {

Environment::Environment(PolygonPtr space, std::list<PolygonPtr> obstacles) :
    space(space), obstacles(obstacles) {
}

}
}
