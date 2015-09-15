/*
 * environment.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include "../../include/common/environment.hpp"

namespace wandrian {
namespace common {

Environment::Environment(PolygonPtr space, std::set<PolygonPtr> obstacles) :
		space(space), obstacles(obstacles) {
}

}
}
