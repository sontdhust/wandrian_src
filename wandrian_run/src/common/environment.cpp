/*
 * environment.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include "../../include/common/environment.hpp"

namespace common {
namespace shapes {

Environment::Environment(Polygon* space, std::set<Polygon*> obstacles) :
		space(space), obstacles(obstacles) {
}

Polygon* Environment::get_space() {
	return space;
}

std::set<Polygon*> Environment::get_obstacles() {
	return obstacles;
}

}
}

