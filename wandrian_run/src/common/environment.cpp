/*
 * environment.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include "../../include/common/environment.hpp"

namespace wandrian {
namespace common {

Environment::Environment(Polygon* space, std::set<Polygon*> obstacles) :
		space(space), obstacles(obstacles) {
}

}
}
