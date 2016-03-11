/*
 * space.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include "../../include/common/space.hpp"

namespace wandrian {
namespace common {

Space::Space(PolygonPtr space, std::list<PolygonPtr> obstacles) :
    boundary(space), obstacles(obstacles) {
}

}
}
