/*
 * space.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include "../../include/environment/space.hpp"

namespace wandrian {
namespace environment {

Space::Space(PolygonPtr space, std::list<PolygonPtr> obstacles) :
    boundary(space), obstacles(obstacles) {
}

}
}
