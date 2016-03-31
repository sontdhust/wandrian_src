/*
 * map.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include "../../include/environment/map.hpp"

namespace wandrian {
namespace environment {

Map::Map(PolygonPtr space, std::list<PolygonPtr> obstacles) :
    boundary(space), obstacles(obstacles) {
}

}
}
