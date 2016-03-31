/*
 * map.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include "../../include/environment/map.hpp"

namespace wandrian {
namespace environment {

Map::Map(RectanglePtr boundary, std::list<RectanglePtr> obstacles) :
    boundary(boundary), obstacles(obstacles) {
}

Map::Map(std::string map_path) {
  this->map_path = map_path;
  initialize();
}

Map::~Map() {
}

RectanglePtr Map::get_boundary() {
  return boundary;
}

std::list<RectanglePtr> Map::get_obstacles() {
  return obstacles;
}

std::string Map::get_map_path() {
  return map_path;
}

void Map::build() {
}

void Map::initialize() {
  build();
}

}
}
