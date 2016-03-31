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

Map::Map(std::string file_name) {
  this->file_name = file_name;
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

std::string Map::get_file_name() {
  return file_name;
}

void Map::build() {
}

void Map::initialize() {
  build();
}

}
}
