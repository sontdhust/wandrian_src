/*
 * extended_map.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_EXTENDED_MAP_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_EXTENDED_MAP_HPP_

#include <string>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "../map.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class ExtendedMap: public Map {

public:
  ExtendedMap(RectanglePtr, std::list<RectanglePtr>);
  ExtendedMap(std::string);
  ~ExtendedMap();
  void build();
  std::list<PolygonPtr> get_extendedmap_obstacles();
  int number_space_need_visit;
private:
  std::list<PolygonPtr> extendedmap_obstacle;
  int static comma_position(std::string);
  PointPtr create_point_to_string(std::string);

};

typedef boost::shared_ptr<ExtendedMap> ExtendedMapPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_EXTENDED_MAP_HPP_ */
