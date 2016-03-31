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
#include "../../common/rectangle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class ExtendedMap {

public:
  std::string file_name;

  ExtendedMap(RectanglePtr, std::list<RectanglePtr>);
  ExtendedMap(std::string);
  void build();

  RectanglePtr get_boundary();
  std::list<RectanglePtr> get_obstacles();

private:
  RectanglePtr environment;
  std::list<RectanglePtr> obstacles;

  int static comma_position(std::string);

};

typedef boost::shared_ptr<ExtendedMap> ExtendedMapPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_EXTENDED_MAP_HPP_ */
