/*
 * map.hpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_MAP_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_MAP_HPP_

#include "../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {

struct Map {

  PolygonPtr boundary;
  std::list<PolygonPtr> obstacles;

  Map(PolygonPtr, std::list<PolygonPtr>);
};

typedef boost::shared_ptr<Map> MapPtr;

}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_MAP_HPP_ */
