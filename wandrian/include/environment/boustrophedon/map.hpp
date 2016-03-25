/*
 * map.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_MAP_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_MAP_HPP_

#include <string>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#include "../../environment/boustrophedon/obstacle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Map {

public:
  std::string namefile;

  Map(ObstaclePtr, std::list<ObstaclePtr>);
  Map(std::string);
  int static commaposition(std::string);

  ObstaclePtr get_environment();
  std::list<ObstaclePtr> get_obstacles();
  void set_environment();

private:
  ObstaclePtr environment;
  std::list<ObstaclePtr> obstacles;

};

typedef boost::shared_ptr<Map> MapPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_MAP_HPP_ */
