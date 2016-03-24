#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHENDON_ENVIRONMENTOFF_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHENDON_ENVIRONMENTOFF_HPP_

#include <string>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#include "obstacle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Map {

public:
  std::string namefile;
  Map(ObstaclePtr, std::list<ObstaclePtr>);
  Map(std::string);

  ObstaclePtr get_environment();
  std::list<ObstaclePtr> get_obstacles();
  int static commaposition(std::string);
  void set_environment();

private:
  ObstaclePtr environment;
  std::list<ObstaclePtr> obstacles;

};

typedef boost::shared_ptr<Map> MapPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHENDON_ENVIRONMENTOFF_HPP_ */
