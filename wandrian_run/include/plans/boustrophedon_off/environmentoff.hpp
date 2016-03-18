#ifndef WANDRIAN_RUN_INCLUDE_PLANS_BOUSTRONPHENDON_OFF_ENVIRONMENTOFF_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_BOUSTRONPHENDON_OFF_ENVIRONMENTOFF_HPP_

#include <string>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#include "obstacle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace boustrophedon_off {

class Environmentoff {


public:
	  std::string namefile;

	  Environmentoff(ObstaclePtr, std::list<ObstaclePtr>);

	  Environmentoff(std::string);

	  ObstaclePtr get_environment();

	  std::list<ObstaclePtr> get_obstacles();

	  int static commaposition(std::string);

	  void set_environment();

private:

	  ObstaclePtr environment;
	  std::list<ObstaclePtr> obstacles;

};

typedef boost::shared_ptr<Environmentoff> EnvironmentOffPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_ENVIRONMENTOFF_HPP_ */
