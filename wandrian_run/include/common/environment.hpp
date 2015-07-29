/*
 * environment.hpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_ENVIRONMENT_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_ENVIRONMENT_HPP_

#include "polygon.hpp"

namespace common {
namespace shapes {

class Environment {

public:
	Environment(Polygon*, std::set<Polygon*>);
	Polygon* get_space();
	std::set<Polygon*> get_obstacles();

private:
	Polygon *space;
	std::set<Polygon*> obstacles;

};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_ENVIRONMENT_HPP_ */
