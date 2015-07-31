/*
 * environment.hpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_ENVIRONMENT_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_ENVIRONMENT_HPP_

#include "polygon.hpp"

namespace wandrian {
namespace common {

struct Environment {

	Environment(Polygon*, std::set<Polygon*>);

	Polygon *space;
	std::set<Polygon*> obstacles;

};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_ENVIRONMENT_HPP_ */
