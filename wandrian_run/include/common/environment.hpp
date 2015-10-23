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

  PolygonPtr space;
  std::list<PolygonPtr> obstacles;

  Environment(PolygonPtr, std::list<PolygonPtr>);

};

typedef boost::shared_ptr<Environment> EnvironmentPtr;

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_ENVIRONMENT_HPP_ */
