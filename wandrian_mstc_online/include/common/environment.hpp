/*
 * environment.hpp
 *
 *  Created on: Jul 28, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_MSTC_ONLINE_INCLUDE_COMMON_ENVIRONMENT_HPP_
#define WANDRIAN_MSTC_ONLINE_INCLUDE_COMMON_ENVIRONMENT_HPP_

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

#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_COMMON_ENVIRONMENT_HPP_ */
