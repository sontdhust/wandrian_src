/*
 * space.hpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_SPACE_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_SPACE_HPP_

#include "../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {

struct Space {

  PolygonPtr boundary;
  std::list<PolygonPtr> obstacles;

  Space(PolygonPtr, std::list<PolygonPtr>);
};

typedef boost::shared_ptr<Space> SpacePtr;

}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_SPACE_HPP_ */
