/*
 * wandrian.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: anhnt
 */

#ifndef WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_
#define WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_

#include "common/vector.hpp"
#include "core.hpp"
#include "plans/boustrophedon_online/boustrophedon_online.hpp"

using namespace wandrian::plans::boustrophedon_online;

namespace wandrian {

class Wandrian {

public:
  bool initialize();
  void spin();

private:
  Core core;

  // Behaviors
  void wandrian_run();
  bool boustrophedon_online_go_to(PointPtr, bool);
  bool boustrophedon_online_see_obstacle(VectorPtr, double);

  // Helpers
  bool go_to(PointPtr, bool);
  bool rotate_to(PointPtr, bool);
  void go(bool);
  void rotate(bool);
};

}

#endif /* WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_ */
