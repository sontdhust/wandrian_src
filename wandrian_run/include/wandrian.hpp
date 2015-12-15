/*
 * wandrian.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_
#define WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_

#include "common/vector.hpp"
#include "core.hpp"
#include "plans/online_boustrophedon/online_boustrophedon.hpp"

using namespace wandrian::plans::online_boustrophedon;

namespace wandrian {

class Wandrian {

public:
  bool initialize();
  void spin();

private:
  Core core;

  // Behaviors
  void wandrian_run();
  bool online_boustrophedon_go_to(PointPtr, bool);
  bool online_boustrophedon_see_obstacle(VectorPtr, double);

  // Helpers
  bool go_to(PointPtr, bool);
  bool rotate_to(PointPtr, bool);
  void go(bool);
  void rotate(bool);
};

}

#endif /* WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_ */
