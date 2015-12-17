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
#include "plans/spiral_stc/spiral_stc.hpp"
#include "plans/boustrophedon_off/boustrophedon.hpp"

using namespace wandrian::plans::spiral_stc;
using namespace wandrian::plans::boustrophedon_off;

namespace wandrian {

class Wandrian {

public:
  bool initialize();
  void spin();

private:
  Core core;

  // Behaviors
  void wandrian_run();
  bool spiral_stc_go_to(PointPtr, bool);
  bool spiral_stc_see_obstacle(VectorPtr, double);
  bool boustrophedon_cd_go_to(PointPtr, bool);
  bool boustrophedon_cd_see_obstacle(VectorPtr, double);

  // Helpers
  bool go_to(PointPtr, bool);
  bool rotate_to(PointPtr, bool);
  void go(bool);
  void rotate(bool);
};

}

#endif /* WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_ */
