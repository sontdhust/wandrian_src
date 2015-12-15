/*
 * wandrian.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_MSTC_ONLINE_INCLUDE_WANDRIAN_HPP_
#define WANDRIAN_MSTC_ONLINE_INCLUDE_WANDRIAN_HPP_

#include "extended_core.hpp"

namespace wandrian {

class Wandrian {

public:
  bool initialize();
  void spin();

private:
  ExtendedCore extended_core;

  // Behaviors
  void wandrian_mstc_online();
  bool spiral_stc_go_to(PointPtr, bool);
  bool spiral_stc_see_obstacle(VectorPtr, double);
  bool full_spiral_stc_go_to(PointPtr, bool);
  bool full_spiral_stc_see_obstacle(VectorPtr, double);

  // Helpers
  bool go_to(PointPtr, bool);
  bool rotate_to(PointPtr, bool);
  void go(bool);
  void rotate(bool);
};

}

#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_WANDRIAN_HPP_ */
