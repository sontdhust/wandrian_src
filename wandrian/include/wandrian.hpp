/*
 * wandrian.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: anhnt
 */

#ifndef WANDRIAN_INCLUDE_WANDRIAN_HPP_
#define WANDRIAN_INCLUDE_WANDRIAN_HPP_

#include "plans/base_plan.hpp"
#include "robot.hpp"

using namespace wandrian::plans;

namespace wandrian {

class Wandrian {

public:
  Wandrian();
  ~Wandrian();
  bool initialize();
  void spin();

private:
  RobotPtr robot;
  BasePlanPtr plan;

  // Behaviors
  void wandrian_run();

  bool spiral_stc_go_to(PointPtr, bool);
  bool spiral_stc_see_obstacle(VectorPtr, double);
  bool full_spiral_stc_go_to(PointPtr, bool);
  bool full_spiral_stc_see_obstacle(VectorPtr, double);
  bool mstc_online_go_to(PointPtr, bool);
  bool mstc_online_see_obstacle(VectorPtr, double);
  bool full_mstc_online_go_to(PointPtr, bool);
  bool full_mstc_online_see_obstacle(VectorPtr, double);
  bool boustrophedon_online_go_to(PointPtr, bool);
  bool boustrophedon_online_see_obstacle(VectorPtr, double);
  bool boustrophedon_go_to(PointPtr, bool);

  // Helpers
  bool go_to(PointPtr, bool);
  bool see_obstacle(Orientation, double);
  bool rotate_to(PointPtr, bool);
  bool rotate_to(VectorPtr, bool);
  void go(bool);
  void rotate(bool);
  void dodge();
};

typedef boost::shared_ptr<Wandrian> WandrianPtr;

}

#endif /* WANDRIAN_INCLUDE_WANDRIAN_HPP_ */
