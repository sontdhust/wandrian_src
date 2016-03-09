/*
 * wandrian.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_
#define WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_

#include "robot.hpp"

namespace wandrian {

class Wandrian {

public:
  Wandrian();
  ~Wandrian();
  bool initialize();
  void spin();

private:
  RobotPtr robot;

  // Behaviors
  void wandrian_run();
  bool spiral_stc_go_to(PointPtr, bool);
  bool spiral_stc_see_obstacle(VectorPtr, double);
  bool full_spiral_stc_go_to(PointPtr, bool);
  bool full_spiral_stc_see_obstacle(VectorPtr, double);
  bool mstc_online_go_to(PointPtr, bool);
  bool mstc_online_see_obstacle(VectorPtr, double);

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

#endif /* WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_ */
