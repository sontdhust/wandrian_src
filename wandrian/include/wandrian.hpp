/*
 * wandrian.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: anhnt
 */

#ifndef WANDRIAN_INCLUDE_WANDRIAN_HPP_
#define WANDRIAN_INCLUDE_WANDRIAN_HPP_

#include "plans/base_plan.hpp"
#include "environment/map.hpp"
#include "robot.hpp"

using namespace wandrian::plans;
using namespace wandrian::environment;

namespace wandrian {

class Wandrian {

public:
  Wandrian();
  ~Wandrian();
  bool initialize();
  void spin();

private:
  RobotPtr robot;
  int step_count;
  int deviation_linear_count;
  int deviation_angular_count;
  std::list<PointPtr> path;
  std::list<PointPtr> actual_path;
  MapPtr map;

  // Behaviors
  void wandrian_run();
  bool wandrian_go_to(PointPtr, bool = STRICTLY);
  bool wandrian_see_obstacle(VectorPtr, double);

  // Helpers
  bool rotate_to(PointPtr, bool);
  bool rotate_to(VectorPtr, bool);
  void go(bool);
  void rotate(bool);
  std::string find_map_path();
};

typedef boost::shared_ptr<Wandrian> WandrianPtr;

}

#endif /* WANDRIAN_INCLUDE_WANDRIAN_HPP_ */
