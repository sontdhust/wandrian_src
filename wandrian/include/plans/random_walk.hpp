/*
 * random_walk.hpp
 *
 *  Created on: Jun 9, 2016
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_PLANS_RANDOM_WALK_HPP_
#define WANDRIAN_INCLUDE_PLANS_RANDOM_WALK_HPP_

#include <boost/function.hpp>

namespace wandrian {
namespace plans {

class RandomWalk {

public:
  RandomWalk();
  ~RandomWalk();
  void cover();

  void set_behavior_rotate_randomly(boost::function<void()>);
  void set_behavior_go_straight(boost::function<void()>);

private:
  boost::function<void()> behavior_rotate_randomly;
  boost::function<void()> behavior_go_straight;

  void rotate_randomly();
  void go_straight();
};

typedef boost::shared_ptr<RandomWalk> RandomWalkPtr;

}
}

#endif /* WANDRIAN_INCLUDE_PLANS_RANDOM_WALK_HPP_ */
