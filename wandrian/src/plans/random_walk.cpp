/*
 * random_walk.cpp
 *
 *  Created on: Jun 9, 2016
 *      Author: sontd
 */

#include <stdlib.h>
#include <ctime>
#include <cmath>
#include "../../include/plans/random_walk.hpp"

namespace wandrian {
namespace plans {

RandomWalk::RandomWalk() {
}

RandomWalk::~RandomWalk() {
}

void RandomWalk::cover() {
  std::srand(std::time(0));
  while (true) {
    go_straight();
    rotate_randomly();
  }
}

void RandomWalk::set_behavior_rotate_randomly(
    boost::function<void()> behavior_rotate_randomly) {
  this->behavior_rotate_randomly = behavior_rotate_randomly;
}

void RandomWalk::set_behavior_go_straight(
    boost::function<void()> behavior_go_straight) {
  this->behavior_go_straight = behavior_go_straight;
}

void RandomWalk::rotate_randomly() {
  if (behavior_rotate_randomly) {
    behavior_rotate_randomly();
  }
}

void RandomWalk::go_straight() {
  if (behavior_go_straight) {
    behavior_go_straight();
  }
}

}
}
