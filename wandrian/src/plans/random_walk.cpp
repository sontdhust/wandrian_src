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
    rotate(-M_PI + (double) rand() / RAND_MAX * 2 * M_PI);
  }
}

void RandomWalk::set_behavior_rotate(
    boost::function<void(double)> behavior_rotate) {
  this->behavior_rotate = behavior_rotate;
}

void RandomWalk::set_behavior_go_straight(
    boost::function<void()> behavior_go_straight) {
  this->behavior_go_straight = behavior_go_straight;
}

void RandomWalk::rotate(double angle) {
  if (behavior_rotate) {
    behavior_rotate(angle);
  }
}

void RandomWalk::go_straight() {
  if (behavior_go_straight) {
    behavior_go_straight();
  }
}

}
}
