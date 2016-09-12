/*
 * base_plan.cpp
 *
 *  Created on: Sep 24, 2015
 *      Author: sontd
 */

#include "../../include/plans/base_plan.hpp"

namespace wandrian {
namespace plans {

BasePlan::BasePlan() {}

BasePlan::~BasePlan() {
  // Override this method
}

void BasePlan::cover() {
  // Override this method
}

void BasePlan::set_behavior_go_to(boost::function<bool(PointPtr, bool)> behavior_go_to) {
  this->behavior_go_to = behavior_go_to;
}

bool BasePlan::go_to(PointPtr position, bool flexibility) {
  // Override this method
  if (behavior_go_to) {
    bool successful = behavior_go_to(position, flexibility);
    if (successful)
      path.insert(path.end(), position);
    return successful;
  }
  path.insert(path.end(), position);
  return true;
}
}
}
