/*
 * base_plan.cpp
 *
 *  Created on: Sep 24, 2015
 *      Author: sontd
 */

#include "../../include/plans/base_plan.hpp"

namespace wandrian {
namespace plans {

BasePlan::~BasePlan() {
	// Override this method
}

void BasePlan::set_go_behavior(
		boost::function<bool(VectorPtr, int)> go_behavior) {
	this->go_behavior = go_behavior;
}

void BasePlan::cover() {
	// Override this method
}

}
}

