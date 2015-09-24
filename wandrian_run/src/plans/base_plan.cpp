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

std::list<PointPtr> BasePlan::get_path() {
	return path;
}

void BasePlan::cover() {
	// Override this method
}

}
}

