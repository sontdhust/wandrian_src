/*
 * base_plan.hpp
 *
 *  Created on: Sep 24, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_BASE_PLAN_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_BASE_PLAN_HPP_

#include <list>
#include <boost/function.hpp>
#include "../common/point.hpp"
#include "../common/vector.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {

class BasePlan {

public:
	std::list<PointPtr> path;

	virtual ~BasePlan();
	void set_behavior_go_to(boost::function<bool(PointPtr)>);
	virtual void cover();

protected:
	boost::function<bool(PointPtr)> behavior_go_to;

	virtual bool go_to(PointPtr);
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BASE_PLAN_HPP_ */
