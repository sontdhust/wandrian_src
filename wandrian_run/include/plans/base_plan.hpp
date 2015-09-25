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
	virtual ~BasePlan();
	void set_go_behavior(boost::function<bool(VectorPtr, int)>);
	virtual void cover();
	std::list<PointPtr> path;

protected:
	boost::function<bool(VectorPtr, int)> go_behavior;
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BASE_PLAN_HPP_ */
