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

#define FLEXIBLY true
#define STRICTLY false

using namespace wandrian::common;

namespace wandrian {
namespace plans {

class BasePlan {

public:
  virtual ~BasePlan();
  virtual void cover();

  void set_behavior_go_to(boost::function<bool(PointPtr, bool)>);

protected:
  std::list<PointPtr> path;
  boost::function<bool(PointPtr, bool)> behavior_go_to;

  virtual bool go_to(PointPtr, bool);
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BASE_PLAN_HPP_ */
