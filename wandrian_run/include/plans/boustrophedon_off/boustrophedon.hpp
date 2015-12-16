/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_OFF_BOUSTROPHEDON_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_OFF_BOUSTROPHEDON_HPP_

#include "../../common/environment.hpp"
#include "../base_plan.hpp"
#include "cell.hpp"
#include "ros/ros.h"

using namespace wandrian::common;

namespace wandrian {
namespace plans {

namespace boustrophedon_off {

class Boustrophedon: public BasePlan {

public:
  Boustrophedon();
  ~Boustrophedon();
  void initialize(PointPtr, double);
  void cover();

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  bool go_to(PointPtr, bool);

private:
  CellPtr starting_cell;
  double robot_size; // = 'cell size' / 2
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
  std::set<CellPtr, CellComp> old_cells;

  bool see_obstacle(VectorPtr, double);
  bool go_with(VectorPtr, double);

  void boustrophedon_pt(CellPtr);
  bool check(CellPtr);
};

typedef boost::shared_ptr<Boustrophedon> BoustrophedonPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_OFF_BOUSTROPHEDON_HPP_ */
