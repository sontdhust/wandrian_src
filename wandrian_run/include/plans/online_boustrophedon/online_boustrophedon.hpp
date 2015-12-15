/*
 * online_boustrophedon.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_ONLINE_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_ONLINE_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_HPP_

#include "../../common/environment.hpp"
#include "../base_plan.hpp"
#include "cell.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace online_boustrophedon {

class OnlineBoustrophedon: public BasePlan {

public:
  OnlineBoustrophedon();
  ~OnlineBoustrophedon();
  void initialize(PointPtr, double);
  void cover();

  /* __attribute__((will_be_removed)) */
  void set_environment(EnvironmentPtr);
  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  bool go_to(PointPtr, bool);

private:
  /* __attribute__((will_be_removed)) */
  EnvironmentPtr environment;
  CellPtr starting_cell;
  double robot_size; // = 'cell size' / 2
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
  std::set<CellPtr, CellComp> old_cells;

  bool see_obstacle(VectorPtr, double);
  bool go_with(VectorPtr, double);
  void online_boustrophedon(CellPtr);
  bool check(CellPtr);
};

typedef boost::shared_ptr<OnlineBoustrophedon> OnlineBoustrophedonPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_ONLINE_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_HPP_ */
