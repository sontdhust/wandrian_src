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
  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  bool go_to(PointPtr, bool);
  bool go_to_bpcell(PointPtr, bool);

private:

  CellPtr starting_cell;
  double robot_size; // = 'cell size' 
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
  std::set<CellPtr, CellComp> old_cells;
  std::set<CellPtr, CellComp> bplist;
  
  bool see_obstacle(VectorPtr, double);
  bool go_with(VectorPtr, double);
  bool go_with_bpcell(PointPtr, VectorPtr, double);
  void turn_left(CellPtr , CellPtr, VectorPtr);
  void turn_right(CellPtr , CellPtr, VectorPtr);
  void go_straight(CellPtr , CellPtr, VectorPtr);
  void move_bpcell(CellPtr);
  void bpmove(CellPtr, CellPtr);
  double get_distance(CellPtr, CellPtr);
  void find_bpcell(CellPtr);
  void online_boustrophedon(CellPtr);
  bool check(CellPtr);
};

typedef boost::shared_ptr<OnlineBoustrophedon> OnlineBoustrophedonPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_ONLINE_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_HPP_ */