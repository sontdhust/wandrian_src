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
#include "obstacle.hpp"
#include "space.hpp"
#include "vertices.hpp"
#include "ros/ros.h"
#include <list>
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
  
  void dfs(SpacePtr);
  std::list<SpacePtr> create_space(std::list<SpacePtr>, SpacePtr);

protected:
  bool go_to(PointPtr, bool);
  bool go_go(SpacePtr);

private:
  CellPtr starting_cell;
  double robot_size; // = 'cell size' / 2
  double environment_size;
  
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
  std::set<CellPtr, CellComp> old_cells;

  bool see_obstacle(VectorPtr, double);
  bool go_with(VectorPtr, double);

  void boustrophedon_cd();
  bool check(CellPtr);
};

typedef boost::shared_ptr<Boustrophedon> BoustrophedonPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_OFF_BOUSTROPHEDON_HPP_ */
