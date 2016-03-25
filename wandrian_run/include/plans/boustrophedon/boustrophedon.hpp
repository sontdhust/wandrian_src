/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_BOUSTROPHEDON_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_BOUSTROPHEDON_HPP_

#include <list>
#include "../../common/vector.hpp"
#include "../base_plan.hpp"
#include "../../environment/cell.hpp"
#include "../../environment/boustrophedon/obstacle.hpp"
#include "../../environment/boustrophedon/vertices.hpp"
#include "../../environment/boustrophedon/map.hpp"
#include "ros/ros.h"
#include "../../environment/boustrophedon/free_zone.hpp"

using namespace wandrian::common;
using namespace wandrian::environment;
using namespace wandrian::environment::boustrophedon;

namespace wandrian {
namespace plans {
namespace boustrophedon {

class Boustrophedon: public BasePlan {

public:
  Boustrophedon();
  ~Boustrophedon();
  void initialize(PointPtr, double, std::string);
  void cover();
  void dfs(FreeZonePtr);
  std::list<FreeZonePtr> create_list_space(ObstaclePtr, std::list<VerticesPtr>);
  std::list<VerticesPtr> create_list_vertices(ObstaclePtr,
      std::list<ObstaclePtr>);

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  bool go_to(PointPtr, bool);
  bool go_go(FreeZonePtr);

private:
  CellPtr starting_cell;
  double robot_size; // = 'cell size' / 2
  MapPtr environment;
  std::set<CellPtr, CellComp> old_cells;

  bool go_with(VectorPtr, double);
  void boustrophedon_cd();
  bool check(CellPtr);
};

typedef boost::shared_ptr<Boustrophedon> BoustrophedonPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_OFF_BOUSTROPHEDON_HPP_ */
