/*
 * boustrophedon.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_BOUSTROPHEDON_HPP_
#define WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_BOUSTROPHEDON_HPP_

#include <list>
#include <ros/ros.h>
#include "../../common/vector.hpp"
#include "../../common/rectangle.hpp"
#include "../../environment/cell.hpp"
#include "../../environment/boustrophedon/extended_map.hpp"
#include "../../environment/boustrophedon/space.hpp"
#include "../../environment/boustrophedon/vertices.hpp"
#include "../base_plan.hpp"

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
  void dfs(SpacePtr);
  std::list<SpacePtr> create_list_space(RectanglePtr, std::list<VerticesPtr>);
  std::list<VerticesPtr> create_list_vertices(RectanglePtr,
      std::list<RectanglePtr>);

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  bool go_to(PointPtr, bool);
  bool go_into(SpacePtr);

private:
  CellPtr starting_cell;
  double robot_size; // = 'cell size' / 2
  ExtendedMapPtr map;
  std::set<CellPtr, CellComp> old_cells;

  bool go_with(VectorPtr, double);
  void boustrophedon_cd();
  bool check(CellPtr);
};

typedef boost::shared_ptr<Boustrophedon> BoustrophedonPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_BOUSTROPHEDON_HPP_ */
