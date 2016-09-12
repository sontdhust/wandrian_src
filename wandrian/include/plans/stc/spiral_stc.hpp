/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_PLANS_STC_SPIRAL_STC_HPP_
#define WANDRIAN_INCLUDE_PLANS_STC_SPIRAL_STC_HPP_

#include "../../common/vector.hpp"
#include "../../environment/cell.hpp"
#include "../base_plan.hpp"

using namespace wandrian::common;
using namespace wandrian::environment;

namespace wandrian {
namespace plans {
namespace stc {

class SpiralStc: public BasePlan {

public:
  SpiralStc();
  ~SpiralStc();
  virtual void initialize(PointPtr, double);
  virtual void cover();

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  std::set<CellPtr, CellComp> old_cells;
  double tool_size; // = 'cell size' / 2

  bool go_to(PointPtr, bool = STRICTLY);
  bool see_obstacle(VectorPtr, double);
  virtual void scan(CellPtr);
  virtual State state_of(CellPtr);

private:
  CellPtr starting_cell;
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;

  bool go_with(VectorPtr, double);
};

typedef boost::shared_ptr<SpiralStc> SpiralStcPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_STC_SPIRAL_STC_HPP_ */
