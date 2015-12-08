/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_

#include "../base_plan.hpp"
#include "cell.hpp"
#include "../../common/vector.hpp"

#define STEP_SIZE (robot_size / 2)

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

class SpiralStc: public BasePlan {

public:
  SpiralStc();
  ~SpiralStc();
  void initialize(PointPtr, double);
  void cover();

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  CellPtr starting_cell;
  std::set<CellPtr, CellComp> old_cells;
  double robot_size; // = 'cell size' / 2

  bool go_to(PointPtr, bool);
  bool see_obstacle(VectorPtr, double);
  bool go_with(VectorPtr, double);
  bool check(CellPtr);
  virtual void scan(CellPtr);

private:
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
};

typedef boost::shared_ptr<SpiralStc> SpiralStcPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_ */
