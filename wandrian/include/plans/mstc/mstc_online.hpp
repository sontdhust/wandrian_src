/*
 * mstc_online.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_INCLUDE_PLANS_MSTC_MSTC_ONLINE_HPP_
#define WANDRIAN_INCLUDE_PLANS_MSTC_MSTC_ONLINE_HPP_

#include "../../common/vector.hpp"
#include "../../environment/mstc/communicator.hpp"
#include "../../environment/mstc/identifiable_cell.hpp"
#include "../base_plan.hpp"

using namespace wandrian::common;
using namespace wandrian::environment;
using namespace wandrian::environment::mstc;

namespace wandrian {
namespace plans {
namespace mstc {

class MstcOnline : public BasePlan {

public:
  MstcOnline();
  ~MstcOnline();
  virtual void initialize(PointPtr, double, CommunicatorPtr);
  virtual void cover();

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  double tool_size; // = 'cell size' / 2
  CommunicatorPtr communicator;

  bool go_to(PointPtr, bool = STRICTLY);
  bool see_obstacle(VectorPtr, double);
  virtual State state_of(CellPtr);
  virtual void scan(CellPtr);

private:
  IdentifiableCellPtr starting_cell;
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;

  bool go_with(VectorPtr, double);
};

typedef boost::shared_ptr<MstcOnline> MstcOnlinePtr;
}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_MSTC_MSTC_ONLINE_HPP_ */
