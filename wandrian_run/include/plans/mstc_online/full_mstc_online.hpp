/*
 * full_spiral_stc.hpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_FULL_MSTC_ONLINE_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_FULL_MSTC_ONLINE_HPP_

#include "mstc_online.hpp"
#include "partially_occupiable_cell.hpp"

namespace wandrian {
namespace plans {
namespace mstc_online {

class FullMstcOnline: public MstcOnline {

public:
  FullMstcOnline();
  ~FullMstcOnline();
  virtual void initialize(PointPtr, double);
  virtual void cover();

protected:
  virtual State state_of(CellPtr);
  virtual void scan(CellPtr);

private:
  PartiallyOccupiableCellPtr starting_cell;

  bool go_from(CellPtr, bool, CellPtr);
  bool visit(CellPtr, Quadrant, bool);
  bool state_of_subcells_of(CellPtr, Orientation);
};

typedef boost::shared_ptr<FullMstcOnline> FullMstcOnlinePtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_FULL_MSTC_ONLINE_HPP_ */
