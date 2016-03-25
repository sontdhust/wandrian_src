/*
 * full_mstc_online.hpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_PLANS_MSTC_FULL_MSTC_ONLINE_HPP_
#define WANDRIAN_INCLUDE_PLANS_MSTC_FULL_MSTC_ONLINE_HPP_

#include "../../environment/mstc/partially_occupiable_identifiable_cell.hpp"
#include "mstc_online.hpp"

namespace wandrian {
namespace plans {
namespace mstc {

class FullMstcOnline: public MstcOnline {

public:
  FullMstcOnline();
  ~FullMstcOnline();
  void initialize(PointPtr, double, CommunicatorPtr);
  void cover();

protected:
  State state_of(CellPtr);
  void scan(CellPtr);

private:
  PartiallyOccupiableIdentifiableCellPtr starting_cell;

  bool go_from(CellPtr, bool, CellPtr);
  bool visit(CellPtr, Quadrant, bool);
  bool state_of_subcells_of(CellPtr, Orientation);
};

typedef boost::shared_ptr<FullMstcOnline> FullMstcOnlinePtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_MSTC_FULL_MSTC_ONLINE_HPP_ */
