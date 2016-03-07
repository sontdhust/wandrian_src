/*
 * full_spiral_stc.hpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_FULL_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_FULL_SPIRAL_STC_HPP_

#include "spiral_stc.hpp"
#include "partially_occupiable_cell.hpp"

namespace wandrian {
namespace plans {
namespace mstc_online {

class FullSpiralStc: public SpiralStc {

public:
  FullSpiralStc();
  ~FullSpiralStc();
  virtual void initialize(PointPtr, double);
  virtual void cover();

//  std::set<CellPtr, CellComp> *old_cells;

protected:
  virtual State state_of(CellPtr);
  virtual void scan(CellPtr);

private:
  PartiallyOccupiableCellPtr starting_cell;

  bool go_from(CellPtr, bool, CellPtr);
  bool visit(CellPtr, Quadrant, bool);
  bool state_of_subcells_of(CellPtr, Orientation);
};

typedef boost::shared_ptr<FullSpiralStc> FullSpiralStcPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_FULL_SPIRAL_STC_HPP_ */
