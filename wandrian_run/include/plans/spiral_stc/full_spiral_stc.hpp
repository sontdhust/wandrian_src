/*
 * full_spiral_stc.hpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_FULL_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_FULL_SPIRAL_STC_HPP_

#include "spiral_stc.hpp"
#include "../../environment/partially_occupiable_cell.hpp"

using namespace wandrian::environment;

namespace wandrian {
namespace plans {
namespace spiral_stc {

class FullSpiralStc: public SpiralStc {

public:
  FullSpiralStc();
  ~FullSpiralStc();
  void initialize(PointPtr, double);
  void cover();

protected:
  State state_of(CellPtr);
  void scan(CellPtr);

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

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_FULL_SPIRAL_STC_HPP_ */
