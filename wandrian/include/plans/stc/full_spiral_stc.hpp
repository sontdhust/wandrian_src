/*
 * full_spiral_stc.hpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_PLANS_STC_FULL_SPIRAL_STC_HPP_
#define WANDRIAN_INCLUDE_PLANS_STC_FULL_SPIRAL_STC_HPP_

#include "../../environment/partially_occupiable_cell.hpp"
#include "spiral_stc.hpp"

#define DIAGONALLY_OPPOSITE true
#define NON_DIAGONALLY_OPPOSITE false

using namespace wandrian::environment;

namespace wandrian {
namespace plans {
namespace stc {

class FullSpiralStc : public SpiralStc {

public:
  FullSpiralStc();
  ~FullSpiralStc();
  void initialize(PointPtr, double);
  void cover();

protected:
  void scan(CellPtr);
  State state_of(CellPtr);
  bool state_of_subcells_of(CellPtr, Orientation);
  virtual bool should_go_to(CellPtr, VectorPtr);

private:
  PartiallyOccupiableCellPtr starting_cell;

  bool go_from(CellPtr, bool, CellPtr);
  bool visit(CellPtr, Quadrant, bool = STRICTLY);
};

typedef boost::shared_ptr<FullSpiralStc> FullSpiralStcPtr;
}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_STC_FULL_SPIRAL_STC_HPP_ */
