/*
 * full_spiral_stc.hpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_FULL_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_FULL_SPIRAL_STC_HPP_

#include "spiral_stc.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {

class FullSpiralStc: public SpiralStc {

public:
  FullSpiralStc();
  ~FullSpiralStc();
  void cover();

protected:
  bool check(CellPtr);
  void scan(CellPtr);

private:
  bool go_across(CellPtr, CellPtr);
  bool visit(CellPtr, Quadrant, bool);
};

typedef boost::shared_ptr<FullSpiralStc> FullSpiralStcPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_FULL_SPIRAL_STC_HPP_ */
