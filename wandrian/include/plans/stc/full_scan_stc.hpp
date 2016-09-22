/*
 * full_scan_stc.hpp
 *
 *  Created on: Mar 28, 2016
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_PLANS_STC_FULL_SCAN_STC_HPP_
#define WANDRIAN_INCLUDE_PLANS_STC_FULL_SCAN_STC_HPP_

#include "full_spiral_stc.hpp"

using namespace wandrian::environment;

namespace wandrian {
namespace plans {
namespace stc {

class FullScanStc : public FullSpiralStc {

public:
  FullScanStc();
  ~FullScanStc();

protected:
  bool should_go_to(CellPtr, VectorPtr);
};

typedef boost::shared_ptr<FullScanStc> FullScanStcPtr;
}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_STC_FULL_SCAN_STC_HPP_ */
