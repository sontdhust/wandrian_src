/*
 * full_scan_stc.cpp
 *
 *  Created on: Mar 28, 2016
 *      Author: sontd
 */

#include "../../../include/plans/stc/full_scan_stc.hpp"

namespace wandrian {
namespace plans {
namespace stc {

FullScanStc::FullScanStc() {
}

FullScanStc::~FullScanStc() {
}

bool FullScanStc::should_go_to(CellPtr neighbor, VectorPtr direction) {
  if (~direction == AT_RIGHT_SIDE || ~direction == AT_LEFT_SIDE) {
    VectorPtr d = +direction;
    PartiallyOccupiableCellPtr diagonal_neighbor = PartiallyOccupiableCellPtr(
        new PartiallyOccupiableCell(neighbor->get_center() + d * 2 * tool_size,
            2 * tool_size));
    PointPtr diagonal_position = diagonal_neighbor->find_position(&~+d);
    d++;
    PartiallyOccupiableCellPtr vertical_neighbor = PartiallyOccupiableCellPtr(
        new PartiallyOccupiableCell(
            diagonal_neighbor->get_center() + d * 2 * tool_size,
            2 * tool_size));
    PointPtr vertical_position = vertical_neighbor->find_position(&~+d);
    PointPtr current_position = path.back();
    if (state_of(neighbor) != OLD)
      return see_obstacle(diagonal_position - current_position,
          diagonal_position % current_position)
          || see_obstacle(vertical_position - current_position,
              vertical_position % current_position);
    else
      return (state_of_subcells_of(neighbor, ~direction) == DIAGONALLY_OPPOSITE);
  }
  return FullSpiralStc::should_go_to(neighbor, direction);
}

}
}
}
