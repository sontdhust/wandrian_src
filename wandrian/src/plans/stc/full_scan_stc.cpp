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
    // TODO: Correctly inspect subcells' state of neighbor cells
    if (state_of(neighbor) != OLD) { // 'neighbor' is new cell
      VectorPtr d = VectorPtr(new Vector(direction));
      PartiallyOccupiableCellPtr current_cell = PartiallyOccupiableCellPtr(
          new PartiallyOccupiableCell(
              neighbor->get_center() + -(-d) * 2 * tool_size, 2 * tool_size));
      PointPtr position = current_cell->find_position(&~+d);
      PointPtr neighbor_position = position + d++ * tool_size;
      PointPtr diagonal_position = neighbor_position + d++ * tool_size;
      PointPtr vertical_position = diagonal_position + d * tool_size;
      // See central point of obstacles, not boundary
      PointPtr current_position = path.back();
      return see_obstacle(position - current_position,
          position % current_position)
          || see_obstacle(neighbor_position - current_position,
              neighbor_position % current_position)
          || see_obstacle(vertical_position - current_position,
              vertical_position % current_position)
          || see_obstacle(diagonal_position - current_position,
              diagonal_position % current_position);
    } else
      // 'neighbor' is old cell
      return (state_of_subcells_of(neighbor, ~direction) == DIAGONALLY_OPPOSITE);
  }
  return FullSpiralStc::should_go_to(neighbor, direction);
}

}
}
}
