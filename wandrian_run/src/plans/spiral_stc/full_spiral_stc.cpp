/*
 * full_spiral_stc.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#include "../../../include/plans/spiral_stc/full_spiral_stc.hpp"

#define NORMAL true
#define ABNORMAL false

namespace wandrian {
namespace plans {
namespace spiral_stc {

FullSpiralStc::FullSpiralStc() {
}

FullSpiralStc::~FullSpiralStc() {
}

void FullSpiralStc::cover() {
  old_cells.insert(starting_cell);
  for (int i = I; i <= IV; i++)
    starting_cell->quadrants[i] = OLD;
  scan(starting_cell);
}

bool FullSpiralStc::check(CellPtr cell_to_check) {
  std::set<CellPtr, CellComp>::iterator cell = old_cells.find(cell_to_check);
  return
      (cell != old_cells.end() && (*cell)->quadrants[I] == OLD
          && (*cell)->quadrants[II] == OLD && (*cell)->quadrants[III] == OLD
          && (*cell)->quadrants[IV] == OLD) ? OLD : NEW;
}

void FullSpiralStc::scan(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr orientation = VectorPtr(
      new Vector(
          (*(current->get_parent()->get_center()) - *(current->get_center()))
              / 2 / robot_size));
  VectorPtr initial_orientation = orientation++;
  // While current cell has a new obstacle-free neighboring cell
  bool is_starting_cell = *(current->get_center())
      == *(starting_cell->get_center());
  do {
    // Scan for new neighbor of current cell in counterclockwise order
    CellPtr neighbor = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    *(current->get_center()) + *orientation * 2 * robot_size)),
            2 * robot_size));
    std::cout << "  \033[1;33mneighbor\033[0m: " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    if (check(neighbor) == OLD) { // Old cell
      std::cout << " \033[1;45m(OLD)\033[0m\n";
      continue;
    } else {
      // Go with absolute orientation
      bool succeed = go_across(current, *(orientation) % Vector());
      if (!succeed) { // Obstacle
        std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
      } else { // New free neighbor
        std::cout << "\n";
        // Construct a spanning-tree edge
        neighbor->set_parent(current);
        old_cells.insert(neighbor);
        for (int i = I; i <= IV; i++)
          neighbor->quadrants[i] = OLD;
        scan(neighbor);
      }
    }
  } while (*orientation % *initial_orientation
      != (is_starting_cell ? AT_RIGHT_SIDE : BEHIND));
  // Back to sub-cell of parent (need to check sub-cell of parent is occupied or not)
  if (!is_starting_cell) {
    go_across(current, *(orientation) % Vector());
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
}

bool FullSpiralStc::go_across(CellPtr cell, Orientation orientation) {
//  PointPtr last_position = *(--path.end());
//  PointPtr new_position = PointPtr(
//      new Point(*last_position + *orientation * 2 * STEP_SIZE));
//  return go_to(new_position, STRICTLY);
  return true;
}

}
}
}
