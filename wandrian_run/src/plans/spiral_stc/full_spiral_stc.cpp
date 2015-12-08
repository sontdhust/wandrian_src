/*
 * full_spiral_stc.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#include "../../../include/plans/spiral_stc/full_spiral_stc.hpp"
#include "../../../include/common/global.hpp"

#define NORMAL true
#define ABNORMAL false

namespace wandrian {
namespace plans {
namespace spiral_stc {

FullSpiralStc::FullSpiralStc() {
}

FullSpiralStc::~FullSpiralStc() {
}

void FullSpiralStc::scan(CellPtr current) {
  flexibly_scan(current, NORMAL);
}

bool FullSpiralStc::flexibly_scan(CellPtr current, bool is_normal) {
#if 0
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
  << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr orientation = ++VectorPtr(
      new Vector(
          (*(current->get_parent()->get_center()) - *(current->get_center()))
          / 2 / robot_size));
  if (is_normal)
  current->quadrant4 = OLD;
  else
  current->quadrant3 = OLD;
  Orientation current_neighbor = AT_RIGHT_SIDE;
#endif

#if 1
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr orientation = ++VectorPtr(
      new Vector(
          (*(current->get_parent()->get_center()) - *(current->get_center()))
              / 2 / robot_size));
  Orientation current_neighbor = AT_RIGHT_SIDE;
  // While current cell has a new obstacle-free neighboring cell
  bool is_starting_cell = *(current->get_center())
      == *(starting_cell->get_center());
  do {
    // Scan for the first new neighbor of current cell in counterclockwise order
    CellPtr neighbor = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    *(current->get_center()) + *orientation * 2 * robot_size)),
            2 * robot_size));
    std::cout << "  \033[1;33mneighbor\033[0m: " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    if (check(neighbor) == OLD) {
      std::cout << " \033[1;45m(OLD)\033[0m\n";
      // Go to next sub-cell (need to check next sub-cell is occupied or not)
      go_with(++orientation, robot_size / STEP_SIZE);
      continue;
    }
    if (see_obstacle(orientation, (robot_size / 2) / STEP_SIZE)) {
      std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
      // Go to next sub-cell (need to check next sub-cell is occupied or not)
      go_with(++orientation, robot_size / STEP_SIZE);
    } else { // New free neighbor
      std::cout << "\n";
      // Construct a spanning-tree edge
      neighbor->set_parent(current);
      go_with(orientation++, robot_size / STEP_SIZE);
      old_cells.insert(neighbor);
      for (int i = I; i <= IV; i++)
        neighbor->quadrants[i] = OLD;
      scan(neighbor);
    }
  } while (current_neighbor++ != (is_starting_cell ? BEHIND : AT_LEFT_SIDE));
  // Back to sub-cell of parent (need to check sub-cell of parent is occupied or not)
  if (!is_starting_cell) {
    go_with(orientation, robot_size / STEP_SIZE);
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
#endif
  return NORMAL;
}

}
}
}
