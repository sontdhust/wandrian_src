/*
 * spiral_stc.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/spiral_stc/spiral_stc.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {

SpiralStc::SpiralStc() :
    robot_size(0) {
}

SpiralStc::~SpiralStc() {
}

void SpiralStc::initialize(PointPtr starting_point, double robot_size) {
  this->robot_size = robot_size;
  // Initialize starting_cell
  starting_cell = CellPtr(
      new Cell(
          PointPtr(
              new Point(starting_point->x - robot_size / 2,
                  starting_point->y + robot_size / 2)), 2 * robot_size));
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - 2 * robot_size)),
              2 * robot_size)));
  path.insert(path.end(), starting_point);
}

void SpiralStc::cover() {
  old_cells.insert(starting_cell);
  scan(starting_cell);
}

void SpiralStc::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool SpiralStc::go_to(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  path.insert(path.end(), position);
  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}

bool SpiralStc::see_obstacle(VectorPtr orientation, double step) {
  if (behavior_see_obstacle)
    return behavior_see_obstacle(orientation, step);
  return false;
}

bool SpiralStc::check(CellPtr cell_to_check) {
  return (old_cells.find(cell_to_check) != old_cells.end()) ? OLD : NEW;
}

void SpiralStc::scan(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr orientation = (current->get_parent()->get_center()
      - current->get_center()) / 2 / robot_size;
  VectorPtr initial_orientation = orientation++;
  // While current cell has a new obstacle-free neighboring cell
  bool is_starting_cell = current->get_center() == starting_cell->get_center();
  do {
    // Scan for new neighbor of current cell in counterclockwise order
    CellPtr neighbor = CellPtr(
        new Cell(current->get_center() + orientation * 2 * robot_size,
            2 * robot_size));
    std::cout << "  \033[1;33mneighbor:\033[0m " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    if (check(neighbor) == OLD) { // Old cell
      std::cout << " \033[1;45m(OLD)\033[0m\n";
      // Go to next sub-cell (need to check next sub-cell is occupied or not)
      go_with(++orientation, robot_size / STEP_SIZE);
      continue;
    }
    if (see_obstacle(orientation, (robot_size / 2) / STEP_SIZE)) { // Obstacle
      std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
      // Go to next sub-cell (need to check next sub-cell is occupied or not)
      go_with(++orientation, robot_size / STEP_SIZE);
    } else { // New free neighbor
      std::cout << "\n";
      // Construct a spanning-tree edge
      neighbor->set_parent(current);
      go_with(orientation++, robot_size / STEP_SIZE);
      old_cells.insert(neighbor);
      scan(neighbor);
    }
  } while (orientation % initial_orientation
      != (is_starting_cell ? AT_RIGHT_SIDE : BEHIND));
  // Back to sub-cell of parent (need to check sub-cell of parent is occupied or not)
  if (!is_starting_cell) {
    go_with(orientation, robot_size / STEP_SIZE);
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
}

bool SpiralStc::go_with(VectorPtr orientation, double step) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = last_position + orientation * step * STEP_SIZE;
  return go_to(new_position, STRICTLY);
}

}
}
}
