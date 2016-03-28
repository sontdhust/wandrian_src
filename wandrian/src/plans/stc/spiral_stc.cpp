/*
 * spiral_stc.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/stc/spiral_stc.hpp"

namespace wandrian {
namespace plans {
namespace stc {

SpiralStc::SpiralStc() :
    tool_size(0) {
}

SpiralStc::~SpiralStc() {
}

void SpiralStc::initialize(PointPtr starting_point, double tool_size) {
  this->tool_size = tool_size;
  // Initialize starting_cell
  starting_cell = CellPtr(
      new Cell(
          PointPtr(
              new Point(starting_point->x - tool_size / 2,
                  starting_point->y + tool_size / 2)), 2 * tool_size));
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - 2 * tool_size)),
              2 * tool_size)));
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

bool SpiralStc::go_to(PointPtr position, bool flexibility) {
  std::cout << "    pos: " << position->x << "," << position->y;
  path.insert(path.end(), position);
  if (behavior_go_to)
    return behavior_go_to(position, flexibility);
  return true;
}

bool SpiralStc::see_obstacle(VectorPtr direction, double distance) {
  bool get_obstacle;
  if (behavior_see_obstacle)
    get_obstacle = behavior_see_obstacle(direction, distance);
  else
    get_obstacle = false;
  if (get_obstacle)
    std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
  return get_obstacle;
}

void SpiralStc::scan(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr direction = (current->get_parent()->get_center()
      - current->get_center()) / 2 / tool_size;
  VectorPtr initial_direction = direction++;
  // While current cell has a new obstacle-free neighboring cell
  bool is_starting_cell = current == starting_cell;
  do {
    // Scan for new neighbor of current cell in counterclockwise order
    CellPtr neighbor = CellPtr(
        new Cell(current->get_center() + direction * 2 * tool_size,
            2 * tool_size));
    std::cout << "  \033[1;33mneighbor:\033[0m " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    if (state_of(neighbor) == OLD) { // Old cell
      // Go to next subcell
      go_with(++direction, tool_size);
      continue;
    }
    if (see_obstacle(direction, tool_size / 2)) { // Obstacle
      // Go to next subcell
      go_with(++direction, tool_size);
    } else { // New free neighbor
      std::cout << "\n";
      // Construct a spanning-tree edge
      neighbor->set_parent(current);
      go_with(direction++, tool_size);
      old_cells.insert(neighbor);
      scan(neighbor);
    }
  } while (direction % initial_direction
      != (is_starting_cell ? AT_RIGHT_SIDE : IN_BACK));
  // Back to subcell of parent
  if (!is_starting_cell) {
    go_with(direction, tool_size);
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
}

State SpiralStc::state_of(CellPtr cell) {
  State state = (old_cells.find(cell) != old_cells.end()) ? OLD : NEW;
  if (state == OLD)
    std::cout << " \033[1;45m(OLD)\033[0m\n";
  return state;
}

bool SpiralStc::go_with(VectorPtr direction, double distance) {
  PointPtr last_position = path.back();
  PointPtr new_position = last_position + direction * distance;
  bool succeed = go_to(new_position, STRICTLY);
  std::cout << "\n";
  return succeed;
}

}
}
}
