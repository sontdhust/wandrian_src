/*
 * full_spiral_stc.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#include "../../../include/plans/stc/full_spiral_stc.hpp"

#define PASS true
#define DONT_PASS false

namespace wandrian {
namespace plans {
namespace stc {

FullSpiralStc::FullSpiralStc() {
}

FullSpiralStc::~FullSpiralStc() {
}

void FullSpiralStc::initialize(PointPtr starting_point, double tool_size) {
  this->tool_size = tool_size;
  // Initialize starting_cell
  starting_cell = PartiallyOccupiableCellPtr(
      new PartiallyOccupiableCell(
          PointPtr(
              new Point(starting_point->x - tool_size / 2,
                  starting_point->y + tool_size / 2)), 2 * tool_size));
  starting_cell->set_parent(
      PartiallyOccupiableCellPtr(
          new PartiallyOccupiableCell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - 2 * tool_size)),
              2 * tool_size)));
  path.insert(path.end(), starting_point);
}

void FullSpiralStc::cover() {
  old_cells.insert(starting_cell);
  starting_cell->set_current_quadrant(IV);
  scan(starting_cell);
}

void FullSpiralStc::scan(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr direction = (current->get_parent()->get_center()
      - current->get_center()) / 2 / tool_size;
  VectorPtr initial_direction = direction++;
  // Check current cell has diagonally opposite obstacles or not
  PartiallyOccupiableCellPtr c = boost::static_pointer_cast<
      PartiallyOccupiableCell>(current);
  Quadrant q = c->get_current_quadrant();
  Orientation orientation;
  if (q == I)
    orientation = AT_LEFT_SIDE;
  else if (q == II)
    orientation = IN_BACK;
  else if (q == III)
    orientation = AT_RIGHT_SIDE;
  else if (q == IV)
    orientation = IN_FRONT;
  c->set_quadrants_state(+q,
      see_obstacle(~orientation, tool_size / 2) ? OBSTACLE : NEW);
  c->set_quadrants_state(-q,
      see_obstacle(~++orientation, tool_size / 2) ? OBSTACLE : NEW);
  // While current cell has a new obstacle-free neighboring cell
  bool is_starting_cell = current == starting_cell;
  do {
    // Scan for new neighbor of current cell in counterclockwise order
    PartiallyOccupiableCellPtr neighbor = PartiallyOccupiableCellPtr(
        new PartiallyOccupiableCell(
            current->get_center() + direction * 2 * tool_size, 2 * tool_size));
    std::cout << "  \033[1;33mneighbor:\033[0m " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    // go_from(current, DONT_PASS, neighbor); // Full Scan-STC preparing
    if (should_go_to(neighbor, direction)) {
      // Go to free subcell of neighbor
      bool succeed = go_from(current, PASS, neighbor);
      if (!succeed) { // Obstacle
      } else { // New free neighbor
        // Construct a spanning-tree edge
        neighbor->set_parent(current);
        old_cells.insert(neighbor);
        scan(neighbor);
      }
    } else {
      // Go to next subcell
      go_from(current, DONT_PASS, neighbor);
      continue;
    }
  } while (direction++ % initial_direction
      != (is_starting_cell ? IN_FRONT : AT_RIGHT_SIDE));
  // Back to subcell of parent
  if (!is_starting_cell) {
    go_from(current, PASS, current->get_parent());
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
}

State FullSpiralStc::state_of(CellPtr cell) {
  State state = (old_cells.find(cell) != old_cells.end()) ? OLD : NEW;
  if (state == OLD)
    std::cout << " \033[1;45m(OLD)\033[0m\n";
  return state;
}

bool FullSpiralStc::state_of_subcells_of(CellPtr cell,
    Orientation orientation) {
  std::set<CellPtr, CellComp>::iterator i = old_cells.find(cell);
  if (i == old_cells.end()) // Assume new cell is diagonally occupied by opposite subcells
    return DIAGONALLY_OPPOSITE;
  PartiallyOccupiableCellPtr c = boost::static_pointer_cast<
      PartiallyOccupiableCell>(*i);
  Quadrant q;
  if (orientation == AT_LEFT_SIDE)
    q = I;
  else if (orientation == IN_BACK)
    q = II;
  else if (orientation == AT_RIGHT_SIDE)
    q = III;
  else if (orientation == IN_FRONT)
    q = IV;
  return
      (c->get_quadrants()[q] == NEW && c->get_quadrants()[+q] == OBSTACLE
          && c->get_quadrants()[-q] == OBSTACLE)
          || (c->get_quadrants()[--q] == NEW
              && c->get_quadrants()[+q] == OBSTACLE
              && c->get_quadrants()[-q] == OBSTACLE) ?
      DIAGONALLY_OPPOSITE :
                                                       NON_DIAGONALLY_OPPOSITE;
}

bool FullSpiralStc::should_go_to(CellPtr neighbor, VectorPtr direction) {
  return (state_of(neighbor) != OLD)
      || (state_of_subcells_of(neighbor, ~direction) == DIAGONALLY_OPPOSITE);
}

bool FullSpiralStc::go_from(CellPtr current, bool need_to_pass, CellPtr next) {
  VectorPtr direction = (next->get_center() - current->get_center())
      / (2 * tool_size);
  PartiallyOccupiableCellPtr c = boost::static_pointer_cast<
      PartiallyOccupiableCell>(current);
  PartiallyOccupiableCellPtr n = boost::static_pointer_cast<
      PartiallyOccupiableCell>(next);
  Quadrant quadrant = c->get_current_quadrant();
  Quadrant q;
  Quadrant q1;
  Quadrant q2;
  Quadrant q3;
  Quadrant q4;
  if (~direction == AT_RIGHT_SIDE)
    q = IV;
  else if (~direction == IN_FRONT)
    q = I;
  else if (~direction == AT_LEFT_SIDE)
    q = II;
  else if (~direction == IN_BACK)
    q = III;
  q1 = q;
  q2 = ++q;
  q3 = ++q;
  q4 = ++q;
  if (quadrant == q1 || quadrant == q2) {
    if (!see_obstacle(direction, tool_size / 2)) {
      if (need_to_pass == DONT_PASS)
        return true;
      bool succeed = visit(next, quadrant == q1 ? q4 : q3, STRICTLY);
      std::cout << "\n";
      return succeed;
    } else {
      n->set_quadrants_state(q1 ? q4 : q3, OBSTACLE);
      if (!see_obstacle(quadrant == q1 ? ++direction : --direction,
          tool_size / 2)) {
        visit(current, quadrant == q1 ? q2 : q1, STRICTLY);
        if (!see_obstacle(quadrant == q1 ? --direction : ++direction,
            tool_size / 2)) {
          if (need_to_pass == DONT_PASS)
            return true;
          bool succeed = visit(next, quadrant == q1 ? q3 : q4, STRICTLY);
          std::cout << "\n";
          return succeed;
        } else {
          n->set_quadrants_state(q1 ? q3 : q4, OBSTACLE);
          return false;
        }
      } else {
        c->set_quadrants_state(q1 ? q2 : q1, OBSTACLE);
        return false;
      }
    }
  } else if (quadrant == q4 || quadrant == q3) {
    if (!see_obstacle(direction, tool_size / 2)) {
      visit(c, quadrant == q4 ? q1 : q2, STRICTLY);
      return go_from(c, need_to_pass, next);
    } else {
      c->set_quadrants_state(quadrant == q4 ? q1 : q2, OBSTACLE);
      if (!see_obstacle(quadrant == q4 ? ++direction : --direction,
          tool_size / 2)) {
        visit(c, quadrant == q4 ? q3 : q4, STRICTLY);
        if (!see_obstacle(quadrant == q4 ? --direction : ++direction,
            tool_size / 2)) {
          visit(c, quadrant == q4 ? q2 : q1, STRICTLY);
          return go_from(c, need_to_pass, next);
        } else {
          c->set_quadrants_state(quadrant == q4 ? q2 : q1, OBSTACLE);
          return false;
        }
      } else {
        c->set_quadrants_state(quadrant == q4 ? q3 : q4, OBSTACLE);
        return false;
      }
    }
  } else
    return false;
}

bool FullSpiralStc::visit(CellPtr cell, Quadrant quadrant, bool flexibility) {
  PartiallyOccupiableCellPtr c = boost::static_pointer_cast<
      PartiallyOccupiableCell>(cell);
  c->set_current_quadrant(quadrant);
  return go_to(c->get_current_position(), flexibility);
}

}
}
}
