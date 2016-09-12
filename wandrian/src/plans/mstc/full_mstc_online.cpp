/*
 * full_mstc_online.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: cslab
 */

#include "../../../include/plans/mstc/full_mstc_online.hpp"
#include "../../../include/environment/mstc/communicator.hpp"

#define PASS true
#define DONT_PASS false
#define DIAGONALLY_OPPOSITE_OBSTACLES true
#define NON_DIAGONALLY_OPPOSITE_OBSTACLES false

using namespace wandrian::environment::mstc;

namespace wandrian {
namespace plans {
namespace mstc {

FullMstcOnline::FullMstcOnline() {
}

FullMstcOnline::~FullMstcOnline() {
}

void FullMstcOnline::initialize(PointPtr starting_point, double tool_size,
    CommunicatorPtr communicator) {
  this->tool_size = tool_size;
  this->communicator = communicator;
  // Initialize starting_cell
  starting_cell = PartiallyOccupiableIdentifiableCellPtr(
      new PartiallyOccupiableIdentifiableCell(
          PointPtr(
              new Point(starting_point->x - tool_size / 2,
                  starting_point->y + tool_size / 2)), 2 * tool_size,
          communicator->get_robot_name()));
  starting_cell->set_parent(
      PartiallyOccupiableIdentifiableCellPtr(
          new PartiallyOccupiableIdentifiableCell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - 2 * tool_size)),
              2 * tool_size, communicator->get_robot_name())));
  path.insert(path.end(), starting_point);
}

void FullMstcOnline::cover() {
  communicator->cells.insert(starting_cell);
  starting_cell->set_current_quadrant(IV);
  scan(starting_cell);
}

State FullMstcOnline::state_of(CellPtr cell) {
  State state =
      (communicator->cells.find(
          boost::static_pointer_cast<IdentifiableCell>(cell))
          != communicator->cells.end()) ? OLD : NEW;
  if (state == OLD)
    std::cout << " \033[1;45m(OLD)\033[0m\n";
  return state;
}

void FullMstcOnline::scan(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr orientation = (current->get_parent()->get_center()
      - current->get_center()) / 2 / tool_size;
  VectorPtr initial_orientation = orientation++;
  // Check current cell has diagonally opposite obstacles or not
  PartiallyOccupiableIdentifiableCellPtr c = boost::dynamic_pointer_cast<
      PartiallyOccupiableIdentifiableCell>(current);
  Quadrant q = c->get_current_quadrant();
  Orientation o;
  if (q == I)
    o = AT_LEFT_SIDE;
  else if (q == II)
    o = IN_BACK;
  else if (q == III)
    o = AT_RIGHT_SIDE;
  else if (q == IV)
    o = IN_FRONT;
  c->set_quadrants_state(+q, see_obstacle(~o, tool_size / 2) ? OBSTACLE : NEW);
  c->set_quadrants_state(-q,
      see_obstacle(~++o, tool_size / 2) ? OBSTACLE : NEW);
  // While current cell has a new obstacle-free neighboring cell
  bool is_starting_cell = current == starting_cell;
  do {
    // Scan for new neighbor of current cell in counterclockwise order
    PartiallyOccupiableIdentifiableCellPtr neighbor =
        PartiallyOccupiableIdentifiableCellPtr(
            new PartiallyOccupiableIdentifiableCell(
                current->get_center() + orientation * 2 * tool_size,
                2 * tool_size, communicator->get_robot_name()));
    std::cout << "  \033[1;33mneighbor:\033[0m " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    if (state_of(neighbor) == OLD
        && (state_of_subcells_of(neighbor, ~orientation)
            == NON_DIAGONALLY_OPPOSITE_OBSTACLES)) { // Old cell
      // Go to next sub-cell
      go_from(current, DONT_PASS, neighbor);
      continue;
    } else {
      // Go with absolute orientation
      bool succeed = go_from(current, PASS, neighbor);
      if (!succeed) { // Obstacle
      } else { // New free neighbor
        // Construct a spanning-tree edge
        neighbor->set_parent(current);
        communicator->cells.insert(neighbor);
        scan(neighbor);
      }
    }
  } while (orientation++ % initial_orientation
      != (is_starting_cell ? IN_FRONT : AT_RIGHT_SIDE));
  // Back to sub-cell of parent (need to check sub-cell of parent is occupied or not)
  if (!is_starting_cell) {
    go_from(current, PASS, current->get_parent());
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
}

bool FullMstcOnline::go_from(CellPtr current, bool pass, CellPtr next) {
  VectorPtr orientation = (next->get_center() - current->get_center())
      / (2 * tool_size);
  PartiallyOccupiableIdentifiableCellPtr c = boost::dynamic_pointer_cast<
      PartiallyOccupiableIdentifiableCell>(current);
  PartiallyOccupiableIdentifiableCellPtr n = boost::dynamic_pointer_cast<
      PartiallyOccupiableIdentifiableCell>(next);
  Quadrant quadrant = c->get_current_quadrant();
  Quadrant q;
  Quadrant q1;
  Quadrant q2;
  Quadrant q3;
  Quadrant q4;
  if (~orientation == AT_RIGHT_SIDE)
    q = IV;
  else if (~orientation == IN_FRONT)
    q = I;
  else if (~orientation == AT_LEFT_SIDE)
    q = II;
  else if (~orientation == IN_BACK)
    q = III;
  q1 = q;
  q2 = ++q;
  q3 = ++q;
  q4 = ++q;
  if (quadrant == q1 || quadrant == q2) {
    if (!see_obstacle(orientation, tool_size / 2)) {
      if (pass == DONT_PASS)
        return true;
      bool succeed = visit(next, quadrant == q1 ? q4 : q3);
      std::cout << "\n";
      return succeed;
    } else {
      n->set_quadrants_state(q1 ? q4 : q3, OBSTACLE);
      if (!see_obstacle(quadrant == q1 ? ++orientation : --orientation,
          tool_size / 2)) {
        visit(current, quadrant == q1 ? q2 : q1);
        if (!see_obstacle(quadrant == q1 ? --orientation : ++orientation,
            tool_size / 2)) {
          if (pass == DONT_PASS)
            return true;
          bool succeed = visit(next, quadrant == q1 ? q3 : q4);
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
    if (!see_obstacle(orientation, tool_size / 2)) {
      visit(c, quadrant == q4 ? q1 : q2);
      return go_from(c, pass, next);
    } else {
      c->set_quadrants_state(quadrant == q4 ? q1 : q2, OBSTACLE);
      if (!see_obstacle(quadrant == q4 ? ++orientation : --orientation,
          tool_size / 2)) {
        visit(c, quadrant == q4 ? q3 : q4);
        if (!see_obstacle(quadrant == q4 ? --orientation : ++orientation,
            tool_size / 2)) {
          visit(c, quadrant == q4 ? q2 : q1);
          return go_from(c, pass, next);
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

bool FullMstcOnline::visit(CellPtr cell, Quadrant quadrant, bool flexibly) {
  PartiallyOccupiableIdentifiableCellPtr c = boost::dynamic_pointer_cast<
      PartiallyOccupiableIdentifiableCell>(cell);
  c->set_current_quadrant(quadrant);
  return go_to(c->get_current_position(), flexibly);
}

bool FullMstcOnline::state_of_subcells_of(CellPtr cell,
    Orientation orientation) {
  PartiallyOccupiableIdentifiableCellPtr c = boost::dynamic_pointer_cast<
      PartiallyOccupiableIdentifiableCell>(
      *communicator->cells.find(
          boost::static_pointer_cast<IdentifiableCell>(cell)));
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
          DIAGONALLY_OPPOSITE_OBSTACLES : NON_DIAGONALLY_OPPOSITE_OBSTACLES;
}

}
}
}
