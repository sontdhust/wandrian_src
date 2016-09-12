/*
 * mstc_online.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: manhnh
 */

#include <algorithm>
#include "../../../include/plans/mstc/mstc_online.hpp"

namespace wandrian {
namespace plans {
namespace mstc {

MstcOnline::MstcOnline() :
    tool_size(0) {
}

MstcOnline::~MstcOnline() {
}

void MstcOnline::initialize(PointPtr starting_point, double tool_size,
    CommunicatorPtr communicator) {
  communicator->set_tool_size(tool_size);
  this->tool_size = tool_size;
  this->communicator = communicator;
  // Initialize starting_cell
  starting_cell = IdentifiableCellPtr(
      new IdentifiableCell(
          PointPtr(
              new Point(starting_point->x - tool_size / 2,
                  starting_point->y + tool_size / 2)), 2 * tool_size,
          communicator->get_robot_name()));
  starting_cell->set_parent(
      IdentifiableCellPtr(
          new IdentifiableCell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - 2 * tool_size)),
              2 * tool_size, communicator->get_robot_name())));
  path.insert(path.end(), starting_point);
}

void MstcOnline::cover() {
  communicator->read_message_then_update_old_cells();
  communicator->insert_old_cell(starting_cell);
  std::string message = communicator->create_old_cells_message();
  communicator->write_old_cells_message(message);
  //  communicator->set_current_cell(starting_cell);
  // FIXME
  std::string status = communicator->create_status_message(starting_cell);
  communicator->write_status_message(status);
  scan(starting_cell);
}

void MstcOnline::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool MstcOnline::go_to(PointPtr position, bool flexibility) {
  std::cout << "    pos: " << position->x << "," << position->y;
  return BasePlan::go_to(position, flexibility);
}

bool MstcOnline::see_obstacle(VectorPtr direction, double distance) {
  bool get_obstacle;
  if (behavior_see_obstacle)
    get_obstacle = behavior_see_obstacle(direction, distance);
  else
    get_obstacle = false;
  if (get_obstacle)
    std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
  return get_obstacle;
}

State MstcOnline::state_of(CellPtr cell) {
  State state =
      (communicator->find_old_cell(
          boost::static_pointer_cast<IdentifiableCell>(cell))) ? OLD : NEW;
  if (state == OLD)
    std::cout << " \033[1;45m(OLD)\033[0m\n";
  return state;
}

void MstcOnline::scan(CellPtr current) {
  std::string status;
  communicator->set_current_cell(current);
  // FIXME
  status = communicator->create_status_message(
      boost::static_pointer_cast<IdentifiableCell>(current));
  communicator->write_status_message(status);
  communicator->read_message_then_update_old_cells();
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr direction = (current->get_parent()->get_center()
      - current->get_center()) / 2 / tool_size;
  VectorPtr initial_direction = direction++;
  // While current cell has a new obstacle-free neighboring cell
  bool is_starting_cell = current == starting_cell;
  do {
    // Scan for new neighbor of current cell in counterclockwise order
    IdentifiableCellPtr neighbor = IdentifiableCellPtr(
        new IdentifiableCell(current->get_center() + direction * 2 * tool_size,
            2 * tool_size, communicator->get_robot_name()));
    std::cout << "  \033[1;33mneighbor:\033[0m " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    communicator->read_message_then_update_old_cells();
    if (state_of(neighbor) == OLD) { // Check neighbor with current old cells
      // Go to next sub-cell
      if (communicator->ask_other_robot_still_alive(
          communicator->find_robot_name(neighbor))) {
        // Still alive
        communicator->set_current_cell(current);
        go_with(++direction, tool_size);
        continue;
      } else {
        // Dead
        if (state_of(neighbor) == OLD) { // Check again neighbor with new old cells
          communicator->set_current_cell(current);
          go_with(++direction, tool_size);
          continue;
        } else {
          std::cout << "\n";
          neighbor->set_parent(current);
          communicator->read_message_then_update_old_cells();
          communicator->insert_old_cell(neighbor);
          std::string message = communicator->create_old_cells_message();
          communicator->write_old_cells_message(message);
          communicator->set_current_cell(current);
          go_with(direction++, tool_size);
          scan(neighbor);
          continue;
        }
      }
    }
    if (see_obstacle(direction, tool_size / 2)) { // Obstacle
      // Go to next sub-cell
      communicator->set_current_cell(current);
      go_with(++direction, tool_size);
    } else { // New free neighbor
      std::cout << "\n";
      // Construct a spanning-tree edge
      neighbor->set_parent(current);
      communicator->read_message_then_update_old_cells();
      communicator->insert_old_cell(neighbor);
      std::string message = communicator->create_old_cells_message();
      communicator->write_old_cells_message(message);
      communicator->set_current_cell(current);
      go_with(direction++, tool_size);
      scan(neighbor);
    }
  } while (direction % initial_direction
      != (is_starting_cell ? AT_LEFT_SIDE : IN_FRONT));
  // Back to sub-cell of parent
  if (!is_starting_cell) {
    communicator->set_current_cell(current);
    go_with(direction, tool_size);
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
}

bool MstcOnline::go_with(VectorPtr direction, double distance) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = last_position + direction * distance;

//  CellPtr new_cell = IdentifiableCellPtr(
//        new IdentifiableCell(
//            PointPtr(
//                new Point(new_position->x - tool_size / 2,
//                    new_position->y + tool_size / 2)), 2 * tool_size,
//            communicator->get_robot_name()));
//  communicator->set_current_cell(new_cell);

  bool succeed = go_to(new_position);
  std::cout << "\n";
  return succeed;
}

}
}
}
