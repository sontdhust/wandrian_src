/*
 * online_boustrophedon.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/online_boustrophedon/online_boustrophedon.hpp"

namespace wandrian {
namespace plans {
namespace online_boustrophedon {

int check_rotate = -1;
bool straight = true;

OnlineBoustrophedon::OnlineBoustrophedon() :
    robot_size(0) {
}

OnlineBoustrophedon::~OnlineBoustrophedon() {
}

void OnlineBoustrophedon::initialize(PointPtr starting_point,
    double robot_size) {
  this->robot_size = robot_size;
  // Initialize starting_cell
  starting_cell = CellPtr(
      new Cell(PointPtr(new Point(starting_point->x, starting_point->y)),
          robot_size));
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - robot_size)),
              robot_size)));
  path.insert(path.end(), starting_point);
}

void OnlineBoustrophedon::cover() {
  old_cells.insert(starting_cell);
  online_boustrophedon(starting_cell);
}

void OnlineBoustrophedon::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool OnlineBoustrophedon::go_to(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  path.insert(path.end(), position);

  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}

bool OnlineBoustrophedon::see_obstacle(VectorPtr orientation, double step) {
  if (behavior_see_obstacle)
    return behavior_see_obstacle(orientation, step);
  return false;
}

bool OnlineBoustrophedon::go_with(VectorPtr orientation, double step) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = PointPtr(
      new Point(*last_position + *orientation * step * robot_size / 2));
  return go_to(new_position, STRICTLY);
}
void OnlineBoustrophedon::go_straight(CellPtr neighbor, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor->set_parent(current);
  current->neighbors.insert(current->neighbors.end(), neighbor);
  old_cells.insert(neighbor);
  go_with(orientation, 2);
  online_boustrophedon(neighbor);
}
void OnlineBoustrophedon::turn_left(CellPtr neighbor_left, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor_left->set_parent(current);
  current->neighbors.insert(current->neighbors.end(), neighbor_left);
  old_cells.insert(neighbor_left);
  go_with(orientation->rotate_counterclockwise_left(), 2);
  check_rotate = -1;
  online_boustrophedon(neighbor_left);
}
void OnlineBoustrophedon::turn_right(CellPtr neighbor_right, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor_right->set_parent(current);
  current->neighbors.insert(current->neighbors.end(), neighbor_right);
  old_cells.insert(neighbor_right);
  go_with(orientation->rotate_counterclockwise_right(), 2);
  check_rotate = 1;
  online_boustrophedon(neighbor_right);
}
void OnlineBoustrophedon::online_boustrophedon(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr orientation = VectorPtr(
      new Vector(
          (*(current->get_parent()->get_center()) - *(current->get_center()))
              / robot_size));
  CellPtr neighbor;
  CellPtr neighbor_left;
  CellPtr neighbor_right;
  // // While current cell has a new obstacle-free neighboring cell
  // bool is_starting_cell = *(current->get_center())
  //     == *(starting_cell->get_center());
  // quay vector 180 do
  orientation = orientation->rotate_counterclockwise_180();
  // Scan for the first new neighbor of current cell in counterclockwise order
  neighbor = CellPtr(
      new Cell(
          PointPtr(
              new Point(*(current->get_center()) + *orientation * robot_size)),
          robot_size));
  neighbor_left = CellPtr(
      new Cell(
          PointPtr(
              new Point(
                  *(current->get_center())
                      + *orientation->rotate_counterclockwise_left()
                          * robot_size)), robot_size));
  neighbor_right = CellPtr(
      new Cell(
          PointPtr(
              new Point(
                  *(current->get_center())
                      + *orientation->rotate_counterclockwise_right()
                          * robot_size)), robot_size));

  std::cout << "  \033[1;33mneighbor\033[0m: " << neighbor->get_center()->x
      << "," << neighbor->get_center()->y;
  if (straight == false) {
    if (check_rotate == 1) {
      if (see_obstacle(orientation->rotate_counterclockwise_right(), 1)
          == false&& check(neighbor_right) != OLD_CELL) {
        straight = true;
        turn_right(neighbor_right, current, orientation);
      } else if (see_obstacle(orientation->rotate_counterclockwise_left(), 1)
          == false && check(neighbor_left) != OLD_CELL) {
        straight = true;
        turn_left(neighbor_left, current, orientation);
      } else if (see_obstacle(orientation, 1)
          == false&& check(neighbor) != OLD_CELL) {
        go_straight(neighbor, current, orientation);
      }
    } else {
      if (see_obstacle(orientation->rotate_counterclockwise_left(), 1)
          == false&& check(neighbor_left) != OLD_CELL) {
        straight = true;
        turn_left(neighbor_left, current, orientation);
      } else if (see_obstacle(orientation->rotate_counterclockwise_right(), 1)
          == false && check(neighbor_right) != OLD_CELL) {
        straight = true;
        turn_right(neighbor_right, current, orientation);
      } else if (see_obstacle(orientation, 1)
          == false&& check(neighbor) != OLD_CELL) {
        go_straight(neighbor, current, orientation);
      }
    }
  } else {
    if (see_obstacle(orientation, 1) || check(neighbor) == OLD_CELL) { // TODO: Check obstacle here
      std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
      // Go to next sub-cell
      if (check_rotate == 1) {
        if (see_obstacle(orientation->rotate_counterclockwise_left(), 1)
            == false && check(neighbor_left) != OLD_CELL) {
          straight = false;
          turn_left(neighbor_left, current, orientation);
        } else if (see_obstacle(orientation->rotate_counterclockwise_right(), 1)
            == false && check(neighbor_right) != OLD_CELL) {
          straight = false;
          turn_right(neighbor_right, current, orientation);
        }

      } else if (check_rotate == -1) {
        if (see_obstacle(orientation->rotate_counterclockwise_right(), 1)
            == false && check(neighbor_right) != OLD_CELL) {
          straight = false;
          turn_right(neighbor_right, current, orientation);
        } else if (see_obstacle(orientation->rotate_counterclockwise_left(), 1)
            == false && check(neighbor_left) != OLD_CELL) {
          straight = false;
          turn_left(neighbor_left, current, orientation);
        }
      }
    } else if (see_obstacle(orientation, 1)
        == false&& check(neighbor) != OLD_CELL) { // New free neighbor
      straight = true;
      go_straight(neighbor, current, orientation);

    }
  }

  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  //exit(0);
}

bool OnlineBoustrophedon::check(CellPtr cell_to_check) {
  return
      (old_cells.find(cell_to_check) != old_cells.end()) ? OLD_CELL : NEW_CELL;
}

}
}
}
