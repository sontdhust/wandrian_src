/*
 * boustrophedon_pt.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/boustrophedon_off/boustrophedon.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon_off {

Boustrophedon::Boustrophedon() :
    robot_size(0) {
}

Boustrophedon::~Boustrophedon() {
}


void Boustrophedon::initialize(PointPtr starting_point, double robot_size) {
  this->robot_size = robot_size;
  // Initialize starting_cell
  starting_cell = CellPtr(
      new Cell(
          PointPtr(
              new Point(starting_point->x,
                  starting_point->y)), robot_size));

  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - robot_size)),
              robot_size)));

  path.insert(path.end(), starting_point);
}

void Boustrophedon::cover() {
  old_cells.insert(starting_cell);
  boustrophedon_pt(starting_cell);
}

void Boustrophedon::set_environment(EnvironmentPtr environment) {
  this->environment = environment;
}

void Boustrophedon::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool Boustrophedon::go_to(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  path.insert(path.end(), position);

  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}

bool Boustrophedon::see_obstacle(VectorPtr orientation, double step) {
  if (behavior_see_obstacle)
    return behavior_see_obstacle(orientation, step);

  // Simulator check obstacle
  PointPtr last_position = *(--path.end());
  PointPtr new_position = PointPtr(
      new Point(*last_position + *orientation * step * robot_size / 2));

  if (environment) {
    CellPtr space = boost::static_pointer_cast<Cell>(environment->space);

    if (new_position->x >= space->get_center()->x + space->get_size() / 2
        || new_position->x <= space->get_center()->x - space->get_size() / 2
        || new_position->y >= space->get_center()->y + space->get_size() / 2
        || new_position->y <= space->get_center()->y - space->get_size() / 2) {
      return true;
    }
    for (std::list<PolygonPtr>::iterator o = environment->obstacles.begin();
        o != environment->obstacles.end(); o++) {
      CellPtr obstacle = boost::static_pointer_cast<Cell>(*o);
      if (new_position->x
          >= obstacle->get_center()->x - obstacle->get_size() / 2
          && new_position->x
              <= obstacle->get_center()->x + obstacle->get_size() / 2
          && new_position->y
              >= obstacle->get_center()->y - obstacle->get_size() / 2
          && new_position->y
              <= obstacle->get_center()->y + obstacle->get_size() / 2) {
        return true;
      }
    }
  }
  return false;
}

bool Boustrophedon::go_with(VectorPtr orientation, double step) {
  PointPtr last_position = *(--path.end());
  
  PointPtr new_position = PointPtr(
      new Point(*last_position + *orientation * step * robot_size ));

  return go_to(new_position, STRICTLY);
}

void Boustrophedon::boustrophedon_pt(CellPtr current) {

  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";

  VectorPtr orientation = VectorPtr(
      new Vector(
          (*(current->get_parent()->get_center()) - *(current->get_center()))
              / robot_size));

  int e_size = 0;

  if (environment){
      CellPtr space = boost::static_pointer_cast<Cell>(environment->space);
      e_size = space->get_size();

      std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << space->get_size() << "\n";
    }
  else{
    e_size = 4;
  }
      
      std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
      << e_size << "\n";

  for (int i = 0; i < e_size; ++i){

        if (i==0){
          orientation = orientation->rotate_counterclockwise();
          orientation = orientation->rotate_counterclockwise();
        }

       // PointPtr last_position = *(--path.end());

        // if ((last_position->x == &&(last_position->y == -e_size/2 + 0.5)) 
        //   break;
     

        go_with(orientation, e_size*2 -1);
        orientation = orientation->rotate_counterclockwise();
        go_with(orientation,1);
         orientation = orientation->rotate_counterclockwise();
        go_with(orientation, e_size*2 -1 );
        orientation = orientation->rotate_alongclockwise();
        go_with(orientation, 1);
        orientation = orientation->rotate_alongclockwise();

       }

  

  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
}

bool Boustrophedon::check(CellPtr cell_to_check) {
  return
      (old_cells.find(cell_to_check) != old_cells.end()) ? OLD_CELL : NEW_CELL;
}

}
}
}
