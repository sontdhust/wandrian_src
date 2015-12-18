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
  old_cells.insert(old_cells.end(), starting_cell);
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
bool OnlineBoustrophedon::go_to_bpcell(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
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
bool OnlineBoustrophedon::go_with_bpcell(PointPtr last_position, VectorPtr orientation, double step) {
  PointPtr new_position = PointPtr(
      new Point(*last_position + *orientation * step * robot_size / 2));
  return go_to_bpcell(new_position, STRICTLY);
}
void OnlineBoustrophedon::go_straight(CellPtr neighbor, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor->set_parent(current);
  // current->neighbors.insert(current->neighbors.end(), neighbor);
  old_cells.insert(neighbor);
  go_with(orientation, 2);
  bplist.erase(neighbor);
  online_boustrophedon(neighbor);
}
void OnlineBoustrophedon::turn_left(CellPtr neighbor_left, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor_left->set_parent(current);
  // current->neighbors.insert(current->neighbors.end(), neighbor_left);
  old_cells.insert(neighbor_left);
  go_with(orientation->rotate_counterclockwise_left(), 2);
  check_rotate = -1;
  bplist.erase(neighbor_left);
  online_boustrophedon(neighbor_left);
}
void OnlineBoustrophedon::turn_right(CellPtr neighbor_right, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor_right->set_parent(current);
  // current->neighbors.insert(current->neighbors.end(), neighbor_right);
  old_cells.insert(neighbor_right);
  go_with(orientation->rotate_counterclockwise_right(), 2);
  check_rotate = 1;
  bplist.erase(neighbor_right);
  online_boustrophedon(neighbor_right);
}
void OnlineBoustrophedon::find_bpcell(CellPtr current){
  double distance = 10000;
  
  for (std::set<CellPtr>::iterator i = bplist.begin(); i != bplist.end(); i++) {
    CellPtr tmp = CellPtr(*i);
    double tmp_distance = get_distance(current,tmp);
    if(tmp_distance < distance){
      distance = tmp_distance;
      starting_cell = tmp;
    }
  }
  std::cout << "\033[1;32mNew Starting Cell:\033[0m" << " " << starting_cell->get_center()->x << ", " << starting_cell->get_center()->y; 
  
}
void OnlineBoustrophedon::bpmove(CellPtr current, CellPtr neighbor){
  CellPtr tmp;
  double distance = 10000;
  int j=0;
  std::cout << "\n"<<current->get_center()->x<< " " <<current->get_center()->y;
  while(true){
    if(*neighbor->get_center() == *current->get_center()){break;}
    for (std::list<boost::shared_ptr<Cell> >::iterator i = current->neighbors.begin(); i != current->neighbors.end(); i++){
      std::cout << "\n neighbor: "<<CellPtr(*i)->get_center()->x<<" "<<CellPtr(*i)->get_center()->y;
      if(check(CellPtr(*i)) == OLD_CELL){
        double tmp_distance = get_distance(neighbor,CellPtr(*i));
        if(tmp_distance < distance){
          distance = tmp_distance;
          tmp = CellPtr(*i);

          std::cout << "\n Tmp: "<<tmp->get_center()->x<<" "<<tmp->get_center()->y;
        }
      }
    }
    std::cout << "\n"<<tmp->get_center()->x<<" "<<tmp->get_center()->y;
    
    VectorPtr orientation = VectorPtr(
            new Vector(
              (*(tmp->get_center()) - *(current->get_center()))
                / robot_size)
          );
          std::cout << "\n"<<tmp->get_center()->x<<tmp->get_center()->y;
     go_with_bpcell(current->get_center(),orientation,2);
     current=*old_cells.find(tmp);
     
  }
  VectorPtr orientation = VectorPtr(
            new Vector(
              (*(starting_cell->get_center()) - *(neighbor->get_center()))
                / robot_size)
          );
  go_with_bpcell(neighbor->get_center(),orientation,2);
  path.insert(path.end(),starting_cell->get_center());
  old_cells.insert(old_cells.end(),starting_cell);
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - robot_size)),
              robot_size)));
  bplist.erase(starting_cell);
  online_boustrophedon(starting_cell);
}
double OnlineBoustrophedon::get_distance(CellPtr begin, CellPtr end){
  return std::abs(end->get_center()->x - begin->get_center()->x) + std::abs(end->get_center()->y - begin->get_center()->y);
}
void OnlineBoustrophedon::move_bpcell(CellPtr current){
  CellPtr neighbor_N = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    starting_cell->get_center()->x, starting_cell->get_center()->y +robot_size)),
            robot_size));
  CellPtr neighbor_W = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    starting_cell->get_center()->x + robot_size, starting_cell->get_center()->y)),
            robot_size));
  CellPtr neighbor_E = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    starting_cell->get_center()->x -robot_size, starting_cell->get_center()->y)),
            robot_size));
  CellPtr neighbor_S = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    starting_cell->get_center()->x, starting_cell->get_center()->y - robot_size)),
            robot_size));
  if(check(neighbor_N)==OLD_CELL){
    std::cout << "\033[1;32mNew neighbor N:\033[0m" << " " << neighbor_N->get_center()->x << ", " << neighbor_N->get_center()->y;     
    bpmove(current, neighbor_N);
  }else if(check(neighbor_W)==OLD_CELL){
    std::cout << "\033[1;32mNew neighbor W:\033[0m" << " " << neighbor_W->get_center()->x << ", " << neighbor_W->get_center()->y;
    bpmove(current, neighbor_W);
  }else if(check(neighbor_E)==OLD_CELL){
    std::cout << "\033[1;32mNew neighbor E:\033[0m" << " " << neighbor_E->get_center()->x << ", " << neighbor_E->get_center()->y;
    bpmove(current, neighbor_E);
  }else if(check(neighbor_S)==OLD_CELL){
    std::cout << "\033[1;32mNew neighbor S:\033[0m" << " " << neighbor_S->get_center()->x << ", " << neighbor_S->get_center()->y;
    bpmove(current, neighbor_S);
  }
}

void OnlineBoustrophedon::online_boustrophedon(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr orientation = VectorPtr(
      new Vector(
          (*(current->get_parent()->get_center()) - *(current->get_center()))
              / robot_size)
      );

    // quay vector 180 do
    orientation = orientation->rotate_counterclockwise_180();
    if(current->get_center()==starting_cell->get_center() && see_obstacle(orientation,1)){
      orientation = orientation->rotate_counterclockwise_180();        
    }
    // Scan for the first new neighbor of current cell in counterclockwise order
    CellPtr neighbor = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    *(current->get_center()) + *orientation * robot_size)),
            robot_size));
    // if(check(neighbor)!=OLD_CELL && see_obstacle(orientation,2)==false){
    //   bplist.insert(neighbor);  
    // }
    CellPtr neighbor_left = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    *(current->get_center()) + *orientation->rotate_counterclockwise_left() * robot_size)),
            robot_size));
    if(check(neighbor_left)!=OLD_CELL && see_obstacle(orientation->rotate_counterclockwise_left(),1)==false){
      bplist.insert(neighbor_left);  
    }
    
    CellPtr neighbor_right = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    *(current->get_center()) + *orientation->rotate_counterclockwise_right() * robot_size)),
            robot_size));
    if(check(neighbor_right)!=OLD_CELL && see_obstacle(orientation->rotate_counterclockwise_right(),1)==false){
      bplist.insert(neighbor_right);  
    }
    CellPtr neighbor_bottom = CellPtr(
        new Cell(
            PointPtr(
                new Point(
                    *(current->get_center()) + *orientation->rotate_counterclockwise_180() * robot_size)),
            robot_size));
    if(check(neighbor_bottom)!=OLD_CELL && see_obstacle(orientation->rotate_counterclockwise_180(),1)==false){
      bplist.insert(neighbor_bottom);  
    }
    current->neighbors.insert(current->neighbors.end(), neighbor);
    current->neighbors.insert(current->neighbors.end(), neighbor_left);
    current->neighbors.insert(current->neighbors.end(), neighbor_right);
    current->neighbors.insert(current->neighbors.end(), neighbor_bottom);
    std::cout << "  \033[1;33mneighbor\033[0m: " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    if(straight==false){
      if(check_rotate==1){
        if(see_obstacle(orientation->rotate_counterclockwise_right(), 1)==false && check(neighbor_right) != OLD_CELL)
        {
          straight = true;
          turn_right(neighbor_right,current,orientation);
        }
        else if(see_obstacle(orientation->rotate_counterclockwise_left(), 1)==false && check(neighbor_left) != OLD_CELL)
        {        
          straight = true;
          turn_left(neighbor_left,current,orientation);
        }
        else if(see_obstacle(orientation, 1)==false && check(neighbor) != OLD_CELL)
        {
          go_straight(neighbor,current,orientation);
        }
      }
      else{
        if(see_obstacle(orientation->rotate_counterclockwise_left(), 1)==false && check(neighbor_left) != OLD_CELL)
        {
          straight = true;
          turn_left(neighbor_left,current,orientation);
        }
        else if(see_obstacle(orientation->rotate_counterclockwise_right(), 1)==false && check(neighbor_right) != OLD_CELL)
        {        
          straight = true;
          turn_right(neighbor_right,current,orientation);
        }
        else if(see_obstacle(orientation, 1)==false && check(neighbor) != OLD_CELL)
        {
          go_straight(neighbor,current,orientation);
        }
      }
    }
    else{
    if(see_obstacle(orientation, 1) || check(neighbor) == OLD_CELL) { // TODO: Check obstacle here
      std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
      // Go to next sub-cell
      
      if(check_rotate==1){
        if(see_obstacle(orientation->rotate_counterclockwise_left(), 1)==false && check(neighbor_left) != OLD_CELL)
        {
          straight = false;
          turn_left(neighbor_left,current,orientation);
        }
        else if(see_obstacle(orientation->rotate_counterclockwise_right(), 1)==false && check(neighbor_right) != OLD_CELL)
        {        
          straight = false;
          turn_right(neighbor_right,current,orientation);
        }

      } else if (check_rotate==-1){
        if(see_obstacle(orientation->rotate_counterclockwise_right(), 1)==false && check(neighbor_right) != OLD_CELL)
        {
          straight = false;
          turn_right(neighbor_right,current,orientation);
        }
        else if(see_obstacle(orientation->rotate_counterclockwise_left(), 1)==false && check(neighbor_left) != OLD_CELL)
        {        
          straight = false;
          turn_left(neighbor_left,current,orientation);
        }
      }
    } else if(see_obstacle(orientation, 1)==false && check(neighbor) != OLD_CELL){ // New free neighbor
          straight = true;
          go_straight(neighbor,current,orientation);
    }
  }
  old_cells.insert(old_cells.end(),current);
  std::cout << "\n\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
        << current->get_center()->x << "," << current->get_center()->y << "\n";
    std::cout << "Backtrack list: "
        << bplist.size() << "\n";
  if(bplist.size()>0){
  find_bpcell(current);
  move_bpcell(current);
  }
  //exit(0);
}


bool OnlineBoustrophedon::check(CellPtr cell_to_check) {
  return
      (old_cells.find(cell_to_check) != old_cells.end()) ? OLD_CELL : NEW_CELL;
}

}
}
}
