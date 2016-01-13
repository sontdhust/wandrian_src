/*
 * boustronphedon_cd.cpp
 *
 */

#include "../../../include/plans/boustrophedon_off/boustrophedon.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon_off {

Boustrophedon::Boustrophedon() :
    robot_size(0),environment_size(0) {
}

Boustrophedon::~Boustrophedon() {
}
//, double environment_size
void Boustrophedon::initialize(PointPtr starting_point, double robot_size) {
  this->robot_size = robot_size;
  //this->environment_size = environment_size;

  path.insert(path.end(), starting_point);
}

void Boustrophedon::cover() {
 // old_cells.insert(starting_cell);
  boustrophedon_cd();
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

bool Boustrophedon::go_go(PointPtr center_cellnext, double edgex, double edgey){
    double x = (center_cellnext->x*2 + edgex - robot_size)/2 ;
    double y = (center_cellnext->y*2 - edgey + robot_size)/2 ;

    PointPtr starting_point = PointPtr(new Point(x,y));
    go_to(starting_point, STRICTLY);

    std::cout << "\033[1;34mLast_position-\033[0m\033[1;31m\033[0m: "
    << starting_point->x << "," << starting_point->y << "\n";
    

    PointPtr last_position;
    PointPtr new_position;

    int flag = 0;

    for (int i = 0; i < edgex; ++i){
      if (i!=0){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x-0.5, last_position->y));
        go_to(new_position, STRICTLY);
      }
      for (int j = 0; j < edgey*2-1; ++j){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x, last_position->y + 0.5));
        go_to(new_position, STRICTLY);
      }
      last_position = *(--path.end());
      new_position = PointPtr(new Point(last_position->x-0.5, last_position->y));
      go_to(new_position, STRICTLY);
      
      for (int j = 0; j < edgey*2-1; ++j){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x, last_position->y - 0.5));
        go_to(new_position, STRICTLY);
      }

      

    }
      return true;
}

bool Boustrophedon::go_with(VectorPtr orientation, double step) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = PointPtr(
      new Point(*last_position + *orientation * step * robot_size / 2));
  return go_to(new_position, STRICTLY);
}

void Boustrophedon::boustrophedon_cd() {

  int e_size = 0;
  if (environment_size!= 0){
    e_size = environment_size;
  }
  else{
    e_size = 4;
  }
  // //No obstacles 
  // PointPtr new_position = PointPtr(new Point(0,0));
  // go_go(new_position, e_size, e_size);
  // Obstacles
  PointPtr d1 = PointPtr(new Point(2,0));
  PointPtr d2 = PointPtr(new Point(0,-2));
  PointPtr d3 = PointPtr(new Point(-2,0));

  go_go(d1,2,6);
  go_go(d2,2,2);
  go_go(d3,2,6);



}



}
}
}
