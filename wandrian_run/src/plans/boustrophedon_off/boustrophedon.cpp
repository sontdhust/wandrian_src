/*
 * boustrophedon_cd.cpp
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


bool Boustrophedon::go_go(SpacePtr space){
    double x = (space->get_center()->x*2 - space->get_sizex() + robot_size)/2 ;
    double y = (space->get_center()->y*2 - space->get_sizey() + robot_size)/2 ;

    PointPtr starting_point = PointPtr(new Point(x,y));
    go_to(starting_point, STRICTLY);

    std::cout << "\033[1;34mLast_position-\033[0m\033[1;31m\033[0m: "
    << starting_point->x << "," << starting_point->y << "\n";
    std::cout << "\033[1;34mddd_position-\033[0m\033[1;31m\033[0m: "
       << space->get_sizex() << "," << space->get_sizey() << "\n";
    

    PointPtr last_position;
    PointPtr new_position;

    int flag = 0;

    for (int i = 0; i < space->get_sizex(); ++i){
      if (i!=0){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x+0.5, last_position->y));
        go_to(new_position, STRICTLY);
      }
      std::cout << "\033[1;34mTest_position-\033[0m\033[1;31m\033[0m: "
            << i << "\n";
      for (int j = 0; j < space->get_sizey()*2-1; ++j){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x, last_position->y + 0.5));
        go_to(new_position, STRICTLY);
      }
      last_position = *(--path.end());
      new_position = PointPtr(new Point(last_position->x+0.5, last_position->y));
      go_to(new_position, STRICTLY);
      
      for (int j = 0; j < space->get_sizey()*2-1; ++j){
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
void Boustrophedon::dfs(SpacePtr space){
	std::list<SpacePtr>::iterator inspectLC;
	space->status_visited = true;
	double x, y;
	std::cout <<"Visit Space"<< space->get_sizex()<< std::endl;
	go_go(space);

	for (inspectLC = space->children.begin(); inspectLC != space->children.end(); ++inspectLC) {
		if((*inspectLC)->status_visited == false){
			//TODO: Space size odd
		     x = (space->get_center()->x*2 + space->get_sizex() - robot_size)/2 ;
		     y = ((*inspectLC)->get_center()->y*2 - (*inspectLC)->get_sizey() + robot_size)/2 ;

		     (*inspectLC)->set_point_backtrack(PointPtr(new Point(x,y)));
			go_to((*inspectLC)->point_backtrack, STRICTLY);
			dfs(*inspectLC);
			go_to((*inspectLC)->point_backtrack, STRICTLY);

		}

	}
}

void Boustrophedon::boustrophedon_cd() {
	ObstaclePtr obstacePtr;
	std::list<ObstaclePtr> listobstacle;
	//std::list<ObstaclePtr>::iterator inspectLP;
	std::list<SpacePtr> listpoint;
	std::list<SpacePtr>::iterator inspectLP;
	std::list<SpacePtr>::iterator inspectLC;
	int i =1,j =1;

	SpacePtr s1 = SpacePtr(new Space(PointPtr(new Point(-2.5,0)), 1 ,6 ));
	SpacePtr s2 = SpacePtr(new Space(PointPtr(new Point(-1.5,2)), 1 ,2 ));
	SpacePtr s3 = SpacePtr(new Space(PointPtr(new Point(-1.5,-2)), 1 ,2 ));
	SpacePtr s4 = SpacePtr(new Space(PointPtr(new Point(0,0)), 2 ,6 ));
	SpacePtr s5 = SpacePtr(new Space(PointPtr(new Point(2,1)), 2 ,4 ));

	listpoint.push_back(s1);
	s1->children.push_back(s3);
	s1->children.push_back(s2);
	listpoint.push_back(s2);
	s2->set_parent(s1);
	listpoint.push_back(s3);
	s3->set_parent(s1);
	s3->children.push_back(s4);
	listpoint.push_back(s4);
	s4->set_parent(s3);
	s4->children.push_back(s5);
	listpoint.push_back(s5);
	s5->set_parent(s4);

	for(inspectLP = listpoint.begin(), i = 1; inspectLP != listpoint.end(); ++inspectLP){
		std::cout <<"Space:"<< i++ <<": "<<std::endl;
		if((*inspectLP)->status_visited == false){
			dfs(*inspectLP);
		}
	}





// int e_size = 0;
//  if (environment_size!= 0){
//    e_size = environment_size;
//  }
//  else{
//    e_size = 4;
//  }
  // //No obstacles 
  // PointPtr new_position = PointPtr(new Point(0,0));
  // go_go(new_position, e_size, e_size);
  // Obstacles
//  PointPtr d1 = PointPtr(new Point(2,0));
//  PointPtr d2 = PointPtr(new Point(0,-2));
//  PointPtr d3 = PointPtr(new Point(-2,0));

//  go_go(PointPtr(new Point(2,0)),2,6);
//  go_go(PointPtr(new Point(0,-2)),2,2);
//  go_go(PointPtr(new Point(-2,0)),2,6);
}
}
}
}
