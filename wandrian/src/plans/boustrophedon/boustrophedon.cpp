/*
 * boustrophedon.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/plans/boustrophedon/boustrophedon.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon {

Boustrophedon::Boustrophedon() :
    robot_size(0) {
}

Boustrophedon::~Boustrophedon() {
}

// double environment_size

void Boustrophedon::initialize(PointPtr starting_point, double robot_size,
    std::string namefile) {
  this->robot_size = robot_size;
  this->map = ExtendedMapPtr(new ExtendedMap(namefile));
  this->map->build();
  path.insert(path.end(), starting_point);
}

ExtendedMapPtr Boustrophedon::get_map() {
  return map;
}

void Boustrophedon::cover() {
  // old_cells.insert(starting_cell);
  boustrophedon_cd();
}

bool Boustrophedon::go_to(PointPtr position, bool flexibility) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  return BasePlan::go_to(position, flexibility);
}

bool Boustrophedon::go_into(SpacePtr space) {
//TODO: Run with space is trapezoid
  double x,y;
  double d_upper, d_below, d_y;
  PointPtr starting_point;
  PointPtr last_position;
  PointPtr new_position;
  VerticesPtr below_vertices, upper_vertices;
  double flag;
  below_vertices = space->get_vertices_below();
  upper_vertices = space->get_vertices_upper();
  d_upper = robot_size *(upper_vertices->get_upper_point()->y - upper_vertices->get_position()->y)/
		  (upper_vertices->get_upper_point()->x - upper_vertices->get_position()->x);
  d_below = -robot_size *(below_vertices->get_below_point()->y - below_vertices->get_position()->y)/
		  (below_vertices->get_below_point()->x - below_vertices->get_position()->x);
  std::cout <<"D_upper : "<< d_upper<<"\n";
  std::cout <<"D_below : "<< d_below<<"\n";
  //Starting point
  x = below_vertices->get_position()->x + robot_size/2;
  y = below_vertices->get_position()->y + d_below/2;
  std::cout <<"Starting Point: ("<< x <<","<< y <<") \n";
  starting_point = PointPtr(new Point(x, y));
  go_to(starting_point, STRICTLY);
  std::cout << "\033[1;34mLast_position-\033[0m\033[1;31m\033[0m: "
      << starting_point->x << "," << starting_point->y << "\n";
  //2 two segment //

  flag = robot_size;
  d_y = below_vertices->get_upper_point()->y-below_vertices->get_position()->y;
  for (int i = 0; i < int((below_vertices->get_below_point()->x-below_vertices->get_position()->x)
  / robot_size + EPSILON); ++i) {
    if (i != 0) {
      last_position = path.back();
      new_position = PointPtr(
          new Point(last_position->x + robot_size, last_position->y));
      go_to(new_position, STRICTLY);
    }
    std::cout << "\033[1;34mNumber_line-\033[0m\033[1;31m\033[0m: " << i
        << "\n";
    if(flag > 0){
    	d_y = d_y + d_upper;
    }else{
    	d_y = d_y + d_below;
    }
    for (int j = 0; j < int(d_y / robot_size + EPSILON); ++j) {
      last_position = path.back();
      new_position = PointPtr(
          new Point(last_position->x, last_position->y + flag));
      go_to(new_position, STRICTLY);
    }
    flag = -flag;
  }
  return true;
}

void Boustrophedon::dfs(SpacePtr space) {
  std::list<SpacePtr>::iterator inspectLC;
  space->status_visited = true;
  double x, y;
  go_into(space);
  map->number_space_need_visit--;
  for (inspectLC = space->children.begin(); inspectLC != space->children.end();
      ++inspectLC) {
    if ((*inspectLC)->status_visited == false) {
      go_to((*inspectLC)->point_backtrack, STRICTLY);
      go_to(
          PointPtr(
              new Point((*inspectLC)->point_backtrack->x + robot_size,
                  (*inspectLC)->point_backtrack->y)), STRICTLY);
      dfs(*inspectLC);
      if(map->number_space_need_visit > 1){
    	  go_to(
    	        PointPtr(
    	        new Point((*inspectLC)->point_backtrack->x + robot_size,
    	                  (*inspectLC)->point_backtrack->y)), STRICTLY);
    	  go_to((*inspectLC)->point_backtrack, STRICTLY);
      }
    }
  }
  if ((space->point_backtrack)&&(map->number_space_need_visit > 1)) {
//    go_to(
//        PointPtr(
//            new Point(
//                space->get_center()->x + space->get_size_x() / 2
//                    - robot_size / 2, space->point_backtrack->y)), STRICTLY);
  }
}

std::list<SpacePtr> Boustrophedon::create_list_space(RectanglePtr environment,
    std::list<VerticesPtr> list_vertices) {
  std::list<VerticesPtr> listvertices_temp;
  std::list<VerticesPtr>::iterator inspectLV;
  std::list<VerticesPtr>::iterator inspectLVT;
  std::list<SpacePtr> list_space;
  std::list<SpacePtr>::iterator inspectLS;
  std::list<SpacePtr>::iterator inspectLS_temp;
  PointPtr center_temp;
  int i = 1, j = 1;
  double size_x = 0, size_y = 0;
  VerticesPtr vertices_previous;
  std::list<PointPtr> list_point;
  std::list<PointPtr>::iterator inspectLP;

  // Create list space!
  std::cout << "Starting add parent!" << "\n";
  list_space.sort(Space::compare_positions_x);
  std::cout << list_space.size() << "\n";
  for (inspectLS = --list_space.end(), i = 1; inspectLS != list_space.end();
      --inspectLS) {
    for (inspectLS_temp = list_space.begin(), i = 1;
        inspectLS_temp != list_space.end(); ++inspectLS_temp) {
      if (Space::is_parent(*inspectLS_temp, *inspectLS)) {
        (*inspectLS_temp)->children.push_back(*inspectLS);
        (*inspectLS)->set_parent(*inspectLS_temp);
        (*inspectLS)->set_point_backtrack(*inspectLS_temp, *inspectLS,
            robot_size);
        break;
      }
      std::cout << "Find \n";
    }
  }
  return list_space;
}

std::list<VerticesPtr> Boustrophedon::create_list_vertices(
    RectanglePtr environment, std::list<PolygonPtr> list_obstacle) {
  std::list<PointPtr> list_point;
  PointPtr near_point, position;
  std::list<PointPtr>::iterator inspectLP;
  std::list<VerticesPtr> list_vertices;
  std::list<VerticesPtr>::iterator inspectLV;
  std::list<PolygonPtr>::iterator inspectLO;
  VerticesPtr temp;
  bool is_of_obstacle;
  int i = 1, j = 1;
  std::cout<<"Create Vertices \n";
  list_obstacle.push_front(environment);
  for (inspectLO = list_obstacle.begin(), i = 1; inspectLO != list_obstacle.end();
      ++inspectLO) {
    list_point = (*inspectLO)->get_points();
    is_of_obstacle = true;
    if(inspectLO == list_obstacle.begin()){
    	is_of_obstacle = false;
    }
    inspectLP = --list_point.end();
    near_point = PointPtr(new Point((*inspectLP)->x, (*inspectLP)->y));
    for (inspectLP = list_point.begin(), j = 0; j < list_point.size() + 1;
        ++inspectLP) {
    	j++;
    	if(inspectLP == list_point.begin()){
    		position = PointPtr(new Point((*inspectLP)->x, (*inspectLP)->y));
    		continue;
    	}
    	if(inspectLP == list_point.end()){
        	    		inspectLP = list_point.begin();
        }
    	temp = VerticesPtr(new Vertices(position, near_point, *inspectLP, is_of_obstacle));
    	if(is_of_obstacle){
    		std::cout<<"\nSet obstacles :("<<position->x<<" ,"<<position->y<<" )\n";
    		temp->set_is_obstacles_upper(*inspectLO, environment->get_height());
    		temp->set_is_obstacles_below(*inspectLO, environment->get_height());
    	}
    	//list_vertices.push_back(VerticesPtr(new Vertices(position, near_point, *inspectLP, is_of_obstacle)));
    	list_vertices.push_back(temp);
    	near_point = position;
    	position = PointPtr(new Point((*inspectLP)->x, (*inspectLP)->y));
    }
    list_point.clear();
  }
  list_vertices.sort(Vertices::compare_positions_x);
  return list_vertices;
}

void Boustrophedon::boustrophedon_cd() {
  std::list<VerticesPtr> list_vertices;
  std::list<SpacePtr> list_space;
  std::list<SpacePtr>::iterator inspectLS;
  std::list<PolygonPtr> list_object;
  std::list<PolygonPtr>::iterator inspectLO;
  std::list<PointPtr> list_point;
  std::list<PointPtr>::iterator inspectLP;
  std::list<VerticesPtr>::iterator u;
  SegmentPtr s1,s2;
  int i, j;
  PointPtr point_temp;
  SpacePtr test_func_into;

   //Create vertices
  list_vertices = create_list_vertices(map->get_boundary(),
      map->get_extendedmap_obstacles());
  for(u = list_vertices.begin(), i=1; u!= list_vertices.end();++u){
	  std::cout<<"Vertices "<< i++<<" : ("<<(*u)->get_position()->x<<" , "
			  	  	  	  	  	  	  	 <<(*u)->get_position()->y<<" )\n";
	  std::cout<<"Upper "<<" : ("<<(*u)->get_upper_point()->x<<" , "
	  			  	  	   	  	<<(*u)->get_upper_point()->y<<" )\n";
	  std::cout<<"Below "<<" : ("<<(*u)->get_below_point()->x<<" , "
	 	  			  	  	   	 <<(*u)->get_below_point()->y<<" )\n";
	  std::cout<<"Type Vertices :"<<(*u)->type_vertice<<"\n";
	  std::cout<<"Upper:\n";
	  if((*u)->is_of_obstacles){
		  std::cout<<" Vertices is obstacles!\n";
	  }
	  std::cout<<"\n";
  }
  //Create Space
  	 s1 = SegmentPtr(new Segment(PointPtr(new Point(0,-1)), PointPtr(new Point(0,0))));
  	 s2 = SegmentPtr(new Segment(PointPtr(new Point(0.5,-0.5)), PointPtr(new Point(-0.5,1.5))));
//  	 s1 = SegmentPtr(new Segment(0,-1, 0, 2));
//  	 s2 = SegmentPtr(new Segment(0.5,-0.5,-0.5,1.5));
  	 point_temp = s1%s2;
  	 if(point_temp){
  		 std::cout<<"This : ("<<point_temp->x<<" ,"<<point_temp->y<<" )\n";
  	 }else{
  		 std::cout<<"Eo giao!"<<"\n";
  	 }
// list_space = create_list_space(map->get_boundary(), list_vertices);

  list_object = map->get_extendedmap_obstacles();

  for(inspectLO = list_object.begin(), i =1; inspectLO != list_object.end();
	  ++inspectLO){
	  std::cout<<"O"<<i++<<" :\n";
	  list_point = (*inspectLO)->get_boundary();
	  for(inspectLP = list_point.begin(), j=1;inspectLP != list_point.end();
		 ++inspectLP){
		  std::cout<<"P"<<j++<<" ("<<(*inspectLP)->x <<","<< (*inspectLP)->y<<" )"<<" \n";
	  }
  }

 //  map->number_space_need_visit = list_space.size();
//  for (inspectLS = list_space.begin(), i = 1; inspectLS != list_space.end();
//      ++inspectLS) {
//    std::cout << "Space:" << ": " << std::endl;
//    if ((*inspectLS)->status_visited == false) {
//      dfs(*inspectLS);
//    }
//  }

}
}
}
}
