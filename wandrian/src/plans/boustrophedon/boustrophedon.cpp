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
  std::list<VerticesPtr>::iterator inspectLV;
  std::list<SegmentPtr> list_segment;
  std::list<SegmentPtr>::iterator u;
  std::list<SpacePtr> list_space;
  std::list<SpacePtr>::iterator inspectLS;
  std::list<SpacePtr>::iterator inspectLS_temp;
  std::list<PointPtr> list_point_of_space;

  SegmentPtr segment_upper, segment_below, segment_temp, segment_intersect;
  int i = 1, j = 1;
  VerticesPtr vertices_previous;
  PointPtr temp_point;
  std::list<PointPtr> list_point;
  std::list<PointPtr>::iterator inspectLP;


  for (inspectLV=list_vertices.begin(), j =1;inspectLV!=list_vertices.end() ; ++inspectLV) {
	  std::cout<<"Current Vertices "<< j++<< " :("<<(*inspectLV)->get_position()->x <<" ,"
			  	  	  	  	  	  	   <<(*inspectLV)->get_position()->y<<" )\n";
	  //Print current segment
//	  std::cout<<"List current segment :\n";
//      for(u = list_segment.begin(), i=1; u!= list_segment.end(); ++u){
//    	  std::cout<<"Segment "<<i++<<":\n"<<" Point 1:("<<(*u)->p1->x<<" ,"<<(*u)->p1->y<<" )\n"
//    			  	  	  	  	  	  	   <<" Point 2:("<<(*u)->p2->x<<" ,"<<(*u)->p2->y<<" )\n";
//      }
	  if((list_segment.size() == 0)&&(inspectLV == list_vertices.begin())){
		  list_segment.push_back(SegmentPtr(new Segment((*inspectLV)->get_position(),(*inspectLV)->get_upper_point())));
		  ++inspectLV;
		  list_segment.push_back(SegmentPtr(new Segment((*inspectLV)->get_position(),(*inspectLV)->get_below_point())));
		  continue;
	  }
	  if(list_segment.size() ==0) break;
	  //Create list space! Important!
	  //1 Create list point of space
	  //2 Update list segment
	  //3 Update list vertices
	  list_point_of_space.clear();
	  segment_temp = SegmentPtr(new Segment((*inspectLV)->get_position()->x, (*inspectLV)->get_position()->y -
					 environment->get_height(),(*inspectLV)->get_position()->x,
			 		 (*inspectLV)->get_position()->y + 2*environment->get_height()));
	  if(!(*inspectLV)->is_obstacles_upper){
		segment_upper = Vertices::get_segment_contain_nearest_intersect_point(list_segment,
			 			(*inspectLV)->get_position(),environment->get_height());
	  }
	  if(!(*inspectLV)->is_obstacles_below){
		segment_below = Vertices::get_segment_contain_nearest_intersect_point(list_segment,
		 			  (*inspectLV)->get_position(),-environment->get_height());
	  }
	  switch((*inspectLV)->type_vertice){
	  case 1:
		  std::cout<<"Type Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  if(!(*inspectLV)->is_obstacles_upper){
			list_point_of_space.push_back((*inspectLV)->get_upper_point());
			list_point_of_space.push_back(Vertices::get_point_x_litte(segment_upper));
			list_point_of_space.push_back(segment_upper%segment_temp);
			list_point_of_space.push_back((*inspectLV)->get_position());
			list_space.push_back(SpacePtr(new Space(list_point_of_space)));
			list_point_of_space.clear();
			Vertices::update_list_vertices(list_vertices, Vertices::get_point_x_large(segment_upper),
					  Vertices::get_point_x_litte(segment_upper), segment_temp%segment_upper);
			Vertices::update_list_segment(list_segment,segment_upper, segment_upper%segment_temp);
			list_point_of_space.push_back(Vertices::get_point_x_litte(segment_below));
			list_point_of_space.push_back((*inspectLV)->get_below_point());
			list_point_of_space.push_back((*inspectLV)->get_position());
			list_point_of_space.push_back(segment_below%segment_temp);
			Vertices::update_list_vertices(list_vertices, Vertices::get_point_x_large(segment_below),
					  Vertices::get_point_x_litte(segment_below), segment_temp%segment_below);
			Vertices::update_list_segment(list_segment,segment_below, segment_below%segment_temp);
			}else{
				list_point_of_space.push_back((*inspectLV)->get_below_point());
				list_point_of_space.push_back((*inspectLV)->get_upper_point());
				list_point_of_space.push_back((*inspectLV)->get_position());
			}
		  list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_below_point(),(*inspectLV)->get_position())));
		  list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_upper_point(),(*inspectLV)->get_position())));
		  break;
	  case 2:
		  std::cout<<"Type Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  if(!(*inspectLV)->is_obstacles_upper){
			  segment_intersect = segment_upper;
			  list_point_of_space.push_back((*inspectLV)->get_below_point());
			  list_point_of_space.push_back(Vertices::get_point_x_litte(segment_upper));
			  list_point_of_space.push_back(segment_upper%segment_temp);
			  list_point_of_space.push_back((*inspectLV)->get_position());
		  }else{
			  segment_intersect = segment_below;
			  list_point_of_space.push_back(Vertices::get_point_x_litte(segment_below));
			  list_point_of_space.push_back((*inspectLV)->get_below_point());
			  list_point_of_space.push_back((*inspectLV)->get_position());
			  list_point_of_space.push_back(segment_below%segment_temp);
		  }
		  Vertices::update_list_vertices(list_vertices, Vertices::get_point_x_large(segment_intersect),
				  	  	  	 Vertices::get_point_x_litte(segment_intersect), segment_temp%segment_intersect);
		  Vertices::update_list_segment(list_segment,segment_intersect, segment_intersect%segment_temp);
		  list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_below_point(),(*inspectLV)->get_position())));
		  list_segment.push_back(SegmentPtr(new Segment((*inspectLV)->get_upper_point(),(*inspectLV)->get_position())));
		  break;
	  case 3:
		  std::cout<<"Type Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  if(!(*inspectLV)->is_obstacles_upper){
			segment_intersect = segment_upper;
			list_point_of_space.push_back((*inspectLV)->get_upper_point());
			list_point_of_space.push_back(Vertices::get_point_x_litte(segment_upper));
			list_point_of_space.push_back(segment_upper%segment_temp);
			list_point_of_space.push_back((*inspectLV)->get_position());
		  }else{
			segment_intersect = segment_below;
			list_point_of_space.push_back(Vertices::get_point_x_litte(segment_below));
			list_point_of_space.push_back((*inspectLV)->get_upper_point());
			list_point_of_space.push_back((*inspectLV)->get_position());
			list_point_of_space.push_back(segment_below%segment_temp);
		  }
		  Vertices::update_list_vertices(list_vertices, Vertices::get_point_x_large(segment_intersect),
				  	  	  	 Vertices::get_point_x_litte(segment_intersect), segment_temp%segment_intersect);
		  Vertices::update_list_segment(list_segment,segment_intersect, segment_intersect%segment_temp);
		  list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_upper_point(),(*inspectLV)->get_position())));
		  list_segment.push_back(SegmentPtr(new Segment((*inspectLV)->get_below_point(),(*inspectLV)->get_position())));
		  break;
	  case 4:
		  std::cout<<"Type Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  if(!(*inspectLV)->is_obstacles_upper){
		 	list_point_of_space.push_back(Vertices::get_point_x_litte(segment_below));
		 	list_point_of_space.push_back(Vertices::get_point_x_litte(segment_upper));
		 	list_point_of_space.push_back(segment_upper%segment_temp);
		 	list_point_of_space.push_back(segment_below%segment_temp);
			Vertices::update_list_vertices(list_vertices, Vertices::get_point_x_large(segment_upper),
					  Vertices::get_point_x_litte(segment_upper), segment_temp%segment_upper);
			Vertices::update_list_segment(list_segment,segment_upper, segment_upper%segment_temp);
			Vertices::update_list_vertices(list_vertices, Vertices::get_point_x_large(segment_below),
					  Vertices::get_point_x_litte(segment_below), segment_temp%segment_below);
			Vertices::update_list_segment(list_segment,segment_below, segment_below%segment_temp);
		 	}
		  	list_segment.push_back(SegmentPtr(new Segment((*inspectLV)->get_below_point(),(*inspectLV)->get_position())));
		  	list_segment.push_back(SegmentPtr(new Segment((*inspectLV)->get_upper_point(),(*inspectLV)->get_position())));
		  break;
	  case 5:
		  std::cout<<"Type Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  if((*inspectLV)->is_obstacles_upper||(!(*inspectLV)->is_of_obstacles)){
			  temp_point = (*inspectLV)->get_upper_point();
			  ++inspectLV;
			  list_point_of_space.push_back((*inspectLV)->get_below_point());
			  list_point_of_space.push_back(temp_point);
			  list_point_of_space.push_back((*inspectLV)->get_upper_point());
			  list_point_of_space.push_back((*inspectLV)->get_position());
		  }
		  break;
	  case 6:
		  std::cout<<"\nType Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  break;
	  default:
		  std::cout<<"\nType Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  break;
	  }
	  if(Space::list_point_fit(list_point_of_space)){
		  list_space.push_back(SpacePtr(new Space(list_point_of_space)));
	  }
  }
  std::cout << "Starting add parent!" << "\n";
  list_space.sort(Space::compare_positions_x);
  Space::print_list_space(list_space);
  std::cout << list_space.size() << "\n";
//  for (inspectLS = --list_space.end(), i = 1; inspectLS != list_space.end();
//      --inspectLS) {
//    for (inspectLS_temp = list_space.begin(), i = 1;
//        inspectLS_temp != list_space.end(); ++inspectLS_temp) {
//      if (Space::is_parent(*inspectLS_temp, *inspectLS)) {
//        (*inspectLS_temp)->children.push_back(*inspectLS);
//        (*inspectLS)->set_parent(*inspectLS_temp);
//        (*inspectLS)->set_point_backtrack(*inspectLS_temp, *inspectLS,
//            robot_size);
//        break;
//      }
//      std::cout << "Find \n";
//    }
//  }
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
    		temp->set_is_obstacles_upper(*inspectLO, environment->get_height());
    		temp->set_is_obstacles_below(*inspectLO, environment->get_height());
    	}
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
  SegmentPtr s1,s2;
  int i, j;
  PointPtr point_temp;

   //Create vertices
  list_vertices = create_list_vertices(map->get_boundary(),
      map->get_extendedmap_obstacles());

  //Create Space
  list_space = create_list_space(map->get_boundary(), list_vertices);



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
