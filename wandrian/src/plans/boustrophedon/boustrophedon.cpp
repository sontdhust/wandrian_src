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
bool Boustrophedon::go_with(PointPtr position, bool flexibility){
	PointPtr last_position, new_position;
	int flag;
	last_position = path.back();
	if(!last_position){
		return go_to(position, flexibility);
	}
	if((position->x -last_position->x)>EPSILON){
		return go_to(position, flexibility);
	}
	if((last_position->y - position->y)>EPSILON){
		flag = -1;
	}else{
		flag = 1;
	}
	for( int i = 1; i*robot_size/2 < std::abs(position->y - last_position->y);i++){
		new_position = PointPtr(new Point(position->x, last_position->y + i*robot_size*flag/2));
		go_to(new_position, flexibility);
	}
	std::cout << "Last position: " << last_position->x << "," << last_position->y << "\n";
	return go_to(position, flexibility);
}
bool Boustrophedon::go_into(SpacePtr space, double size_y) {
  double x,y;
  double d_upper, d_below;
  PointPtr starting_point, last_position,new_position;
  PointPtr intersect_upper, intersect_below;
  SegmentPtr segment_cut;
  VerticesPtr below_vertices;
  double flag;

  below_vertices = space->get_vertices_below();
  go_with(space->starting_point, STRICTLY);
  std::cout << "\033[1;34mStarting Point-\033[0m\033[1;31m\033[0m: "
      << space->starting_point->x << "," << space->starting_point->y << "\n";
  flag = 1;
  for (int i = 0; i < int((below_vertices->get_below_point()->x-below_vertices->get_position()->x)
		  / robot_size + EPSILON); ++i) {
      segment_cut = SegmentPtr(new Segment(below_vertices->get_position()->x + (i*2+1)*robot_size/2,
    		  	  	  	  	  	  	  	   below_vertices->get_position()->y - size_y,
    		  	  	  	  	  	  	  	   below_vertices->get_position()->x + (i*2+1)*robot_size/2,
    		  	  	  	  	  	  	  	   below_vertices->get_position()->y + 2*size_y));
      intersect_below = segment_cut%space->segment_below;
      intersect_upper = segment_cut%space->segment_upper;
    if (i != 0) {
      last_position = path.back();
      if(flag > 0){
    	 new_position = PointPtr(
    			 	 	 new Point(intersect_below->x , intersect_below->y + space->vary_below));
    	 go_with(new_position, STRICTLY);
      }else{
    	  new_position = PointPtr(
    			  	  	 new Point(intersect_upper->x , intersect_upper->y - space->vary_upper));
    	  go_with(new_position, STRICTLY);
      }
    }
    last_position = path.back();
    if(flag>0){
      new_position = PointPtr(new Point(last_position->x, intersect_upper->y - space->vary_upper));
    }else{
      new_position = PointPtr(new Point(last_position->x, intersect_below->y + space->vary_below));
    }
    go_with(new_position, STRICTLY);
    flag = -flag;
  }
  return true;
}

void Boustrophedon::dfs(SpacePtr space, double size_y) {
  std::list<SpacePtr>::iterator inspectLC;
  space->status_visited = true;
  double x, y;
  go_into(space, size_y);
  map->number_space_need_visit--;
  for (inspectLC = space->children.begin(); inspectLC != space->children.end();
      ++inspectLC) {
    if ((*inspectLC)->status_visited == false) {
      go_with((*inspectLC)->backtrack_point, STRICTLY);
      go_with(
          PointPtr(
              new Point((*inspectLC)->backtrack_point->x + robot_size,
                  (*inspectLC)->backtrack_point->y)), STRICTLY);
      dfs(*inspectLC, size_y);
      if(map->number_space_need_visit >= 1){
    	  go_with(
    	        PointPtr(
    	        new Point((*inspectLC)->backtrack_point->x + robot_size,
    	                  (*inspectLC)->backtrack_point->y)), STRICTLY);
    	  go_with((*inspectLC)->backtrack_point, STRICTLY);
      }
    }
  }
  if ((space->backtrack_point)&&(map->number_space_need_visit >= 1)) {
	  std::cout<<space->get_vertices_below()->get_below_point()->x
              - robot_size / 2<<" ,"<<space->backtrack_point->y<<"\n";
    go_with(
        PointPtr(
            new Point(
                space->get_vertices_below()->get_below_point()->x
                    - robot_size / 2, space->backtrack_point->y)), STRICTLY);
  }
}

std::list<SpacePtr> Boustrophedon::create_list_space(RectanglePtr environment,
    std::list<VerticesPtr> list_vertices) {
  std::list<VerticesPtr>::iterator inspectLV;
  VerticesPtr temp_vertices;
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
			list_space.push_back(SpacePtr(new Space(list_point_of_space, robot_size)));
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
		  Vertices::update_list_vertices(list_vertices,  Vertices::get_point_x_large(segment_intersect),
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
		  if((!(*inspectLV)->is_of_obstacles)){
			  temp_point = (*inspectLV)->get_upper_point();
			  ++inspectLV;
			  list_point_of_space.push_back((*inspectLV)->get_below_point());
			  list_point_of_space.push_back(temp_point);
			  list_point_of_space.push_back((*inspectLV)->get_upper_point());
			  list_point_of_space.push_back((*inspectLV)->get_position());
			  break;
		  }
		 if(!(*inspectLV)->is_obstacles_below){
			 segment_intersect = segment_below;
			 list_point_of_space.push_back(Vertices::get_point_x_litte(segment_below));
			 list_point_of_space.push_back((*inspectLV)->get_below_point());
			 list_point_of_space.push_back((*inspectLV)->get_position());
			 list_point_of_space.push_back(segment_below%segment_temp);
			 list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_below_point(),(*inspectLV)->get_position())));
		 }
		 else{
			if(!(*inspectLV)->is_obstacles_upper){
			 segment_intersect = segment_upper;
			 list_point_of_space.push_back((*inspectLV)->get_upper_point());
			 list_point_of_space.push_back(Vertices::get_point_x_litte(segment_upper));
			 list_point_of_space.push_back(segment_upper%segment_temp);
			 list_point_of_space.push_back((*inspectLV)->get_position());
			 list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_upper_point(),(*inspectLV)->get_position())));
			}else{
			 temp_vertices = VerticesPtr(new Vertices((*inspectLV)->get_position(),(*inspectLV)->get_below_point(),
					 	 	 	 	 	 	 	 	  (*inspectLV)->get_upper_point(),true));
			 ++inspectLV;
			 std::cout<<"Temp pre: ("<< temp_vertices->get_position()->x<<" ,"<<temp_vertices->get_position()->y<<" )\n";
			 std::cout<<"Temp pre: ("<< (*inspectLV)->get_position()->x<<" ,"<<(*inspectLV)->get_position()->y<<" )\n";
			 if((*inspectLV)->is_obstacles_below){
				 list_point_of_space.push_back((*inspectLV)->get_below_point());
				 list_point_of_space.push_back(temp_vertices->get_upper_point());
				 list_point_of_space.push_back(temp_vertices->get_position());
				 list_point_of_space.push_back((*inspectLV)->get_position());
				 list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_upper_point(),(*inspectLV)->get_position())));
				 list_segment.remove(SegmentPtr(new Segment((*inspectLV)->get_below_point(),(*inspectLV)->get_position())));
				 std::cout<<"break khong?\n";
				 break;
			 }else{
				 segment_intersect = Vertices::get_segment_contain_nearest_intersect_point(list_segment,
			 			  (*inspectLV)->get_position(),-environment->get_height());
				 list_point_of_space.push_back(Vertices::get_point_x_litte(segment_intersect));
				 list_point_of_space.push_back((temp_vertices)->get_upper_point());
				 list_point_of_space.push_back((temp_vertices)->get_position());
				 list_point_of_space.push_back(segment_intersect%segment_temp);
			 }
			}
		 }
		 Vertices::update_list_vertices(list_vertices,  Vertices::get_point_x_large(segment_intersect),
						  	  	  	 Vertices::get_point_x_litte(segment_intersect), segment_temp%segment_intersect);
		 Vertices::update_list_segment(list_segment,segment_intersect, segment_intersect%segment_temp);
		 break;
	  case 6:
		  std::cout<<"\nType Vertices :"<<(*inspectLV)->type_vertice<<"\n";

		  break;
	  default:
		  std::cout<<"\nType Vertices :"<<(*inspectLV)->type_vertice<<"\n";
		  break;
	  }

	  if(Space::list_point_fit(list_point_of_space)){
		  list_space.push_back(SpacePtr(new Space(list_point_of_space, robot_size)));
	  }else{
		  std::cout<<"Can't crate space!!!!\n";
	  }
  }
  std::cout << "Starting add parent!" << "\n";
  list_space.sort(Space::compare_positions_x);
  Space::print_list_space(list_space);
  std::cout << list_space.size() << "\n";
  for (inspectLS = --list_space.end(), i = 1; inspectLS != list_space.end();
      --inspectLS) {
    (*inspectLS)->set_stating_point(environment->get_height(), robot_size);
    for (inspectLS_temp = list_space.begin(), i = 1;
        inspectLS_temp != list_space.end(); ++inspectLS_temp) {
      if (Space::is_parent(*inspectLS_temp, *inspectLS)) {
        (*inspectLS_temp)->children.push_back(*inspectLS);
        (*inspectLS)->set_parent(*inspectLS_temp);
        (*inspectLS)->set_point_backtrack(*inspectLS_temp, robot_size, environment->get_height());
        break;
      }
    }
  }
  Space::print_list_space(list_space);
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
  for (inspectLO = list_obstacle.begin(), i = 1;
      inspectLO != list_obstacle.end(); ++inspectLO) {
    list_point = (*inspectLO)->get_points();
    is_of_obstacle = true;
    if (inspectLO == list_obstacle.begin()) {
      is_of_obstacle = false;
    }
    inspectLP = --list_point.end();
    near_point = PointPtr(new Point((*inspectLP)->x, (*inspectLP)->y));
    for (inspectLP = list_point.begin(), j = 0; j < list_point.size() + 1;
        ++inspectLP) {
      j++;
      if (inspectLP == list_point.begin()) {
        position = PointPtr(new Point((*inspectLP)->x, (*inspectLP)->y));
        continue;
      }
      if (inspectLP == list_point.end()) {
        inspectLP = list_point.begin();
      }
      temp = VerticesPtr(
          new Vertices(position, near_point, *inspectLP, is_of_obstacle));
      if (is_of_obstacle) {
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
  SegmentPtr s1, s2;
  int i, j;
  PointPtr point_temp;

  //Create vertices
  list_vertices = create_list_vertices(map->get_boundary(),
      map->get_extended_obstacles());

  //Create Space
  list_space = create_list_space(map->get_boundary(), list_vertices);

  map->number_space_need_visit = list_space.size();
  for (inspectLS = list_space.begin(), i = 1; inspectLS != list_space.end();
      ++inspectLS) {
    i++;
    std::cout << "Space:" << ": " << std::endl;
    if ((*inspectLS)->status_visited == false) {
      dfs(*inspectLS, map->get_boundary()->get_height());
    }
  }
}

}
}
}
