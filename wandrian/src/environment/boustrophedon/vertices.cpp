/*
 * vertices.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/environment/boustrophedon/vertices.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

Vertices::Vertices(PointPtr positions, PointPtr near_point1, PointPtr near_point2, bool is_of_obstacle){

  this->position = positions;
  if(near_point1->y > near_point2->y){
	  set_below_point(near_point2);
	  set_upper_point(near_point1);
  }else{
	  set_below_point(near_point1);
	  set_upper_point(near_point2);
  }
  set_is_of_obstacles(is_of_obstacle);
  set_type_vertices();
}
void Vertices::set_is_of_obstacles(bool is_of_obstacles){
	this->is_of_obstacles = is_of_obstacles;
}
void Vertices::set_type_vertices(){
	if(position->x<= below_point->x){
		if(position->x <= upper_point->x){
			type_vertice = 1;
		}else{
			type_vertice = 3;
		}
	}else{
		if(position->x >= upper_point->x){
			type_vertice = 4;
		}else{
			type_vertice = 2;
		}
	}
}
PointPtr Vertices::get_below_point(){
		return below_point;
 }
 void Vertices::set_below_point(PointPtr below_point) {
		this->below_point = below_point;
 }
 PointPtr Vertices::get_upper_point(){
	 return upper_point;
 }
 void Vertices::set_upper_point(PointPtr upper_point) {
		this->upper_point = upper_point;
 }

 void Vertices::set_is_obstacles_upper(PolygonPtr obstacles, double size_y){
	 std::list<PointPtr> list_point;
	 std::list<PointPtr>::iterator u;
	 PointPtr temp_p1, temp_p;
	 SegmentPtr temp1, temp2;
	 int i;
	 if(this->type_vertice == 1){
		 this->is_obstacles_upper = false;
		 std::cout<<"Type = 1 No is obstacles upper\n";
		 return;
	 }
	 temp1 = SegmentPtr(new Segment(position->x, position->y, position->x, position->y + size_y));
	 list_point = obstacles->get_points();
	 u = --list_point.end();
	 temp_p1 = PointPtr(new Point((*u)->x, (*u)->y));
	 for (u = list_point.begin(), i =0; u != list_point.end() ; ++u) {
		 temp2 = SegmentPtr(new Segment(temp_p1->x, temp_p1->y, (*u)->x, (*u)->y));
		 temp_p = temp1%temp2;
		 if(temp_p){
			if(temp_p!= position){
				std::cout<<"Point intersect :("<<temp_p->x<<" , "<<temp_p->y<<")\n";
			}
		 }
		 temp_p1 = PointPtr(new Point((*u)->x, (*u)->y));
	}
	 if(i%2 == 0){
		 this->is_obstacles_upper = false;
	 }else{
		 this->is_obstacles_upper = true;
	 }
 };

 void Vertices::set_is_obstacles_below(PolygonPtr obstacles, double size_y){
	 std::list<PointPtr> list_point;
	 std::list<PointPtr>::iterator u;
	 int i;
	 PointPtr temp_p1, temp_p;
	 SegmentPtr temp1, temp2;
	 if(this->type_vertice == 1){
		 this->is_obstacles_below = false;
		 std::cout<<"Type = 1 No is obstacles upper\n";
		 return;
	 }
	 temp1 = SegmentPtr(new Segment(position->x, position->y, position->x, position->y - size_y));
	 list_point = obstacles->get_points();
	 u = --list_point.end();
	 temp_p1 = PointPtr(new Point((*u)->x, (*u)->y));
	 for (u = list_point.begin(),i=0; u != list_point.end() ; ++u) {
		 temp2 = SegmentPtr(new Segment(temp_p1->x, temp_p1->y, (*u)->x, (*u)->y));
		 temp_p = temp1%temp2;
		 if(temp_p){
			if(temp_p!= position){
				i++;
				std::cout<<"Point intersect :("<<temp_p->x<<" , "<<temp_p->y<<")\n";
			}
		 }
		 temp_p1 = PointPtr(new Point((*u)->x, (*u)->y));
	}
	 if(i%2 == 0){
		 this->is_obstacles_below = false;
	 }else{
		 this->is_obstacles_below = true;
	 }
 };

bool Vertices::compare_vertices(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if ((vertices1->get_position()->x == vertices2->get_position()->x)
      || (vertices1->get_position()->y == vertices2->get_position()->y))
    return true;
  return false;
}

bool Vertices::compare_positions_x(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_position()->x < vertices2->get_position()->x)
    return true;
  return false;
}

bool Vertices::compare_positions_y(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_position()->y < vertices2->get_position()->y)
    return true;
  return false;
}

PointPtr Vertices::get_position() {
  return position;
}

void Vertices::set_positions(PointPtr positions) {
  this->position = positions;
}
}
}
}
