/*
 * space.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/environment/boustrophedon/space.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

Space::Space(std::list<PointPtr> list_point, double robot_size):status_visited(false){
	PointPtr t1,t2,t3,t4;
	std::list<PointPtr>::iterator u;
	int i;
	for(u = list_point.begin(), i= 0; u!= list_point.end(); ++u){
		std::cout<<"\n Point "<< (i++) + 1 <<" : ("<< (*u)->x <<" ,"<<(*u)->y <<" )\n";
		points.insert(points.end(), PointPtr(new Point((*u)->x, (*u)->y)));
		if(i==1){
			t1 = PointPtr(new Point((*u)->x, (*u)->y));
			continue;
		}
		if(i==2){
			t2 = PointPtr(new Point((*u)->x, (*u)->y));
			continue;}
		if(i==3){
			t3 = PointPtr(new Point((*u)->x, (*u)->y));
		}
		t4 = PointPtr(new Point((*u)->x, (*u)->y));
	}
	set_vertices_below(VerticesPtr(new Vertices(t1,t4,t2, false)));
	set_vertices_upper(VerticesPtr(new Vertices(t2,t1,t3, false)));
	this->vertices_below->set_below_point(t4);
	this->vertices_below->set_upper_point(t2);
	this->vertices_upper->set_below_point(t1);
	this->vertices_upper->set_upper_point(t3);
	this->segment_below = SegmentPtr(new Segment(t1->x, t1->y, t4->x,t4->y));
	this->segment_upper = SegmentPtr(new Segment(t2->x, t2->y, t3->x,t3->y));
	this->set_vary(robot_size);
	build();
}

bool Space::compare_positions_x(boost::shared_ptr<Space> space1,
    boost::shared_ptr<Space> space2) {
  if ((space2->get_vertices_below()->get_position()->x
      - space1->get_vertices_below()->get_position()->x) > EPSILON) {
    return true;
  }
  return false;
}

//Space1 is parent space2 ?
bool Space::is_parent(boost::shared_ptr<Space> space1,
    boost::shared_ptr<Space> space2) {
	if (fabs(space1->get_vertices_below()->get_below_point()->x -
			space2->get_vertices_below()->get_position()->x )< EPSILON) {
	    if (((space1->get_vertices_below()->get_below_point()->y -
	    	 space2->get_vertices_upper()->get_position()->y)> EPSILON)
	    	||((space2->get_vertices_below()->get_position()->y -
	            space1->get_vertices_upper()->get_upper_point()->y)> EPSILON)) {
	      return false;
	    }
	    return true;
	  }
	return false;
}


bool Space::list_point_fit(std::list<PointPtr> list_point){
	std::list<PointPtr>::iterator u;
	double temp;
	if((list_point.size()<2)||(list_point.size()>4)) return false;
	for (u = list_point.begin();  u!= list_point.end() ; ++u) {
		std::cout<<"Point : ( "<<(*u)->x<<" ,"<<(*u)->y <<")\n";
	}
	u = list_point.begin();
	temp = (*u)->x;
	u = --list_point.end();
	if(std::abs(temp - (*u)->x) <EPSILON) return false;
	return true;
}
VerticesPtr Space::get_vertices_upper(){
	return vertices_upper;
}
VerticesPtr Space::get_vertices_below(){
	return vertices_below;
}

SpacePtr Space::get_parent() {
  return parent;
}

void Space::set_vary(double robot_size){
	double temp_x, temp_y;
	temp_x = this->get_vertices_below()->get_below_point()->x - this->get_vertices_below()->get_position()->x;
	temp_y = this->get_vertices_below()->get_below_point()->y - this->get_vertices_below()->get_position()->y;
    this->vary_below =(robot_size* std::sqrt(std::pow(temp_x,2)+std::pow(temp_y,2)))/(2*temp_x);
    if(std::abs(this->vary_below-robot_size/2)>EPSILON){
    	this->vary_below = this->vary_below + robot_size/5;
    }
    temp_x = this->get_vertices_upper()->get_upper_point()->x - this->get_vertices_upper()->get_position()->x;
    temp_y = this->get_vertices_upper()->get_upper_point()->y - this->get_vertices_upper()->get_position()->y;
    this->vary_upper =(robot_size* std::sqrt(std::pow(temp_x,2)+std::pow(temp_y,2)))/(2*temp_x);
    if(std::abs(this->vary_upper-robot_size/2)>EPSILON){
       	this->vary_upper = this->vary_upper + robot_size/5;
    }
}
void Space::set_stating_point(double size_y, double robot_size){
	SegmentPtr segment_cut;
	PointPtr temp_point;
	double flag;
	segment_cut = SegmentPtr(new Segment(vertices_below->get_position()->x + robot_size/2,
	 	  	  	  	  	  	  	  	     vertices_below->get_position()->y - size_y,
			  	  	  	  	  	  	  	 vertices_below->get_position()->x + robot_size/2,
			  	  	  	  	  	  	  	 vertices_below->get_position()->y + 2*size_y));
	temp_point = segment_cut%segment_below;
	this->starting_point = PointPtr(new Point(temp_point->x, temp_point->y + this->vary_below));
}

void Space::set_vertices_upper(VerticesPtr verticesUpper) {
  this->vertices_upper = verticesUpper;
}

void Space::set_vertices_below(VerticesPtr verticesBelow) {
  this->vertices_below = verticesBelow;
}
void Space::set_parent(SpacePtr parent) {
  this->parent = parent;
}

void Space::set_point_backtrack(boost::shared_ptr<Space> space1,
								double robot_size, double size_y) {
 SegmentPtr segment_cut;
 PointPtr temp_point, temp_point1;
 if((space1->vertices_below->get_upper_point()->x -
	 space1->vertices_below->get_below_point()->x) > EPSILON){
	 temp_point = space1->vertices_below->get_upper_point();
 }else {
	 temp_point = space1->vertices_below->get_below_point();
 }
 segment_cut = SegmentPtr(new Segment(temp_point->x - robot_size/2,
	 	  	  	  	  	  	  	  	  temp_point->y - size_y,
			  	  	  	  	  	  	  temp_point->x - robot_size/2,
			  	  	  	  	  	  	  temp_point->y + 2*size_y));
 temp_point1 = segment_cut%(space1->segment_below);
 temp_point1->y = temp_point1->y + space1->vary_below;
 
 if((((temp_point->y + robot_size/2 - this->get_vertices_below()->get_position()->y)> EPSILON)||
	 (temp_point1->y - this->get_vertices_below()->get_position()->y)>EPSILON)){
	if (temp_point->y + robot_size/2 - temp_point1->y > EPSILON){
		this->backtrack_point = PointPtr(new Point(temp_point->x -robot_size/2, temp_point->y +robot_size/2));
	}else{
		this->backtrack_point = PointPtr(new Point(temp_point->x -robot_size/2, temp_point1->y ));
	}
 }else{
	 std::cout<<"BELow: "<<temp_point->x <<", "<<this->get_vertices_below()->get_position()->y<" )\n";
	if((this->starting_point->y-this->get_vertices_below()->get_position()->y-this->vary_below)>
		EPSILON){
	   	this->backtrack_point = PointPtr(new Point(
	   			space1->get_vertices_below()->get_below_point()->x -robot_size/2,
	   		    this->starting_point->y));
	}else{
	   	this->backtrack_point = PointPtr(new Point(
		space1->get_vertices_below()->get_below_point()->x -robot_size/2,
		this->get_vertices_below()->get_position()->y + robot_size/2 ));
	}
 }
 std::cout<<"POINT Backtrack: "<<this->backtrack_point->x <<", "<<this->backtrack_point->y<<" )\n";
}
void Space::print_list_space(std::list< boost::shared_ptr<Space> > list_space){
	std::list<boost::shared_ptr<Space> >::iterator inspectLS;
	std::list<boost::shared_ptr<Space> >::iterator inspectLS_child;
	int i,j;
	for (inspectLS = list_space.begin(), i = 1; inspectLS != list_space.end();
	      ++inspectLS) {
		  std::cout<<" Space "<<i++<< ": \n";
		  std::cout<<"Point below : ("<<(*inspectLS)->get_vertices_below()->get_position()->x
				  <<"," <<(*inspectLS)->get_vertices_below()->get_position()->y <<" )\n";
		  std::cout<<"Point upper : ("<<(*inspectLS)->get_vertices_upper()->get_position()->x
				  <<"," <<(*inspectLS)->get_vertices_upper()->get_position()->y <<")\n";
		if((*inspectLS)->backtrack_point){
			std::cout<<"Point Backtrack : ("<<(*inspectLS)->backtrack_point->x <<" ,"<<
											   (*inspectLS)->backtrack_point->y<<" )\n";
		}
	    for (inspectLS_child = (*inspectLS)->children.begin(), j = 1;
	        inspectLS_child != (*inspectLS)->children.end(); ++inspectLS_child) {
	      std::cout << "Children: "<< j++ <<" :"<< std::endl;
		  std::cout<<"Point below : ("<<(*inspectLS_child)->get_vertices_below()->get_position()->x
				  <<"," <<(*inspectLS_child)->get_vertices_below()->get_position()->y <<")\n";
		  std::cout<<"Point upper : ("<<(*inspectLS_child)->get_vertices_upper()->get_position()->x
				  <<"," <<(*inspectLS_child)->get_vertices_upper()->get_position()->y <<")\n";
	    }
	    std::cout << " " << std::endl;
	 }
}
}
}
}
