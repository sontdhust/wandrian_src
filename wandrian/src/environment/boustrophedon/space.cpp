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

Space::Space(std::list<PointPtr> list_point):status_visited(false){
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
	build();
}

bool Space::compare_positions_x(boost::shared_ptr<Space> space1,
    boost::shared_ptr<Space> space2) {
//  if (space1->get_center()->x < space2->get_center()->x)
//    return true;
  return false;
}
//Space1 is parent space2.
bool Space::is_parent(boost::shared_ptr<Space> space1,
    boost::shared_ptr<Space> space2) {

  return false;
}

SpacePtr Space::get_parent() {
  return parent;
}
bool Space::list_point_fit(std::list<PointPtr> list_point){
	std::list<PointPtr>::iterator u;
	double temp;
	if((list_point.size()<2)||(list_point.size()>4)) return false;
	u = list_point.begin();
	temp = (*u)->x;
	u = --list_point.end();
	if(std::abs(temp - (*u)->x) <EPSILON) return false;
	return true;
}
VerticesPtr Space::get_vertices_upper(){
	return vertices_upper;
}

void Space::set_vertices_upper(VerticesPtr verticesUpper) {
	this->vertices_upper = verticesUpper;
}
VerticesPtr Space::get_vertices_below(){
	return vertices_below;
}

void Space::set_vertices_below(VerticesPtr verticesBelow) {
	this->vertices_below = verticesBelow;
}

void Space::set_parent(SpacePtr parent) {
  this->parent = parent;
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
	    for (inspectLS_child = (*inspectLS)->children.begin(), j = 1;
	        inspectLS_child != (*inspectLS)->children.end(); ++inspectLS_child) {
	      std::cout << "Children: "<< j++ <<" :"<< std::endl;
		  std::cout<<"Point below : ("<<(*inspectLS)->get_vertices_below()->get_position()->x
				  <<"," <<(*inspectLS)->get_vertices_below()->get_position()->y <<")\n";
		  std::cout<<"Point upper : ("<<(*inspectLS)->get_vertices_upper()->get_position()->x
				  <<"," <<(*inspectLS)->get_vertices_upper()->get_position()->y <<")\n";
	    }
	    std::cout << " " << std::endl;
	 }
}
void Space::set_point_backtrack(boost::shared_ptr<Space> space1,
								boost::shared_ptr<Space> space2, double robot_size) {
// if((space1->get_center()->y - space1->get_size_y() / 2
//   > space2->get_center()->y - space2->get_size_y() / 2)){
//   	this->point_backtrack = PointPtr(new Point(space1->get_center()->x + space1->get_size_x()/2 -robot_size/2,
//    									   space1->get_center()->y - space1->get_size_y()/2+ robot_size/2));
// }else{
//   	this->point_backtrack = PointPtr(new Point(space1->get_center()->x + space1->get_size_x()/2 -robot_size/2,
//   	    							   	   	   space2->get_center()->y - space2->get_size_y()/2+robot_size/2));
// }
}

}
}
}
