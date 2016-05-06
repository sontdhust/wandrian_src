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
		std::cout<<"Point of Space"<< (i++) + 1 <<" : "<< (*u)->x <<" ,"<<(*u)->y <<"\n";
		points.insert(points.end(), PointPtr(new Point((*u)->x, (*u)->y)));
		if(i==1){
			t1 = PointPtr(new Point((*u)->x, (*u)->y));
			std::cout<<"T1 ("<< t1->x <<" ,"<<t1->y <<" )\n";
			continue;}
		if(i==2){
			t2 = PointPtr(new Point((*u)->x, (*u)->y));
			std::cout<<"T2 ("<< t2->x <<" ,"<<t2->y <<") \n";
			continue;}
		if(i==3){
			t3 = PointPtr(new Point((*u)->x, (*u)->y));
			std::cout<<"T3 ("<< t3->x <<" ,"<<t3->y <<" )\n";
		}
		t4 = PointPtr(new Point((*u)->x, (*u)->y));
		std::cout<<"T4 ("<< t4->x <<" ,"<<t4->y <<" )\n";
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
//  if (fabs((space1->get_center()->x + space1->get_size_x() / 2)
//      -(space2->get_center()->x - space2->get_size_x() / 2))< EPSILON) {
//    if ((space1->get_center()->y - space1->get_size_y() / 2
//        > space2->get_center()->y + space2->get_size_y() / 2)
//        || (space1->get_center()->y + space1->get_size_y() / 2
//            < space2->get_center()->y - space2->get_size_y() / 2)) {
//      return false;
//    }
//    return true;
//  }
  return false;
}

SpacePtr Space::get_parent() {
  return parent;
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
		  //TODO: Print list point of space
	    for (inspectLS_child = (*inspectLS)->children.begin(), j = 1;
	        inspectLS_child != (*inspectLS)->children.end(); ++inspectLS_child) {
	      if (inspectLS_child == (*inspectLS)->children.begin()) {
	        std::cout << "Children:" << std::endl;
	      }
	     // TODO: Print list point of space children
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
