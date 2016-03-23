
#include "../../../include/plans/boustrophedon/space.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon {

Space::Space(PointPtr center, double sizex, double sizey) :
    center(center), sizex(sizex), sizey(sizey),status_visited(false) {
  points.insert(points.end(),
      PointPtr(new Point(center->x - sizex / 2, center->y + sizey / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + sizex / 2, center->y + sizey / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + sizex / 2, center->y - sizey / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x - sizex / 2, center->y - sizey / 2)));
  build();
}

bool Space::compare_positionsx(boost::shared_ptr<Space> space1, boost::shared_ptr<Space> space2 ){
 	if(space1->get_center()->x < space2->get_center()->x)
 		return true;
 	return false;
 };

bool Space::is_parent(boost::shared_ptr<Space> space1, boost::shared_ptr<Space> space2 ){
 	if((space1->get_center()->x + space1->get_sizex()/2)  == (space2->get_center()->x - space2->get_sizex()/2)){

 		if((space1->get_center()->y - space1->get_sizey()/2 > space2->get_center()->y + space2->get_sizey()/2)||
 		   (space1->get_center()->y + space1->get_sizey()/2 < space2->get_center()->y - space2->get_sizey()/2)){
 			return false;
 		}

 		return true;
 	}
 	return false;
 };

PointPtr Space::get_center() {
  return center;
}

double Space::get_sizex() {
  return sizex;
}
double Space::get_sizey() {
  return sizey;
}

void Space::set_parent(SpacePtr parent){

	this->parent = parent;
}
SpacePtr Space::get_parent() {
  return parent;
}

void Space::set_point_backtrack(PointPtr point_backtrack){
	this->point_backtrack = point_backtrack;
}

}
}
}

