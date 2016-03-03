
#include "../../../include/plans/boustrophedon_off/space.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon_off {

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

