#include "../../../include/environment/boustrophedon/freezone.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

FreeZone::FreeZone(PointPtr center, double sizex, double sizey) :
    center(center), sizex(sizex), sizey(sizey), status_visited(false) {
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

bool FreeZone::compare_positionsx(boost::shared_ptr<FreeZone> space1,
    boost::shared_ptr<FreeZone> space2) {
  if (space1->get_center()->x < space2->get_center()->x)
    return true;
  return false;
}
;

bool FreeZone::is_parent(boost::shared_ptr<FreeZone> space1,
    boost::shared_ptr<FreeZone> space2) {
  if ((space1->get_center()->x + space1->get_sizex() / 2)
      == (space2->get_center()->x - space2->get_sizex() / 2)) {

    if ((space1->get_center()->y - space1->get_sizey() / 2
        > space2->get_center()->y + space2->get_sizey() / 2)
        || (space1->get_center()->y + space1->get_sizey() / 2
            < space2->get_center()->y - space2->get_sizey() / 2)) {
      return false;
    }

    return true;
  }
  return false;
}
;

PointPtr FreeZone::get_center() {
  return center;
}

double FreeZone::get_sizex() {
  return sizex;
}
double FreeZone::get_sizey() {
  return sizey;
}

void FreeZone::set_parent(FreeZonePtr parent) {

  this->parent = parent;
}
FreeZonePtr FreeZone::get_parent() {
  return parent;
}

void FreeZone::set_point_backtrack(PointPtr point_backtrack) {
  this->point_backtrack = point_backtrack;
}

}
}
}

