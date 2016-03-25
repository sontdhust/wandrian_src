#include "../../../include/environment/boustrophedon/free_zone.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

FreeZone::FreeZone(PointPtr center, double size_x, double size_y) :
    center(center), size_x(size_x), size_y(size_y), status_visited(false) {
  points.insert(points.end(),
      PointPtr(new Point(center->x - size_x / 2, center->y + size_y / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size_x / 2, center->y + size_y / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + size_x / 2, center->y - size_y / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x - size_x / 2, center->y - size_y / 2)));
  build();
}

bool FreeZone::compare_positions_x(boost::shared_ptr<FreeZone> space1,
    boost::shared_ptr<FreeZone> space2) {
  if (space1->get_center()->x < space2->get_center()->x)
    return true;
  return false;
}

bool FreeZone::is_parent(boost::shared_ptr<FreeZone> space1,
    boost::shared_ptr<FreeZone> space2) {
  if ((space1->get_center()->x + space1->get_size_x() / 2)
      == (space2->get_center()->x - space2->get_size_x() / 2)) {
    if ((space1->get_center()->y - space1->get_size_y() / 2
        > space2->get_center()->y + space2->get_size_y() / 2)
        || (space1->get_center()->y + space1->get_size_y() / 2
            < space2->get_center()->y - space2->get_size_y() / 2)) {
      return false;
    }
    return true;
  }
  return false;
}

PointPtr FreeZone::get_center() {
  return center;
}

double FreeZone::get_size_x() {
  return size_x;
}

double FreeZone::get_size_y() {
  return size_y;
}

FreeZonePtr FreeZone::get_parent() {
  return parent;
}

void FreeZone::set_parent(FreeZonePtr parent) {
  this->parent = parent;
}

void FreeZone::set_point_backtrack(PointPtr point_backtrack) {
  this->point_backtrack = point_backtrack;
}

}
}
}

