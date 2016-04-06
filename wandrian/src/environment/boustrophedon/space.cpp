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

Space::Space(PointPtr center, double size_x, double size_y) :
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

bool Space::compare_positions_x(boost::shared_ptr<Space> space1,
    boost::shared_ptr<Space> space2) {
  if (space1->get_center()->x < space2->get_center()->x)
    return true;
  return false;
}

bool Space::is_parent(boost::shared_ptr<Space> space1,
    boost::shared_ptr<Space> space2) {
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

PointPtr Space::get_center() {
  return center;
}

double Space::get_size_x() {
  return size_x;
}

double Space::get_size_y() {
  return size_y;
}

SpacePtr Space::get_parent() {
  return parent;
}

void Space::set_parent(SpacePtr parent) {
  this->parent = parent;
}

void Space::set_point_backtrack(PointPtr point_backtrack) {
  this->point_backtrack = point_backtrack;
}

}
}
}
