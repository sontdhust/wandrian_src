/*
 * partially_occupiable.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: sontd
 */

#include "../../include/environment/partially_occupiable.hpp"

namespace wandrian {
namespace environment {

PartiallyOccupiable::PartiallyOccupiable() :
    current_quadrant(IV) {
  for (int i = I; i <= IV; i++)
    quadrants[i] = NEW;
}

PartiallyOccupiable::~PartiallyOccupiable() {
}

PointPtr PartiallyOccupiable::find_position(Quadrant quadrant) {
  PointPtr center = _center();
  double size = _size();
  switch (quadrant) {
  case I:
    return PointPtr(new Point(center->x + size / 4, center->y + size / 4));
  case II:
    return PointPtr(new Point(center->x - size / 4, center->y + size / 4));
  case III:
    return PointPtr(new Point(center->x - size / 4, center->y - size / 4));
  case IV:
    return PointPtr(new Point(center->x + size / 4, center->y - size / 4));
  default:
    return PointPtr(new Point(center->x, center->y));
  }
}

PointPtr PartiallyOccupiable::get_current_position() {
  return find_position(current_quadrant);
}

Quadrant PartiallyOccupiable::get_current_quadrant() {
  return current_quadrant;
}

State* PartiallyOccupiable::get_quadrants() {
  return quadrants;
}

void PartiallyOccupiable::set_current_quadrant(Quadrant quadrant) {
  current_quadrant = quadrant;
  set_quadrants_state(quadrant, OLD);
}

void PartiallyOccupiable::set_quadrants_state(Quadrant quadrant, State state) {
  quadrants[quadrant] = state;
}

}
}
