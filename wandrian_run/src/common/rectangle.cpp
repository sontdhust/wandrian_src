/*
 * rectangle.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: cslab
 */

#include "../../include/common/rectangle.hpp"

namespace wandrian {
namespace common {

Rectangle::Rectangle(PointPtr center, double width, double height) :
    center(center), width(width), height(height) {
  points.insert(points.end(),
      PointPtr(new Point(center->x - width / 2, center->y + height / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + width / 2, center->y + height / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + width / 2, center->y - height / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x - width / 2, center->y - height / 2)));
  build();
}

Rectangle::~Rectangle() {
}

PointPtr Rectangle::get_center() const {
  return center;
}

double Rectangle::get_width() const {
  return width;
}

double Rectangle::get_height() const {
  return height;
}

}
}
