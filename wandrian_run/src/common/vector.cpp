/*
 * vector.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: anhnt
 */

#include "../../include/common/vector.hpp"

namespace wandrian {
namespace common {

Vector::Vector(double x, double y) :
    x(x), y(y) {
}

Vector::Vector(const Vector &vector) :
    x(vector.x), y(vector.y) {
}

VectorPtr Vector::rotate_counterclockwise_left() {
  return VectorPtr(new Vector(-y, x));
}
VectorPtr Vector::rotate_counterclockwise_right() {
  return VectorPtr(new Vector(y, -x));
}
VectorPtr Vector::rotate_counterclockwise_180() {
  return VectorPtr(new Vector(-x, -y));
}

}
}
