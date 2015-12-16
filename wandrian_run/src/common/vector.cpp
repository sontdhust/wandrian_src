/*
 * vector.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: sontd
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

VectorPtr Vector::rotate_counterclockwise() {
  return VectorPtr(new Vector(-y, x));
}

VectorPtr Vector::rotate_clockwise() {
  return VectorPtr(new Vector(y, -x));
}

}
}
