/*
 * vector.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: sontd
 */

#include "../../include/common/vector.hpp"

namespace wandrian {
namespace common {

Vector::Vector() :
    x(0), y(-1) {
}

Vector::Vector(double x, double y) :
    x(x), y(y) {
}

Vector::Vector(const Vector &vector) :
    x(vector.x), y(vector.y) {
}

void Vector::rotate_counterclockwise() {
  double tmp = x;
  x = -y;
  y = tmp;
}

}
}
