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

Vector::Vector(double a) :
    x(cos(a)), y(sin(a)) {
}

Vector::Vector(double x, double y) :
    x(x), y(y) {
}

Vector::Vector(const Vector &vector) :
    x(vector.x), y(vector.y) {
}

void Vector::rotate_counterclockwise() {
  double d = x;
  x = -y;
  y = d;
}

void Vector::rotate_clockwise() {
  double d = x;
  x = y;
  y = -d;
}

double Vector::get_magnitude() {
  return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

double Vector::get_angle() {
  return atan2(y, x);
}

}
}
