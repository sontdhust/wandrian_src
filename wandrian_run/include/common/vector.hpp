/*
 * vector.hpp
 *
 *  Created on: Sep 16, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_VECTOR_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_VECTOR_HPP_

#include "point.hpp"

namespace wandrian {
namespace common {

enum Orientation {
  //                      IN_FRONT (1)
  //                           |
  //                        ___|___
  //                       |   |   |
  // AT_LEFT_SIDE (2) _____|___|___|_____ AT_RIGHT_SIDE (0)
  //                       |   |   |
  //                       |___|___|
  //                           |
  //                           |
  //                       BEHIND (3)

  AT_RIGHT_SIDE,
  IN_FRONT,
  AT_LEFT_SIDE,
  BEHIND
};

struct Vector {

  double x, y;

  Vector();
  Vector(double, double);
  Vector(const Vector&);
  void rotate_counterclockwise();
  void rotate_clockwise();
};

typedef boost::shared_ptr<Vector> VectorPtr;

inline VectorPtr operator*(VectorPtr v, const double &k) {
  return VectorPtr(new Vector(v->x * k, v->y * k));
}

inline VectorPtr operator/(VectorPtr v, const double &k) {
  return VectorPtr(new Vector(v->x / k, v->y / k));
}

inline PointPtr operator+(PointPtr p, VectorPtr v) {
  return PointPtr(new Point(p->x + v->x, p->y + v->y));
}

inline VectorPtr operator-(PointPtr p1, PointPtr p2) {
  return VectorPtr(new Vector(p1->x - p2->x, p1->y - p2->y));
}

inline double operator^(VectorPtr v1, VectorPtr v2) {
  double a1 = atan2(v1->y, v1->x) - atan2(v2->y, v2->x);
  double a2 = (a1 > 0) ? a1 - 2 * M_PI : a1 + 2 * M_PI;
  return (std::abs(a1) < std::abs(a2)) ? a1 : a2;
}

inline Orientation operator%(VectorPtr v1, VectorPtr v2) {
  double angle = v1 ^ v2;
  if (std::abs(angle) >= 3 * M_PI_4)
    return IN_FRONT;
  else if (std::abs(angle) <= M_PI_4)
    return BEHIND;
  else if (angle > 0)
    return AT_RIGHT_SIDE;
  else
    return AT_LEFT_SIDE;
}

inline VectorPtr operator++(VectorPtr v) {
  v->rotate_counterclockwise();
  return v;
}

inline VectorPtr operator++(VectorPtr v, int) {
  VectorPtr vector = VectorPtr(new Vector(*v));
  v->rotate_counterclockwise();
  return vector;
}

inline VectorPtr operator--(VectorPtr v) {
  v->rotate_clockwise();
  return v;
}

inline VectorPtr operator--(VectorPtr v, int) {
  VectorPtr vector = VectorPtr(new Vector(*v));
  v->rotate_clockwise();
  return vector;
}

inline Orientation operator~(VectorPtr o) {
  return o % VectorPtr(new Vector());
}

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_VECTOR_HPP_ */
