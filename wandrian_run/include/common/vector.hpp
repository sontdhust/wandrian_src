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
};

typedef boost::shared_ptr<Vector> VectorPtr;

inline Vector operator*(const Vector &v, const double &k) {
  return Vector(v.x * k, v.y * k);
}

inline Vector operator/(const Vector &v, const double &k) {
  return Vector(v.x / k, v.y / k);
}

inline Vector operator-(const Point &p1, const Point &p2) {
  return Vector(p1.x - p2.x, p1.y - p2.y);
}

inline Point operator+(const Point &p, const Vector &v) {
  return Point(p.x + v.x, p.y + v.y);
}

inline Vector operator-(const Vector &v1, const Vector &v2) {
  return Vector(v1.x - v2.x, v1.y - v2.y);
}

inline double operator^(const Vector &v1, const Vector &v2) {
  double a1 = atan2(v1.y, v1.x) - atan2(v2.y, v2.x);
  double a2 = (a1 > 0) ? a1 - 2 * M_PI : a1 + 2 * M_PI;
  return (std::abs(a1) < std::abs(a2)) ? a1 : a2;
}

inline Orientation operator%(const Vector &v1, const Vector &v2) {
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
  return VectorPtr(new Vector(*v));
}

inline VectorPtr operator++(VectorPtr v, int) {
  VectorPtr vector = VectorPtr(new Vector(*v));
  v->rotate_counterclockwise();
  return vector;
}

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_VECTOR_HPP_ */
