/*
 * vector2d.hpp
 *
 *  Created on: Sep 16, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_VECTOR2D_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_VECTOR2D_HPP_

#include "point.hpp"

namespace wandrian {
namespace common {

struct Vector2d {

	double x, y;

	Vector2d(double, double);
	Vector2d(const Vector2d&);
	boost::shared_ptr<Vector2d> rotate_counterclockwise();
};

typedef boost::shared_ptr<Vector2d> Vector2dPtr;

inline Vector2d operator *(const Vector2d &v, const double &k) {
	return Vector2d(v.x * k, v.y * k);
}

inline Vector2d operator /(const Vector2d &v, const double &k) {
	return Vector2d(v.x / k, v.y / k);
}

inline Vector2d operator -(const Point &a, const Point &b) {
	return Vector2d(a.x - b.x, a.y - b.y);
}

inline Point operator +(const Point &p, const Vector2d &v) {
	return Point(p.x + v.x, p.y + v.y);
}

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_VECTOR2D_HPP_ */
