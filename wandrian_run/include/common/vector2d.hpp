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

	Vector2d(double, double);

	double x, y;
};

inline Vector2d operator *(const double &k, const Vector2d &v) {
	return Vector2d(k * v.x, k * v.y);
}

inline Point operator +(const Point &p, const Vector2d &v) {
	return Point(p.x + v.x, p.y + v.y);
}

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_VECTOR2D_HPP_ */
