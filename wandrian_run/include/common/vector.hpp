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

struct Vector {

	double x, y;

	Vector(double, double);
	Vector(const Vector&);
	boost::shared_ptr<Vector> rotate_counterclockwise();
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

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_VECTOR_HPP_ */
