/*
 * point.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_

#include <limits>

namespace common {
namespace shapes {

struct Point {

	Point(double x, double y);
	Point(const Point &point);

	double x, y;

};

inline bool operator <(const Point &a, const Point &b) {
	if (a.x != b.x)
		return a.x - b.x < -std::numeric_limits<double>::epsilon();
	else
		return a.y - b.y < -std::numeric_limits<double>::epsilon();
}

inline bool operator !=(const Point &a, const Point &b) {
	return a < b || b < a;
}

inline bool operator >(const Point &a, const Point &b) {
	return a != b && !(a < b);
}

struct PointComp {
	bool operator()(const Point *a, const Point *b) const {
		return *a < *b;
	}
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_ */
