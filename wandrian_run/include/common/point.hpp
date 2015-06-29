/*
 * point.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_

namespace common {
namespace shapes {

struct Point {

	Point(double x, double y);

	double x, y;
};

inline bool operator <(const Point &a, const Point &b) {
	if (a.y != b.y)
		return a.y < b.y;
	else
		return a.x < b.x;
}

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_ */
