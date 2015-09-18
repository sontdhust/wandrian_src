/*
 * point.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_

#include <stdlib.h>
#include <cmath>
#include <limits>
#include <boost/shared_ptr.hpp>

namespace wandrian {
namespace common {

struct Point {

	double x, y;

	Point(double, double);
	Point(const Point&);

};

typedef boost::shared_ptr<Point const> PointConstPtr;
typedef boost::shared_ptr<Point> PointPtr;

inline double operator %(const Point &a, const Point &b) {
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

inline bool operator <(const Point &a, const Point &b) {
	// TODO Choose relevant epsilon value
	double EPS = 20 * std::numeric_limits<double>::epsilon();
	return std::abs(a.x - b.x) > EPS ? a.x - b.x < -EPS : a.y - b.y < -EPS;
}

inline bool operator !=(const Point &a, const Point &b) {
	return a < b || b < a;
}

inline bool operator ==(const Point &a, const Point &b) {
	return !(a != b);
}

inline bool operator >(const Point &a, const Point &b) {
	return a != b && !(a < b);
}

struct PointComp {
	bool operator()(PointConstPtr a, PointConstPtr b) const {
		return *a < *b;
	}
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_ */
