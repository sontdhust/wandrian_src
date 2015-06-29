/*
 * line.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_LINE_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_LINE_HPP_

#include <assert.h>
#include <stdexcept>
#include "point.hpp"

namespace common {
namespace shapes {

struct Line {

	Line(Point* p1, Point* p2);
	Line(double x1, double y1, double x2, double y2);
	~Line();

	Point *p1, *p2;

private:
	void construct(Point& p1, Point& p2);
};

inline Point operator %(const Line &a, const Line &b) {
	double dx;
	double dy;

	dx = a.p2->x - a.p1->x;
	dy = a.p2->y - a.p1->y;
	double am = dy / dx;
	double ac = a.p1->y - am * a.p1->x;

	dx = b.p2->x - b.p1->x;
	dy = b.p2->y - b.p1->y;
	double bm = dy / dx;
	double bc = b.p1->y - bm * b.p1->x;

	if ((am - bm) == 0)
		throw std::invalid_argument("No (or too much) intersection(s) between the lines");
	else {
		double xi = (bc - ac) / (am - bm);
		double yi = am * xi + ac;
		return Point(xi, yi);
	}
}

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_LINE_HPP_ */
