/*
 * segment.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_SEGMENT_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_SEGMENT_HPP_

#include "point.hpp"

namespace common {
namespace shapes {

struct Segment {

	Segment(const Point* p1, const Point* p2);
	Segment(double x1, double y1, double x2, double y2);
	~Segment();

	Point *p1, *p2;

private:
	void construct(const Point& p1, const Point& p2);
};

/**
 * Find intersect between 2 segments
 */
inline Point* operator %(const Segment &a, const Segment &b) {
	double dx;
	double dy;

	dx = a.p2->x - a.p1->x;
	dy = a.p2->y - a.p1->y;
	double am = dy / dx;
	double ac = a.p1->y - am * a.p1->x;
	// Equation of line a: y = am * x + ac

	dx = b.p2->x - b.p1->x;
	dy = b.p2->y - b.p1->y;
	double bm = dy / dx;
	double bc = b.p1->y - bm * b.p1->x;
	// Equation of line b: y = bm * x + bc

	double EPS = std::numeric_limits<double>::epsilon();
	double INF = std::numeric_limits<double>::infinity();

	if ((am == INF && bm == INF) || std::abs(am - bm) < EPS) // Line a is parallel to line b
		return NULL;
	else {
		double xi;
		double yi;
		if (am == INF) { // Line a is parallel to vertical axis, but not b
			xi = a.p1->x; // = a.p2->x
			yi = bm * xi + bc;
		} else if (bm == INF) { // Line b is parallel to vertical axis, but not a
			xi = b.p1->x; // = b.p2->x
			yi = am * xi + ac;
		} else { // Both are not parallel to vertical axis
			xi = (bc - ac) / (am - bm);
			yi = am * xi + ac; // = bm * xi + bc
		}
		if (a.p1->x <= xi && xi <= a.p2->x && b.p1->x <= xi && xi <= b.p2->x
				&& ((a.p1->y <= yi && yi <= a.p2->y) || (a.p2->y <= yi && yi <= a.p1->y))
				&& ((b.p1->y <= yi && yi <= b.p2->y) || (b.p2->y <= yi && yi <= b.p1->y)))
			return new Point(xi, yi);
		else
			return NULL;
	}
}

struct SegmentComp {
	bool operator()(const Segment *a, const Segment *b) const {
		return *(a->p1) != *(b->p1) ? *(a->p1) < *(b->p1) : *(a->p2) < *(b->p2);
	}
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_SEGMENT_HPP_ */
