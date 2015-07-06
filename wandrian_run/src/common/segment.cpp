/*
 * segment.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#include "../../include/common/segment.hpp"

namespace common {
namespace shapes {

Segment::Segment(Point* p1, Point* p2) {
	construct(*p1, *p2);
}

Segment::Segment(double x1, double y1, double x2, double y2) {
	Point *p1 = new Point(x1, y1);
	Point *p2 = new Point(x2, y2);
	construct(*p1, *p2);
}

Segment::~Segment() {
	delete p1;
	delete p2;
}

void Segment::construct(Point& p1, Point& p2) {
	this->p1 = &p1;
	this->p2 = &p2;
	if (p1.y > p2.y) {
		this->p2 = &p1;
		this->p1 = &p2;
	} else if (p1.y == p2.y) {
		if (p1.x > p2.x) {
			this->p2 = &p1;
			this->p1 = &p2;
		} else if (p1.x == p2.x) {
			assert(false);
		}
	}
}

}
}

