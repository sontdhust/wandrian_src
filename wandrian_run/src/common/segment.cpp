/*
 * segment.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#include <stdexcept>
#include "../../include/common/segment.hpp"

namespace wandrian {
namespace common {

Segment::Segment(PointConstPtr p1, PointConstPtr p2) {
	construct(*p1, *p2);
}

Segment::Segment(double x1, double y1, double x2, double y2) {
	PointPtr p1 = boost::shared_ptr<Point>(new Point(x1, y1));
	PointPtr p2 = boost::shared_ptr<Point>(new Point(x2, y2));
	construct(*p1, *p2);
}

Segment::~Segment() {
}

void Segment::construct(const Point& p1, const Point& p2) {
	if (p1 == p2)
		throw std::invalid_argument("Two end points of segment are coincident");
	else if (p1 < p2) {
		this->p1 = boost::shared_ptr<Point>(new Point(p1));
		this->p2 = boost::shared_ptr<Point>(new Point(p2));
	} else {
		this->p1 = boost::shared_ptr<Point>(new Point(p2));
		this->p2 = boost::shared_ptr<Point>(new Point(p1));
	}
}

}
}
