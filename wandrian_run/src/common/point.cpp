/*
 * point.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#include "../../include/common/point.hpp"

namespace common {

Point::Point(double x, double y) :
		x(x), y(y) {
}

Point::Point(const Point &point) :
		x(point.x), y(point.y) {
}

}
