/*
 * vector2d.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: sontd
 */

#include "../../include/common/vector2d.hpp"

namespace wandrian {
namespace common {

Vector2d::Vector2d(double x, double y) :
		x(x), y(y) {
}

Vector2d::Vector2d(const Vector2d &vector) :
		x(vector.x), y(vector.y) {

}

Vector2d* Vector2d::rotate_counterclockwise() {
	return new Vector2d(-y, x);
}

}
}
