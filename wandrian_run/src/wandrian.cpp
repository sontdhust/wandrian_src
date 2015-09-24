/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#include "../include/wandrian.hpp"
#include "../include/plans/spiral_stc/spiral_stc.hpp"

#define SUB_CELL_SIZE 100

using namespace wandrian::plans::spiral_stc;

namespace wandrian {

void Wandrian::run() {
	if (plan == "spiral_stc") {
		SpiralStc::initialize(current_position, SUB_CELL_SIZE);
		go_behavior = boost::bind(&Wandrian::go_spiral_stc, this, _1, _2);
		return SpiralStc::cover();
	}
}

bool Wandrian::go_spiral_stc(VectorPtr orientation, int step) {
	ROS_WARN("GO");

	PointPtr last_position = *(--path.end());
	return true;
}

}
