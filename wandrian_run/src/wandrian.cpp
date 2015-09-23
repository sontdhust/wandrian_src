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

void Wandrian::cover() {
	if (plan == "spiral_stc") {
		SpiralStcPtr spiral_stc = SpiralStcPtr(
				new SpiralStc(EnvironmentPtr(), current_position, SUB_CELL_SIZE));
		spiral_stc->set_go_behavior(
				boost::bind(&Wandrian::go_spiral_stc, this, _1, _2));
		return spiral_stc->cover();
	}
}

bool Wandrian::go_spiral_stc(VectorPtr direction, int step) {
	ROS_WARN("GO");
	return true;
}

}
