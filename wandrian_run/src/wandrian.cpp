/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#include "../include/wandrian.hpp"
#include "../include/plans/spiral_stc/spiral_stc.hpp"

#define CLOCKWISE true
#define COUNTERCLOCKWISE false

// TODO: Choose relevant epsilon values
#define EPS_ORI_TO_ROTATE 0.06
#define EPS_ORI_TO_MOVE 4 * EPS_ORI_TO_ROTATE
#define EPS_POS 0.04

namespace wandrian {

void Wandrian::run() {
	if (plan == "spiral_stc") {
		spiral_stc = SpiralStcPtr(new SpiralStc());
		spiral_stc->initialize(current_position, robot_size);
		spiral_stc->set_go_behavior(
				boost::bind(&Wandrian::go_spiral_stc, this, _1, _2));
		return spiral_stc->cover();
	}
}

bool Wandrian::move(PointPtr new_position) {
	rotate(new_position);
	move_ahead();
	while (true) {
		// Check current_position + k * current_orientation == new_position
		Vector direction_vector = (*new_position - *current_position)
				/ (*new_position % *current_position);
		if (!(std::abs(direction_vector.x - current_orientation->x)
				< EPS_ORI_TO_MOVE
				&& std::abs(direction_vector.y - current_orientation->y)
						< EPS_ORI_TO_MOVE)) {
			stop();
			rotate(new_position);
			move_ahead();
		}

		if (!is_bumper_pressed) {
			if (std::abs(new_position->x - current_position->x) < EPS_POS
					&& std::abs(new_position->y - current_position->y) < EPS_POS) {
				stop();
				break;
			}
		} else
			return false;
	}
	return true;
}

void Wandrian::move_ahead() {
	twist->linear.x = linear_vel_step;
}

void Wandrian::rotate(PointPtr new_position) {
	VectorPtr new_orientation = VectorPtr(
			new Vector(
					(*new_position - *current_position)
							/ (*new_position % *current_position)));
	std::cout << "      new_ori: " << new_orientation->x << ","
			<< new_orientation->y << " (" << (*new_orientation ^ *current_orientation)
			<< ")" << "\n";
	if ((*new_orientation ^ *current_orientation) > EPS_ORI_TO_ROTATE)
		rotate(COUNTERCLOCKWISE);
	else if ((*new_orientation ^ *current_orientation) < -EPS_ORI_TO_ROTATE)
		rotate(CLOCKWISE);
	while (true) {
		if (std::abs(
				new_orientation->x - current_orientation->x) < EPS_ORI_TO_ROTATE
				&& std::abs(new_orientation->y - current_orientation->y)
				< EPS_ORI_TO_ROTATE) {
			stop();
			break;
		}
	}
}

void Wandrian::rotate(bool clockwise) {
	if (clockwise)
		twist->angular.z = -angular_vel_step;
	else
		twist->angular.z = angular_vel_step;
}

bool Wandrian::go_spiral_stc(VectorPtr orientation, int step) {
	PointPtr last_position = *(--(spiral_stc->path.end()));
	std::cout << "    pos: " << last_position->x << "," << last_position->y
			<< "; ori: " << orientation->x << "," << orientation->y << "\n";
	PointPtr new_position = PointPtr(
			new Point(
					*last_position
							+ *orientation * step * spiral_stc->get_sub_cell_size() / 2));
	spiral_stc->path.insert(spiral_stc->path.end(), new_position);
	return move(new_position);
}

}
