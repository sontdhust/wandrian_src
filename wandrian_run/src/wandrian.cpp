/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#include "../include/wandrian.hpp"
#include "../include/plans/spiral_stc/spiral_stc.hpp"

#define SUB_CELL_SIZE 0.5
#define CLOCKWISE false
#define COUNTERCLOCKWISE true

namespace wandrian {

void Wandrian::run() {
	if (plan == "spiral_stc") {
		spiral_stc = SpiralStcPtr(new SpiralStc());
		spiral_stc->initialize(current_position, SUB_CELL_SIZE);
		spiral_stc->set_go_behavior(
				boost::bind(&Wandrian::go_spiral_stc, this, _1, _2));
		return spiral_stc->cover();
	}
}

bool Wandrian::move(PointPtr new_position) {
	VectorPtr new_orientation = VectorPtr(
			new Vector(
					(*new_position - *current_position)
							/ (*new_position % *current_position)));
	std::cout << "    new_ori: " << new_orientation->x << ","
			<< new_orientation->y << "\n";
	// TODO: Choose relevant epsilon orientation value
	const double EPS_ORI = 0.05;
	const double EPS_POS = 0.01;
	if (*current_orientation % *new_orientation < 0)
		rotate(CLOCKWISE);
	else
		rotate(COUNTERCLOCKWISE);
	while (true) {
		if (std::abs(new_orientation->x - current_orientation->x) < EPS_ORI
				&& std::abs(new_orientation->y - current_orientation->y) < EPS_ORI) {
			reset_vel();
			break;
		}
	}
	move_ahead();
	while (true) {
		if (!is_bumper_pressed) {
			if (std::abs(new_position->x - current_position->x) < EPS_POS
					&& std::abs(new_position->y - current_position->y) < EPS_POS) {
				reset_vel();
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

void Wandrian::rotate(bool clockwise) {
	if (clockwise)
		twist->angular.z = -2 * angular_vel_step;
	else
		twist->angular.z = 2 * angular_vel_step;
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
