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
		spiral_stc->initialize(
				PointPtr(new Point(starting_point_x, starting_point_y)), robot_size);
		spiral_stc->set_behavior_go_with(
				boost::bind(&Wandrian::spiral_stc_go_with, this, _1, _2));
		spiral_stc->set_behavior_go_to(
				boost::bind(&Wandrian::spiral_stc_go_to, this, _1, _2));
		return spiral_stc->cover();
	}
}

bool Wandrian::go_to(PointPtr new_position, bool flexibly) {
	bool forward;
	forward = rotate(new_position, flexibly);
	move(forward);
	while (true) {
		// Check current_position + k * current_orientation == new_position
		Vector direction_vector = (*new_position - *current_position)
				/ (*new_position % *current_position);
		if (forward ?
				(!(std::abs(direction_vector.x - current_orientation->x)
						< EPS_ORI_TO_MOVE
						&& std::abs(direction_vector.y - current_orientation->y)
								< EPS_ORI_TO_MOVE)) :
				(!(std::abs(direction_vector.x + current_orientation->x)
						< EPS_ORI_TO_MOVE
						&& std::abs(direction_vector.y + current_orientation->y)
								< EPS_ORI_TO_MOVE))) {
			stop();
			forward = rotate(new_position, flexibly);
			move(forward);
		}

		if (!is_bumper_pressed) {
			if (std::abs(new_position->x - current_position->x) < EPS_POS
					&& std::abs(new_position->y - current_position->y) < EPS_POS) {
				stop();
				break;
			}
		} else {
			stop();
			return false;
		}
	}
	return true;
}

bool Wandrian::rotate(PointPtr new_position, bool flexibly) {
	VectorPtr new_orientation = VectorPtr(
			new Vector(
					(*new_position - *current_position)
							/ (*new_position % *current_position)));
	double a1 = atan2(new_orientation->y, new_orientation->x)
			- atan2(current_orientation->y, current_orientation->x);
	double a2 = (a1 > 0) ? a1 - 2 * M_PI : a1 + 2 * M_PI;
	double angle = (std::abs(a1) < std::abs(a2)) ? a1 : a2;
	std::cout << "      new_ori: " << new_orientation->x << ","
			<< new_orientation->y << " (" << angle << ")" << "\n";

	bool will_move_forward = !flexibly ? true : std::abs(angle) < M_PI_2;
	if (angle > EPS_ORI_TO_ROTATE)
		rotate(will_move_forward ? COUNTERCLOCKWISE : CLOCKWISE);
	else if (angle < -EPS_ORI_TO_ROTATE)
		rotate(will_move_forward ? CLOCKWISE : COUNTERCLOCKWISE);
	while (true) {
		if (will_move_forward ?
				(std::abs(new_orientation->x - current_orientation->x)
						< EPS_ORI_TO_ROTATE
						&& std::abs(new_orientation->y - current_orientation->y)
								< EPS_ORI_TO_ROTATE) :
				(std::abs(new_orientation->x + current_orientation->x)
						< EPS_ORI_TO_ROTATE
						&& std::abs(new_orientation->y + current_orientation->y)
								< EPS_ORI_TO_ROTATE)) {
			stop();
			break;
		}
	}
	return will_move_forward;
}

void Wandrian::rotate(bool clockwise) {
	if (clockwise)
		twist->angular.z = -angular_vel_step;
	else
		twist->angular.z = angular_vel_step;
}

void Wandrian::move(bool forward) {
	if (forward)
		twist->linear.x = linear_vel_step;
	else
		twist->linear.x = -linear_vel_step;
}

bool Wandrian::spiral_stc_go_to(PointPtr position, bool flexibly) {
	spiral_stc->path.insert(spiral_stc->path.end(), position);
	return go_to(position, flexibly);
}

bool Wandrian::spiral_stc_go_with(VectorPtr orientation, int step) {
	PointPtr last_position = *(--(spiral_stc->path.end()));
	PointPtr new_position = PointPtr(
			new Point(
					*last_position
							+ *orientation * step * spiral_stc->get_sub_cell_size() / 2));
	spiral_stc->path.insert(spiral_stc->path.end(), new_position);
	std::cout << "  p: " << new_position->x << "," << new_position->y << "; ("
			<< last_position->x << "," << last_position->y << "; " << orientation->x
			<< "," << orientation->y << "; " << step << ")\n";
	return go_to(new_position, STRICTLY);
}

}
