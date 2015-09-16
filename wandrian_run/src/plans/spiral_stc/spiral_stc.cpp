/*
 * spiral_stc.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/spiral_stc/spiral_stc.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {

SpiralStc::SpiralStc(Environment *environment, Point *starting_point,
		const double sub_cell_size) :
		environment(environment), sub_cell_size(sub_cell_size), step_size(
				sub_cell_size), current_direction_vector(new Vector2d(0, 1)) {
	// Initialize current_cell as 'starting cell'
	current_cell = new Cell(
			new Point(starting_point->x - sub_cell_size / 2,
					starting_point->y + sub_cell_size / 2), 2 * sub_cell_size);
	current_cell->set_parent(
			new Cell(
					new Point(current_cell->get_center()->x,
							current_cell->get_center()->y - 2 * sub_cell_size),
					2 * sub_cell_size));
	path.insert(starting_point);
}

void SpiralStc::cover() {
	spiral_stc(current_cell);
}

std::set<Point*> SpiralStc::get_path() {
	return path;
}

void SpiralStc::spiral_stc(Cell *current) {

}

}
}
}
