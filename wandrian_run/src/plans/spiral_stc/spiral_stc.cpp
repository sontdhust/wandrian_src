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

SpiralStc::SpiralStc() :
		robot_size(0) {
}

SpiralStc::~SpiralStc() {
}

void SpiralStc::initialize(PointPtr starting_point, double sub_cell_size) {
	this->robot_size = sub_cell_size;
	// Initialize starting_cell
	starting_cell = CellPtr(
			new Cell(
					PointPtr(
							new Point(starting_point->x - sub_cell_size / 2,
									starting_point->y + sub_cell_size / 2)), 2 * sub_cell_size));
	starting_cell->set_parent(
			CellPtr(
					new Cell(
							PointPtr(
									new Point(starting_cell->get_center()->x,
											starting_cell->get_center()->y - 2 * sub_cell_size)),
							2 * sub_cell_size)));
	path.insert(path.end(), starting_point);
}

void SpiralStc::set_environment(EnvironmentPtr environment) {
	this->environment = environment;
}

void SpiralStc::set_behavior_see_obstacle(
		boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
	this->behavior_see_obstacle = behavior_see_obstacle;
}

void SpiralStc::cover() {
	spiral_stc(starting_cell);
}

bool SpiralStc::go_to(PointPtr position, bool flexibly) {
	if (behavior_go_to != NULL)
		return behavior_go_to(position, flexibly);

	path.insert(path.end(), position);
	std::cout << "    pos: " << position->x << "," << position->y << "\n";
	return true;
}

bool SpiralStc::see_obstacle(VectorPtr orientation, double step) {
	if (behavior_see_obstacle != NULL)
		return behavior_see_obstacle(orientation, step);

	// Simulator check obstacle here
	PointPtr last_position = *(--path.end());
	PointPtr new_position = PointPtr(
			new Point(*last_position + *orientation * step * robot_size / 2));
	if (environment) {
		CellPtr space = boost::static_pointer_cast<Cell>(environment->space);
		if (new_position->x >= space->get_center()->x + space->get_size() / 2
				|| new_position->x <= space->get_center()->x - space->get_size() / 2
				|| new_position->y >= space->get_center()->y + space->get_size() / 2
				|| new_position->y <= space->get_center()->y - space->get_size() / 2) {
			return true;
		}
		for (std::list<PolygonPtr>::iterator o = environment->obstacles.begin();
				o != environment->obstacles.end(); o++) {
			CellPtr obstacle = boost::static_pointer_cast<Cell>(*o);
			if (new_position->x
					>= obstacle->get_center()->x - obstacle->get_size() / 2
					&& new_position->x
							<= obstacle->get_center()->x + obstacle->get_size() / 2
					&& new_position->y
							>= obstacle->get_center()->y - obstacle->get_size() / 2
					&& new_position->y
							<= obstacle->get_center()->y + obstacle->get_size() / 2) {
				return true;
			}
		}
	}
	return false;
}

bool SpiralStc::go_with(VectorPtr orientation, double step) {
	PointPtr last_position = *(--path.end());
	PointPtr new_position = PointPtr(
			new Point(*last_position + *orientation * step * robot_size / 2));
	return go_to(new_position, STRICTLY);
}

void SpiralStc::spiral_stc(CellPtr current) {
	std::cout << "current-BEGIN: " << current->get_center()->x << ","
			<< current->get_center()->y << "\n";
	VectorPtr orientation = VectorPtr(
			new Vector(
					(*(current->get_parent()->get_center()) - *(current->get_center()))
							/ 2 / robot_size));
	int neighbors_count = 0;
	// While current cell has a new obstacle-free neighboring cell
	bool is_starting_cell = *(current->get_center())
			== *(starting_cell->get_center());
	while (neighbors_count < (is_starting_cell ? 4 : 3)) {
		orientation = orientation->rotate_counterclockwise();
		// Scan for the first new neighbor of current cell in counterclockwise order
		CellPtr neighbor = CellPtr(
				new Cell(
						PointPtr(
								new Point(
										*(current->get_center()) + *orientation * 2 * robot_size)),
						2 * robot_size));
		std::cout << "  neighbor: " << neighbor->get_center()->x << ","
				<< neighbor->get_center()->y;
		neighbors_count++;
		if (check(neighbor) == OLD_CELL) {
			std::cout << " (OLD)\n";
			// Go to next sub-cell
			go_with(orientation->rotate_counterclockwise(), 2);
			continue;
		} else {
			std::cout << "\n";
		}
		if (see_obstacle(orientation, 1)) { // TODO: Check obstacle here
			std::cout << "    (OBSTACLE)\n";
			// Go to next sub-cell
			go_with(orientation->rotate_counterclockwise(), 2);
		} else { // New free neighbor
			neighbor->set_parent(current);
			// Construct a spanning-tree edge
			current->neighbors.insert(current->neighbors.end(), neighbor);
			go_with(orientation, 2);
			spiral_stc(neighbor);
		}
	}
	// Back to sub-cell of parent
	if (!is_starting_cell) {
		orientation = orientation->rotate_counterclockwise();
		go_with(orientation, 2);
	}
	std::cout << "current-END: " << current->get_center()->x << ","
			<< current->get_center()->y << "\n";
}

bool SpiralStc::check(CellPtr cell_to_check) {
	std::list<CellPtr> list;
	list.insert(list.end(), starting_cell);
	while (list.size() > 0) {
		CellPtr cell = *(list.begin());
		if (*(cell->get_center()) == *(cell_to_check->get_center()))
			return OLD_CELL;
		list.erase(list.begin());
		for (std::list<CellPtr>::iterator c = cell->neighbors.begin();
				c != cell->neighbors.end(); c++)
			list.insert(list.end(), *c);
	}
	return NEW_CELL;
}

}
}
}
