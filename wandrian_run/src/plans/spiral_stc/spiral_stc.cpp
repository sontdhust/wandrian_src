/*
 * spiral_stc.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

//#include <stdio.h>
//#include <iostream>
#include "../../../include/plans/spiral_stc/spiral_stc.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {

SpiralStc::SpiralStc() :
		sub_cell_size(0) {
}

SpiralStc::~SpiralStc() {
}

void SpiralStc::initialize(PointPtr starting_point, double sub_cell_size) {
	this->sub_cell_size = sub_cell_size;
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

double SpiralStc::get_sub_cell_size() {
	return sub_cell_size;
}

void SpiralStc::set_behavior_go_with(
		boost::function<bool(VectorPtr, int)> behavior_go_with) {
	this->behavior_go_with = behavior_go_with;
}

void SpiralStc::cover() {
	spiral_stc(starting_cell);
}

bool SpiralStc::go_to(PointPtr position) {
	// Override this method
	if (behavior_go_to != NULL)
		return behavior_go_to(position);

	path.insert(path.end(), position);
	// TODO: Need to check bumper here?
	return true;
}

bool SpiralStc::go_with(VectorPtr orientation, int step) {
	if (behavior_go_with != NULL)
		return behavior_go_with(orientation, step);

	PointPtr last_position = *(--path.end());
	PointPtr new_position = PointPtr(
			new Point(*last_position + *orientation * step * sub_cell_size / 2));
	path.insert(path.end(), new_position);
	std::cout << "  p: " << new_position->x << "," << new_position->y << "; ("
			<< last_position->x << "," << last_position->y << "; " << orientation->x
			<< "," << orientation->y << "; " << step << ")\n";

	// Bumper event here
	// TODO: Correctly check (now temporarily)
	if (environment) {
		CellPtr space = boost::static_pointer_cast<Cell>(environment->space);
		if (new_position->x >= space->get_center()->x + space->get_size() / 2
				|| new_position->x <= space->get_center()->x - space->get_size() / 2
				|| new_position->y >= space->get_center()->y + space->get_size() / 2
				|| new_position->y <= space->get_center()->y - space->get_size() / 2) {
			return false;
		}
		for (std::list<PolygonPtr>::iterator o = environment->obstacles.begin();
				o != environment->obstacles.end(); o++) {
			CellPtr obstacle = boost::static_pointer_cast<Cell>(*o);
			if (new_position->x
					<= obstacle->get_center()->x + obstacle->get_size() / 2
					&& new_position->x
							>= obstacle->get_center()->x - obstacle->get_size() / 2
					&& new_position->y
							<= obstacle->get_center()->y + obstacle->get_size() / 2
					&& new_position->y
							>= obstacle->get_center()->y - obstacle->get_size() / 2) {
				return false;
			}
		}
	}
	return true;
}

void SpiralStc::spiral_stc(CellPtr current) {
	std::cout << "current-BEGIN: " << current->get_center()->x << ","
			<< current->get_center()->y << "\n";
	// TODO: Correctly compute starting orientation
	VectorPtr orientation = VectorPtr(
			new Vector(
					(*(current->get_parent()->get_center()) - *(current->get_center()))
							/ 2 / sub_cell_size));
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
										*(current->get_center())
												+ *orientation * 2 * sub_cell_size)),
						2 * sub_cell_size));
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
		if (!go_with(orientation, 1)) { // Obstacle
			std::cout << "    (BUMP)\n";
			// Go back
			go_to(*(----path.end()));
			// Go to next sub-cell
			go_with(orientation->rotate_counterclockwise(), 2);
		} else { // New free neighbor
			neighbor->set_parent(current);
			// Construct a spanning-tree edge
			current->neighbors.insert(current->neighbors.end(), neighbor);
			go_with(orientation, 1);
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
