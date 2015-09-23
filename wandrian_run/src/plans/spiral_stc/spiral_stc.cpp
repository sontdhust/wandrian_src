/*
 * spiral_stc.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include <stdio.h>
#include <iostream>
#include "../../../include/plans/spiral_stc/spiral_stc.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {

SpiralStc::SpiralStc(EnvironmentPtr environment, PointPtr starting_point,
		const double sub_cell_size) :
		environment(environment), sub_cell_size(sub_cell_size), go_behavior(NULL) {
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

SpiralStc::~SpiralStc() {

}

void SpiralStc::cover() {
	spiral_stc(starting_cell);
}

std::list<PointPtr> SpiralStc::get_path() {
	return path;
}

void SpiralStc::set_go_behavior(
		boost::function<bool(VectorPtr, int)> go_behavior) {
	this->go_behavior = go_behavior;
}

bool SpiralStc::go(VectorPtr direction, int step) {
	if (go_behavior != NULL)
		return go_behavior(direction, step);

	PointPtr last_position = *(--path.end());
	PointPtr new_position = PointPtr(
			new Point(*last_position + *direction * step * sub_cell_size / 2));
	path.insert(path.end(), new_position);
	std::cout << "  p: " << new_position->x << "," << new_position->y << "; ("
			<< last_position->x << "," << last_position->y << "; " << direction->x
			<< "," << direction->y << "; " << step << ")\n";

// Bumper event here
// TODO: correctly check (now temporarily)
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
		if (new_position->x <= obstacle->get_center()->x + obstacle->get_size() / 2
				&& new_position->x
						>= obstacle->get_center()->x - obstacle->get_size() / 2
				&& new_position->y
						<= obstacle->get_center()->y + obstacle->get_size() / 2
				&& new_position->y
						>= obstacle->get_center()->y - obstacle->get_size() / 2) {
			return false;
		}
	}
	return true;
}

void SpiralStc::spiral_stc(CellPtr current) {
	std::cout << "current-BEGIN: " << current->get_center()->x << ","
			<< current->get_center()->y << "\n";
	// TODO: correctly compute starting direction
	VectorPtr direction = VectorPtr(
			new Vector(
					(*(current->get_parent()->get_center()) - *(current->get_center()))
							/ 2 / sub_cell_size));
	int neighbors_count = 0;
	// While current cell has a new obstacle-free neighboring cell
	bool is_starting_cell = *(current->get_center())
			== *(starting_cell->get_center());
	while (neighbors_count < (is_starting_cell ? 4 : 3)) {
		direction = direction->rotate_counterclockwise();
		// Scan for the first new neighbor of current cell in counterclockwise order
		CellPtr neighbor = CellPtr(
				new Cell(
						PointPtr(
								new Point(
										*(current->get_center()) + *direction * 2 * sub_cell_size)),
						2 * sub_cell_size));
		std::cout << "  neighbor: " << neighbor->get_center()->x << ","
				<< neighbor->get_center()->y;
		neighbors_count++;
		if (check(neighbor) == OLD_CELL) {
			std::cout << " (OLD)\n";
			// Go to next sub-cell
			go(direction->rotate_counterclockwise(), 2);
			continue;
		} else {
			std::cout << "\n";
		}
		if (!go(direction, 1)) { // Obstacle
			std::cout << "    (BUMP)\n";
			// Go back
			go(direction->rotate_counterclockwise()->rotate_counterclockwise(), 1);
			// Go to next sub-cell
			go(direction->rotate_counterclockwise(), 2);
		} else { // New free neighbor
			neighbor->set_parent(current);
			// Construct a spanning-tree edge
			current->neighbors.insert(current->neighbors.end(), neighbor);
			go(direction, 1);
			spiral_stc(neighbor);
		}
	}
	// Back to sub-cell of parent
	if (!is_starting_cell) {
		direction = direction->rotate_counterclockwise();
		go(direction, 2);
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
