/*
 * square.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#include "../../../include/plans/spiral_stc/cell.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {

Cell::Cell(Point *center, double size) :
		center(center), size(size), state(NEW_CELL), parent(NULL) {
	points.insert(new Point(center->x - size / 2, center->y + size / 2));
	points.insert(new Point(center->x + size / 2, center->y + size / 2));
	points.insert(new Point(center->x + size / 2, center->y - size / 2));
	points.insert(new Point(center->x - size / 2, center->y - size / 2));
	build();
}

Point* Cell::get_center() {
	return center;
}

double Cell::get_size() {
	return size;
}

void Cell::set_state(bool state) {
	this->state = state;
}

bool Cell::get_state() {
	return state;
}

void Cell::set_parent(Cell *parent) {
	this->parent = parent;
}

Cell* Cell::get_parent() {
	return parent;
}

}
}
}

