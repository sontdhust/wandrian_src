/*
 * square.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_

#include "../../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

#define OLD_CELL false
#define NEW_CELL true

class Cell: public Polygon {

public:
	Cell(Point*, double);
	Point* get_center();
	void set_state(bool);
	bool get_state();
	void set_parent(Cell*);
	Cell* get_parent();

private:
	Point *center;
	bool state;
	Cell *parent;
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_ */
