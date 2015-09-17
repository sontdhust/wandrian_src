/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_

#include "../../common/vector2d.hpp"
#include "../../common/environment.hpp"
#include "cell.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

class SpiralStc {

public:
	SpiralStc(Environment*, Point*, const double);
	void cover();
	std::set<Point*> get_path();

private:
	Environment *environment;
	const double sub_cell_size; // = 'robot size' = 'cell size' / 2
	const double step_size;
	bool is_bumper_pressing;
	Cell *starting_cell;
	std::set<Point*> path;

	void go(Vector2d*, int);
	void spiral_stc(Cell*);
	bool check(Cell*);
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_ */
