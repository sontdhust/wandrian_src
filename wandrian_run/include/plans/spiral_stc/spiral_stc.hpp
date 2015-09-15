/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_

#include "../../common/environment.hpp"
#include "cell.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

class SpiralStc {
public:
	SpiralStc(Environment*, Cell*);
	void cover();
private:
	Environment *environment;
	Cell *starting_cell;
	void spiral_stc(Cell*, Cell*);
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_ */
