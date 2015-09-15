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

SpiralStc::SpiralStc(Environment *environment, Cell *starting_cell) :
		environment(environment), starting_cell(starting_cell) {
}

void SpiralStc::cover() {
	spiral_stc(NULL, starting_cell);
}

void SpiralStc::spiral_stc(Cell *parent, Cell *current) {

}

}
}
}
