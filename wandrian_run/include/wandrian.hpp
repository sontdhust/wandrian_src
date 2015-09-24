/*
 * wandrian.hpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_
#define WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_

#include "common/vector.hpp"
#include "core.hpp"
#include "plans/spiral_stc/spiral_stc.hpp"

using namespace wandrian::plans::spiral_stc;

namespace wandrian {

class Wandrian: public Core, public SpiralStc {

public:
	using Core::initialize;

protected:
	void run();

private:
	bool go_spiral_stc(VectorPtr, int);
};

}

#endif /* WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_ */
