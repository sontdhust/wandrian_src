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

class Wandrian: public Core {

protected:
	void run();

private:
	SpiralStcPtr spiral_stc;

	// Helpers
	bool go_to(PointPtr, bool);
	bool rotate(PointPtr, bool);
	void rotate(bool);
	void move(bool);

	// Behaviors
	bool spiral_stc_go_to(PointPtr, bool);
	bool spiral_stc_go_with(VectorPtr, int);
};

}

#endif /* WANDRIAN_RUN_INCLUDE_WANDRIAN_HPP_ */
