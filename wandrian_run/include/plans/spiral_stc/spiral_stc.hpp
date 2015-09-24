/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_

#include "../../common/environment.hpp"
#include "../base_plan.hpp"
#include "cell.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

class SpiralStc: public BasePlan {

public:
	SpiralStc();
	~SpiralStc();
	void initialize(PointPtr, double);
	/* __attribute__((will_be_removed)) */
	void set_environment(EnvironmentPtr);
	void cover();

protected:
	double sub_cell_size; // = 'robot size' = 'cell size' / 2

private:
	EnvironmentPtr environment;
	CellPtr starting_cell;

	bool go(VectorPtr, int);
	void spiral_stc(CellPtr);
	bool check(CellPtr);
};

typedef boost::shared_ptr<SpiralStc> SpiralStcPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_ */
