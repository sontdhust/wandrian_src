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
	double get_sub_cell_size();
	void set_behavior_go_with(boost::function<bool(VectorPtr, int)>);
	void cover();

protected:
	bool go_to(PointPtr, bool);

private:
	/* __attribute__((will_be_removed)) */
	EnvironmentPtr environment;
	CellPtr starting_cell;
	double sub_cell_size; // = 'robot size' = 'cell size' / 2
	boost::function<bool(VectorPtr, int)> behavior_go_with;

	bool go_with(VectorPtr, int);
	void spiral_stc(CellPtr);
	bool check(CellPtr);
};

typedef boost::shared_ptr<SpiralStc> SpiralStcPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_ */
