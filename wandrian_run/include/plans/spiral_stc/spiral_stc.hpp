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
	void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);
	void cover();

protected:
	bool go_to(PointPtr, bool);

private:
	/* __attribute__((will_be_removed)) */
	EnvironmentPtr environment;
	CellPtr starting_cell;
	double robot_size; // = 'cell size' / 2
	boost::function<bool(VectorPtr, double)> behavior_see_obstacle;

	bool see_obstacle(VectorPtr, double);
	bool go_with(VectorPtr, double);
	void spiral_stc(CellPtr);
	bool check(CellPtr);
};

typedef boost::shared_ptr<SpiralStc> SpiralStcPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_ */
