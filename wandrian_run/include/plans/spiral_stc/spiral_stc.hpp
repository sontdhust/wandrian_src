/*
 * spiral_stc.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_

#include <boost/function.hpp>
#include "../../common/environment.hpp"
#include "../../common/vector.hpp"
#include "cell.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

class SpiralStc {

public:
	SpiralStc(EnvironmentPtr, PointPtr, const double);
	~SpiralStc();
	void cover();
	std::list<PointPtr> get_path();
	void set_go_behavior(boost::function<bool(VectorPtr, int)>);

private:
	EnvironmentPtr environment;
	const double sub_cell_size; // = 'robot size' = 'cell size' / 2
	CellPtr starting_cell;
	std::list<PointPtr> path;
	boost::function<bool(VectorPtr, int)> go_behavior;

	bool go(VectorPtr, int);
	void spiral_stc(CellPtr);
	bool check(CellPtr);
};

typedef boost::shared_ptr<SpiralStc> SpiralStcPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_SPIRAL_STC_HPP_ */
