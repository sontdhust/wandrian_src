/*
 * polygon.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_

#include <map>
#include <set>
#include <list>
#include "point.hpp"

namespace wandrian {
namespace common {

class Polygon {

public:
	Polygon(std::list<PointPtr>);
	~Polygon();
	std::list<PointPtr> get_upper_bound(); // list of points
	std::list<PointPtr> get_lower_bound(); // list of points
	__attribute__ ((will_be_removed)) std::map<PointPtr,
			std::set<PointPtr, PointComp>, PointComp> get_graph();

private:
	std::list<PointPtr> points;
	std::map<PointPtr, std::set<PointPtr, PointComp>, PointComp> graph;
	void build();
	PointPtr get_leftmost();
	PointPtr get_rightmost();
	std::list<PointPtr> get_partial_bound(bool); // list of points
};

typedef boost::shared_ptr<Polygon> PolygonPtr;

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_ */
