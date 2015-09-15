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
#include "point.hpp"

namespace wandrian {
namespace common {

class Polygon {

public:
	Polygon(std::set<PointPtr>);
	~Polygon();
	std::set<PointPtr> upper_vertices(); // list of points
	std::set<PointPtr> lower_vertices(); // list of points
	__attribute__ ((will_be_removed)) std::map<PointPtr,
			std::set<PointPtr, PointComp>, PointComp> get_graph();

private:
	std::set<PointPtr> points;
	std::map<PointPtr, std::set<PointPtr, PointComp>, PointComp> graph;
	void build();
	PointPtr get_leftmost();
	PointPtr get_rightmost();
	std::set<PointPtr> get_vertices(bool); // list of points
};

typedef boost::shared_ptr<Polygon> PolygonPtr;

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_ */
