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

namespace common {

class Polygon {

public:
	Polygon(std::set<Point*>);
	~Polygon();
	std::set<Point*> upper_vertices(); // list of points
	std::set<Point*> lower_vertices(); // list of points
	__attribute__ ((will_be_removed)) std::map<Point*,
			std::set<Point*, PointComp>, PointComp> get_graph();

private:
	std::set<Point*> points;
	std::map<Point*, std::set<Point*, PointComp>, PointComp> graph;
	void build();
	Point* get_leftmost();
	Point* get_rightmost();
	std::set<Point*> get_vertices(bool); // list of points
};

}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_ */
