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
	Polygon();
	Polygon(std::set<Point*>);
	~Polygon();
	std::set<Point*> get_bound();
	/* __attribute__ ((will_be_removed)) */
	std::set<Point*> get_points();

protected:
	std::set<Point*> points;
	void build();

private:
	std::map<Point*, std::set<Point*, PointComp>, PointComp> graph;
	Point* get_leftmost();
	Point* get_rightmost();
	std::set<Point*> get_upper_bound(); // list of points
	std::set<Point*> get_lower_bound(); // list of points
	std::set<Point*> get_partial_bound(bool); // list of points
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_ */
