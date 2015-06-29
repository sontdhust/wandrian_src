/*
 * polygon.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_

#include <vector>
#include <map>
#include <set>
#include "point.hpp"

namespace common {
namespace shapes {

class Polygon {

public:
	Polygon(std::vector<Point*> points);
	~Polygon();
	void add(Point* point);
	std::vector<Point*> vertices(); // list of points

private:
	std::vector<Point*> points;
	std::map<Point*, std::set<Point*> > graph;
	void build();
};
}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POLYGON_HPP_ */
