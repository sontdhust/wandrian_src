/*
 * polygon.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#include <iostream>
#include "../../include/common/polygon.hpp"

namespace common {
namespace shapes {

Polygon::Polygon(std::vector<Point*> points) :
		points(points) {
	build();
}

Polygon::~Polygon() {

}

void Polygon::add(Point* point) {
	points.push_back(point);
	build();
}

std::vector<Point*> Polygon::vertices() {
	std::vector<Point*> vertices;

	return vertices;
}

void Polygon::build() {
	for (std::vector<Point*>::iterator current = points.begin();
			current != points.end(); current++) {
		// Insert current point into graph if not yet
		if (graph.find(*current) == graph.end())
			graph.insert(
					std::pair<Point*, std::set<Point*> >(*current,
							std::set<Point*>()));
		// Find next point
		std::vector<Point*>::iterator next;
		if (current + 1 != points.end())
			next = current + 1;
		else
			next = points.begin();
		// Insert next point into graph if not yet
		if (graph.find(*next) == graph.end())
			graph.insert(
					std::pair<Point*, std::set<Point*> >(*next,
							std::set<Point*>()));
		graph.find(*current)->second.insert(*next);
		graph.find(*next)->second.insert(*current);

	}

	for (std::map<Point*, std::set<Point*> >::iterator gi = graph.begin();
			gi != graph.end(); gi++) {
		std::cout << gi->first->x << "-" << gi->first->y << ": ";
		for (std::set<Point*>::iterator pi = gi->second.begin();
				pi != gi->second.end(); pi++) {
			std::cout << (*pi)->x << "-" << (*pi)->y << " ";
		}
		std::cout << "\n";
	}
}

}
}

