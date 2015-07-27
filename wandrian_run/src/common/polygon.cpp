/*
 * polygon.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <boost/next_prior.hpp>
#include "../../include/common/segment.hpp"
#include "../../include/common/polygon.hpp"

namespace common {
namespace shapes {

Polygon::Polygon() :
		points(), graph() {
}

Polygon::Polygon(std::set<Point*> points) :
		points(points), graph() {
	build();
}

Polygon::~Polygon() {
	for (std::set<Point*>::iterator p = points.begin(); p != points.end();
			p++) {
		delete *p;
	}
	for (std::map<Point*, std::set<Point*, PointComp> >::iterator n =
			graph.begin(); n != graph.end(); n++) {
		delete (*n).first;
		for (std::set<Point*>::iterator p = (*n).second.begin();
				p != (*n).second.end(); p++) {
			delete *p;
		}
	}
}

void Polygon::add(Point* point) {
	// TODO Need improvement
	if (points.size() > 1) {
		std::set<Point*>::iterator begin = points.begin();
		std::set<Point*>::iterator end = points.end();
		end = boost::prior(end);
		(*graph.find(*begin)).second.erase(*end);
		(*graph.find(*end)).second.erase(*begin);
	}
	points.insert(point);
	build();
}

std::set<Point*> Polygon::upper_vertices() {
	return get_vertices(true);
}

std::set<Point*> Polygon::lower_vertices() {
	return get_vertices(false);
}

std::map<Point*, std::set<Point*, PointComp>, PointComp> Polygon::get_graph() {
	return graph;
}

void Polygon::build() {
	for (std::set<Point*>::iterator current = points.begin();
			current != points.end(); current++) {
		// Insert current point into graph if not yet
		if (graph.find(*current) == graph.end())
			graph.insert(
					std::pair<Point*, std::set<Point*, PointComp> >(*current,
							std::set<Point*, PointComp>()));
		// Find next point
		std::set<Point*>::iterator next;
		if (boost::next(current) != points.end())
			next = boost::next(current);
		else
			next = points.begin();
		// Insert next point into graph if not yet
		if (graph.find(*next) == graph.end())
			graph.insert(
					std::pair<Point*, std::set<Point*, PointComp> >(*next,
							std::set<Point*, PointComp>()));
		// Create edge
		if (current != next) {
			graph.find(*current)->second.insert(*next);
			graph.find(*next)->second.insert(*current);
		}
	}
	// Find all intersects
	std::map<Point*, std::set<Point*, PointComp>, PointComp> intersects;
	for (std::map<Point*, std::set<Point*, PointComp> >::iterator current =
			graph.begin(); current != graph.end(); current++) {
		for (std::set<Point*>::iterator current_adjacent =
				current->second.begin();
				current_adjacent != current->second.end(); current_adjacent++) {
			for (std::map<Point*, std::set<Point*, PointComp> >::iterator another =
					boost::next(current); another != graph.end(); another++) {
				for (std::set<Point*>::iterator another_adjacent =
						another->second.begin();
						another_adjacent != another->second.end();
						another_adjacent++) {
					Point *intersect = *(new Segment(current->first,
							*current_adjacent))
							% *(new Segment(another->first, *another_adjacent));
					if (intersect != NULL) {
						// Insert intersect into list intersects if this is new vertex
						std::set<Point*, PointComp> intersect_adjacents;
						if (*intersect != *(*current).first
								&& *intersect != **current_adjacent) {
							intersect_adjacents.insert((*current).first);
							intersect_adjacents.insert(*current_adjacent);
						}
						if (*intersect != *(*another).first
								&& *intersect != **another_adjacent) {
							intersect_adjacents.insert((*another).first);
							intersect_adjacents.insert(*another_adjacent);
						}
						intersects.insert(
								std::pair<Point*, std::set<Point*, PointComp> >(
										intersect, intersect_adjacents));
					}
				}
			}
		}
	}
	// Insert intersects and new edges into graph
	for (std::map<Point*, std::set<Point*, PointComp> >::iterator intersect =
			intersects.begin(); intersect != intersects.end(); intersect++) {
		if (graph.find((*intersect).first) == graph.end()) {
			graph.insert(
					std::pair<Point*, std::set<Point*, PointComp> >(
							(*intersect).first, std::set<Point*, PointComp>()));
		}

		for (std::set<Point*>::iterator adjacent = intersect->second.begin();
				adjacent != intersect->second.end(); adjacent++) {
			graph.find((*intersect).first)->second.insert(*adjacent);
			graph.find(*adjacent)->second.insert((*intersect).first);
		}
	}
	// TODO Remove redundant edges
}

Point* Polygon::get_leftmost() {
	Point* leftmost = *(points.begin());
	for (std::set<Point*>::iterator current = boost::next(points.begin());
			current != points.end(); current++) {
		if (**current < *leftmost)
			leftmost = *current;
	}
	return leftmost;
}

Point* Polygon::get_rightmost() {
	Point* rightmost = *(points.begin());
	for (std::set<Point*>::iterator current = boost::next(points.begin());
			current != points.end(); current++) {
		if (**current > *rightmost)
			rightmost = *current;
	}
	return rightmost;
}

std::set<Point*> Polygon::get_vertices(bool getUpper) {
	std::set<Point*> vertices;
	Point *leftmost = get_leftmost();
	Point *rightmost = get_rightmost();
	vertices.insert(leftmost);

	double EPS = std::numeric_limits<double>::epsilon();

	Point *current = leftmost;
	Point *previous = new Point(current->x - 1, current->y);
	while (*current != *rightmost) {
		double angle;
		double distance = std::numeric_limits<double>::infinity();
		if (getUpper)
			angle = 2 * M_PI;
		else
			angle = 0;
		Point *next;
		std::cout << "p: " << previous->x << " " << previous->y << ", c: "
				<< current->x << " " << current->y << "\n";
		for (std::set<Point*>::iterator adjacent =
				graph.find(current)->second.begin();
				adjacent != graph.find(current)->second.end(); adjacent++) {
			double a = atan2(previous->y - current->y, previous->x - current->x)
					- atan2((*adjacent)->y - current->y,
							(*adjacent)->x - current->x);
			double d = sqrt(
					pow(current->x - (*adjacent)->x, 2)
							+ pow(current->y - (*adjacent)->y, 2));
			if (getUpper) {
//				std::cout << "    " << a << "\n";
				a = std::abs(a) <= EPS ? 2 * M_PI : a > 0 ? a : 2 * M_PI + a;
				std::cout << "  (U)a: " << (*adjacent)->x << " "
						<< (*adjacent)->y << ", " << a << ", " << angle << "; "
						<< d << ", " << distance << "\n";
				if (a - angle < -EPS || (a == angle && d < distance)) {
					angle = a;
					distance = d;
					next = new Point(**adjacent);
				}
			} else {
//				std::cout << "    " << a << "\n";
				a = std::abs(a) <= EPS ? 0 : a > 0 ? a : 2 * M_PI + a;
				std::cout << "  (L)a: " << (*adjacent)->x << " "
						<< (*adjacent)->y << "; " << a << ", " << angle << "; "
						<< d << ", " << distance << "\n";
				if (a - angle > EPS || (a == angle && d < distance)) {
					angle = a;
					distance = d;
					next = new Point(**adjacent);
				}
			}
		}
		previous = new Point(*current);
		current = new Point(*next);
		vertices.insert(current);
//		getchar();
	}
	return vertices;
}

}
}

