/*
 * main.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: sontd
 */

#include <iostream>
#include <set>
#include <boost/next_prior.hpp>
#include "../include/common/polygon.hpp"
#include "../include/common/segment.hpp"

using namespace common::shapes;

int main(int argc, char **argv) {
	/*
	 Segment *l1 = new Segment(0, 0, 2, 0);
	 Segment *l2 = new Segment(new Point(1, 1), new Point(1, -1));
	 Point *p = *l1 % *l2;
	 if (p != NULL)
	 std::cout << p->x << " " << p->y << "\n";
	 /*
	 std::set<int> numbers;
	 numbers.insert(1);
	 numbers.insert(5);
	 numbers.insert(2);
	 numbers.insert(4);
	 numbers.insert(2);
	 for (std::set<int>::iterator p = numbers.begin(); p != numbers.end(); p++) {
	 std::cout << (*p) << "\n";
	 }

	 std::set<Point*> ps;
	 ps.insert(new Point(0, 0));
	 ps.insert(new Point(2, 0));
	 ps.insert(new Point(0, 2));
	 ps.insert(new Point(-2, 4));
	 Polygon *p = new Polygon(ps);
	 std::set<Point*> s;
	 s.insert(new Point(1, 2));
	 s.insert(new Point(3, 4));
	 std::map<Point*, std::set<Point*, PointComp>, PointComp> m;

	 std::set<Point*>::iterator b = s.begin();
	 std::set<Point*>::iterator e = boost::next(b);
	 m.insert(
	 std::pair<Point*, std::set<Point*, PointComp> >(*b,
	 std::set<Point*, PointComp>()));
	 m.insert(
	 std::pair<Point*, std::set<Point*, PointComp> >(*e,
	 std::set<Point*, PointComp>()));
	 (*m.find(*b)).second.insert(*e);
	 (*m.find(*e)).second.insert(*b);

	 (*m.find(*b)).second.erase(*e);
	 for (std::set<Point*>::iterator c = s.begin(); c != s.end(); c++) {
	 std::cout << (*c)->x << " " << (*c)->y << "\n";
	 }
	 std::set<Point*, PointComp> points;
	 points.insert(new Point(1, 2));
	 points.insert(new Point(2, 5));
	 points.insert(new Point(3, 4));
	 points.insert(new Point(2, 4));
	 for (std::set<Point*>::iterator p = points.begin(); p != points.end();
	 p++) {
	 std::cout << (*p)->x << " " << (*p)->y << "\n";
	 }*/

	std::set<Point*> ps;
	ps.insert(new Point(0, 0));
	ps.insert(new Point(2, 0));
	ps.insert(new Point(0, 2));
	ps.insert(new Point(0, 4));
	Polygon *p = new Polygon(ps);
//	Polygon *p = new Polygon();
//	p->add(new Point(0, 0));
//	p->add(new Point(2, 0));
//	p->add(new Point(2, 2));
//	p->add(new Point(0, 2));
	std::map<Point*, std::set<Point*, PointComp>, PointComp> g = p->get_graph();
	for (std::map<Point*, std::set<Point*, PointComp> >::iterator current =
			g.begin(); current != g.end(); current++) {
		std::cout << (*current).first->x << " " << (*current).first->y << ": ";
		for (std::set<Point*>::iterator p = (*current).second.begin();
				p != (*current).second.end(); p++) {
			std::cout << (*p)->x << " " << (*p)->y << ", ";
		}
		std::cout << "\n";
	}

	return 0;
}
