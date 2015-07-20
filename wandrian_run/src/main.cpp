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
	 }
	 Point *p1 = new Point(1, 2);
	 Point *p2 = new Point(3, 4);
	 bool b = false;
	 if (*p1 < *p2)
	 b = true;
	 if (b)
	 std::cout << "true\n";*/
//	std::set<Point*> ps;
//	ps.insert(new Point(0, 0));
//	ps.insert(new Point(2, 0));
//	ps.insert(new Point(0, 2));
//	ps.insert(new Point(0, 4));
//	Polygon *p = new Polygon(ps);
//	Point *p1 = new Point(1, 2);
//	Point *p2;
//	*p2 = *p1;
//	p1->x = 3;
//	bool b = false;
//	if(*p1 != *p2) b = true;
//	if(b) std::cout << p2->x << "\n";
	Polygon *polygon = new Polygon();
	polygon->add(new Point(0, 0));
	polygon->add(new Point(1, 2));
	polygon->add(new Point(3, 2));
	polygon->add(new Point(4, 0));
	polygon->add(new Point(1, -2));
	polygon->add(new Point(0, -1));
	std::map<Point*, std::set<Point*, PointComp>, PointComp> g = polygon->get_graph();
	for (std::map<Point*, std::set<Point*, PointComp> >::iterator current =
			g.begin(); current != g.end(); current++) {
		std::cout << (*current).first->x << " " << (*current).first->y << ": ";
		for (std::set<Point*>::iterator p = (*current).second.begin();
				p != (*current).second.end(); p++) {
			std::cout << (*p)->x << " " << (*p)->y << ", ";
		}
		std::cout << "\n";
	}

//	Point *p1 = new Point(2, 2);
//	for (std::set<Point*>::iterator p = g.find(p1)->second.begin();
//			p != g.find(p1)->second.end(); p++) {
//		std::cout << (*p)->x << " " << (*p)->y << ", ";
//	}
//	std::cout << "\n";
//	Point *p2 = new Point;
//	std::cout << (**(g.find(p1)->second.begin())).y << "\n";
//	*p2 = **(g.find(p1)->second.begin());

	std::set<Point*> u = polygon->upper_vertices();
	for (std::set<Point*>::iterator p = u.begin(); p != u.end(); p++) {
		std::cout << (*p)->x << " " << (*p)->y << "; ";
	}
	std::cout << "\n";

	std::set<Point*> l = polygon->lower_vertices();
	for (std::set<Point*>::iterator p = l.begin(); p != l.end(); p++) {
		std::cout << (*p)->x << " " << (*p)->y << "; ";
	}
	std::cout << "\n";

	return 0;
}
