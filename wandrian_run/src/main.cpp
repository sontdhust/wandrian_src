/*
 * main.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: sontd
 */

#include <iostream>
#include <vector>
#include "../include/common/line.hpp"
#include "../include/common/polygon.hpp"

using namespace common::shapes;

int main(int argc, char **argv) {
	Point *p1 = new Point(0, 0);
	Point *p2 = new Point(2, 0);
	Point *p3 = new Point(0, 1);
	std::vector<Point*> ps;
	ps.push_back(p1);
	ps.push_back(p2);
	ps.push_back(p3);

	Polygon *po = new Polygon(ps);

	return 0;
}
