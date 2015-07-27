/*
 * main.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: sontd
 */

#include <iostream>
#include <limits>
#include <GL/glut.h>
#include <set>
#include <ctime>
#include <cstdlib>
#include <boost/next_prior.hpp>
#include "../include/common/polygon.hpp"
#include "../include/common/segment.hpp"

using namespace common::shapes;

Polygon *polygon;

/**
 * Linked libraries to compile: -lglut -lGL (g++)
 */

void display() {
	glClear( GL_COLOR_BUFFER_BIT);

	glMatrixMode( GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-6, 6, -6, 6, -1, 1);

	glMatrixMode( GL_MODELVIEW);
	glLoadIdentity();

	glScalef(0.5, 0.5, 0);

	// TODO draw primitives here

	//std::cout.precision(0);

	glPointSize(4);
	glColor3ub(255, 255, 0);
	glBegin(GL_POINTS);
	glVertex2i(0, 0);
	glEnd();

	glPointSize(1);
	glColor3ub(255, 255, 255);
	glBegin(GL_POINTS);
	for (int i = -10; i <= 10; i++) {
		for (int j = -10; j <= 10; j++) {
			if (i != 0 || j != 0)
				glVertex2i(i, j);
		}
	}
	glEnd();

	std::map<Point*, std::set<Point*, PointComp>, PointComp> graph;
	std::set<Point*> upper_vertices;
	std::set<Point*> lower_vertices;

	graph = polygon->get_graph();
	glColor3ub(255, 255, 255);
	for (std::map<Point*, std::set<Point*, PointComp> >::iterator current =
			graph.begin(); current != graph.end(); current++) {
		std::cout << (*current).first->x << " " << (*current).first->y << ": ";
		for (std::set<Point*>::iterator adjacent = (*current).second.begin();
				adjacent != (*current).second.end(); adjacent++) {
			glBegin(GL_LINE_STRIP);
			glVertex2d((*current).first->x, (*current).first->y);
			glVertex2d((*adjacent)->x, (*adjacent)->y);
			glEnd();
			std::cout << (*adjacent)->x << " " << (*adjacent)->y << ", ";
		}
		std::cout << "\n\n";
	}
	std::cout << "\n";

	upper_vertices = polygon->upper_vertices();
	glColor3ub(255, 0, 0);
	for (std::set<Point*>::iterator current = upper_vertices.begin();
			boost::next(current) != upper_vertices.end(); current++) {
		glBegin(GL_LINE_STRIP);
		glVertex2d((*current)->x, (*current)->y);
		glVertex2d((*boost::next(current))->x, (*boost::next(current))->y);
		glEnd();
	}

	lower_vertices = polygon->lower_vertices();
	glColor3ub(0, 255, 0);
	for (std::set<Point*>::iterator current = lower_vertices.begin();
			boost::next(current) != lower_vertices.end(); current++) {
		glBegin(GL_LINE_STRIP);
		glVertex2d((*current)->x, (*current)->y);
		glVertex2d((*boost::next(current))->x, (*boost::next(current))->y);
		glEnd();
	}

	glutSwapBuffers();
}

int run(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(600, 600);
	glutCreateWindow("GLUT");
	glutDisplayFunc(display);
	glutMainLoop();
	return 0;
}

int main(int argc, char **argv) {
	std::set<Point*> points;
	points.insert(new Point(0, 1));
	points.insert(new Point(2, 3));
	points.insert(new Point(4, 2));
	points.insert(new Point(5, -1));
	points.insert(new Point(3, -2));
	points.insert(new Point(-1, 4));
	points.insert(new Point(4, -2));
	points.insert(new Point(0, -3));
	points.insert(new Point(3, 2));
	points.insert(new Point(-4, -5));
	points.insert(new Point(-3, 3));
//	points.insert(new Point(3, 0));
//	std::set<Point*> points;
//	std::srand(std::time(0));
//	int r = std::rand() % 20;
//	for (int i = 0; i <= r; i++) {
//		int x = std::rand() % 16 - 8;
//		int y = std::rand() % 16 - 8;
//		points.insert(new Point(x, y));
//	}
	polygon = new Polygon(points);

	run(argc, argv);
	return 0;
}
