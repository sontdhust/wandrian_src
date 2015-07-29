/*
 * main.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: sontd
 */

#include <GL/glut.h>
#include <stdio.h>
#include <iostream>
#include <limits>
#include <set>
#include <ctime>
#include <cstdlib>
#include <boost/next_prior.hpp>
#include "../include/common/environment.hpp"

using namespace common::shapes;

std::set<std::map<Point*, std::set<Point*, PointComp>, PointComp> > graphs;

/**
 * Linked libraries to compile: -lglut -lGL (g++)
 */

void display() {
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-6, 6, -6, 6, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glScalef(0.5, 0.5, 0);

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

	// TODO draw primitives here

	glColor3ub(255, 255, 255);

	for (std::set<std::map<Point*, std::set<Point*, PointComp>, PointComp> >::iterator graph =
			graphs.begin(); graph != graphs.end(); graph++) {
		for (std::map<Point*, std::set<Point*, PointComp> >::const_iterator current =
				graph->begin(); current != graph->end(); current++) {
			std::cout << "\033[1;31m" << (*current).first->x << " "
					<< (*current).first->y << "\033[1;0m: ";
			for (std::set<Point*>::iterator adjacent = (*current).second.begin();
					adjacent != (*current).second.end(); adjacent++) {
				glBegin(GL_LINE_STRIP);
				glVertex2d((*current).first->x, (*current).first->y);
				glVertex2d((*adjacent)->x, (*adjacent)->y);
				glEnd();
				std::cout << (*adjacent)->x << " " << (*adjacent)->y << ", ";
			}
			std::cout << "\n";
		}
		std::cout << "\n";
	}
	std::cout << "\n";

	glutSwapBuffers();
}

int run(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(600, 600);
	glutCreateWindow("GLUT");
	glutDisplayFunc(display);
	glutMainLoop();
	return 0;
}

int main(int argc, char **argv) {
	Polygon *space;
	{
		std::set<Point*> points;
		points.clear();
		std::srand(std::time(0));
		for (int i = 0; i <= std::rand() % 10; i++) {
			int x = std::rand() % 2 == 0 ? std::rand() % 4 - 10 : std::rand() % 4 + 7;
			int y = std::rand() % 2 == 0 ? std::rand() % 4 - 10 : std::rand() % 4 + 7;
			std::cout << x << " " << y << "\n";
			points.insert(new Point(x, y));
		}
		space = new Polygon(points);
	}

	std::set<Polygon*> obstacles;

	std::srand(std::time(0));
	for (int i = 0; i <= std::rand() % 5; i++) {
		std::set<Point*> points;
		std::cout << i << ":\n";
		for (int j = 0; j <= std::rand() % 4 + 2; j++) {
			int x = std::rand() % 11 - 5;
			int y = std::rand() % 11 - 5;
			std::cout << "  " << x << " " << y << "\n";
			points.insert(new Point(x, y));
		}
		obstacles.insert(new Polygon(points));
		points.clear();
		for (std::set<Point*>::iterator p = points.begin(); p != points.end(); p++)
			delete *p;
	}

	Environment *environment = new Environment(space, obstacles);
	graphs.insert(environment->get_space()->get_graph());
	std::set<Polygon*> os = environment->get_obstacles();
	for (std::set<Polygon*>::iterator obstacle = os.begin(); obstacle != os.end();
			obstacle++) {
		graphs.insert((*obstacle)->get_graph());
	}

	run(argc, argv);
	return 0;
}
