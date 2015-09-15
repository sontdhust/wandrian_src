/*
 * test.cpp
 *
 *  Created on: Jul 31, 2015
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
#include "../include/plans/spiral_stc/spiral_stc.hpp"

#define CELL_SIZE 2

using namespace wandrian::plans::spiral_stc;

Environment *environment;
SpiralStc *spiral_stc;

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

	// TODO draw primitives here

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

	glColor3ub(255, 255, 255);
	for (std::map<Point*, std::set<Point*, PointComp> >::iterator current =
			environment->space->get_graph().begin();
			current != environment->space->get_graph().end(); current++) {
		for (std::set<Point*>::iterator adjacent = (*current).second.begin();
				adjacent != (*current).second.end(); adjacent++) {
			glBegin(GL_LINE_STRIP);
			glVertex2d((*current).first->x, (*current).first->y);
			glVertex2d((*adjacent)->x, (*adjacent)->y);
			glEnd();
		}
	}

	std::cout << "[Obstacles]: " << environment->obstacles.size() << "\n";
	for (std::set<Polygon*>::iterator obstacle = environment->obstacles.begin();
			obstacle != environment->obstacles.end(); obstacle++) {
		for (std::map<Point*, std::set<Point*, PointComp> >::iterator current =
				(*obstacle)->get_graph().begin();
				current != (*obstacle)->get_graph().end(); current++) {
			for (std::set<Point*>::iterator adjacent = (*current).second.begin();
					adjacent != (*current).second.end(); adjacent++) {
				glBegin(GL_LINE_STRIP);
				glVertex2d((*current).first->x, (*current).first->y);
				glVertex2d((*adjacent)->x, (*adjacent)->y);
				glEnd();
			}
		}
	}

	glutSwapBuffers();
}

int run(int argc, char **argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(600, 600);
	glutCreateWindow("Environment");
	glutDisplayFunc(display);
	glutMainLoop();
	return 0;
}

int main(int argc, char **argv) {
	Cell *space = new Cell(new Point(0, 0), 20);
	std::set<Polygon*> obstacles;

	std::srand(std::time(0));
	int r = std::rand() % 5;
	for (int i = 0; i <= r; i++) {
		obstacles.insert(
				new Cell(new Point(std::rand() % 19 - 9, std::rand() % 19 - 9), 2));
	}
	environment = new Environment(space, obstacles);
	spiral_stc = new SpiralStc(environment, new Cell(new Point(0, 0), 2));

	run(argc, argv);
	return 0;
}
