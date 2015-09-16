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
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-6, 6, -6, 6, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glScalef(0.25, 0.25, 0);

	// TODO draw environment here

	glPointSize(4);
	glColor3ub(255, 255, 0);
	glBegin(GL_POINTS);
	glVertex2i(0, 0);
	glEnd();

	glPointSize(1);
	glColor3ub(255, 255, 255);
	glBegin(GL_POINTS);
	for (int i = -20; i <= 20; i++) {
		for (int j = -20; j <= 20; j++) {
			if ((i != 0 || j != 0) && i % 2 == 0 && j % 2 == 0)
				glVertex2i(i, j);
		}
	}
	glEnd();

	glColor3ub(255, 0, 0);

	std::set<Point*> ps = environment->space->get_points();
	glBegin(GL_LINE_STRIP);
	for (std::set<Point*>::iterator p = ps.begin(); p != ps.end(); p++) {
		glVertex2d((*p)->x, (*p)->y);
	}
	glVertex2d((*ps.begin())->x, (*ps.begin())->y);
	glEnd();

	std::cout << "[Obstacles]: " << environment->obstacles.size() << "\n";
	for (std::set<Polygon*>::iterator obstacle = environment->obstacles.begin();
			obstacle != environment->obstacles.end(); obstacle++) {
		std::set<Point*> ps = (*obstacle)->get_points();
		glBegin(GL_POLYGON);
		for (std::set<Point*>::iterator p = ps.begin(); p != ps.end(); p++) {
			glVertex2d((*p)->x, (*p)->y);
		}
		glVertex2d((*ps.begin())->x, (*ps.begin())->y);
		glEnd();
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
	Cell *space = new Cell(new Point(0, 0), 40);
	std::set<Polygon*> obstacles;

	std::srand(std::time(0));
	int r = std::rand() % 19 + 20;
	for (int i = 0; i <= r; i++) {
		Point *center = new Point((std::rand() % 19 - 9) * 2 + 1,
				(std::rand() % 19 - 9) * 2 + 1);
		bool valid = true;
		for (std::set<Polygon*>::iterator p = obstacles.begin();
				p != obstacles.end(); p++)
			if (*(((Cell*) *p)->get_center()) == *center) {
				valid = false;
				break;
			};
		if (valid)
			obstacles.insert(new Cell(center, 2));
	}

	environment = new Environment(space, obstacles);
	spiral_stc = new SpiralStc(environment, new Cell(new Point(0, 0), 2));

	run(argc, argv);
	return 0;
}
