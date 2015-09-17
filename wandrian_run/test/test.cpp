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

#define SUB_CELL_SIZE 1
#define STARTING_POINT new Point(1.5, 0.5)

using namespace wandrian::plans::spiral_stc;

Environment *environment;
SpiralStc *spiral_stc;

/**
 * Linked libraries to compile: -lglut -lGL (g++)
 */

void draw(std::set<Point*> points, int type) {
	glBegin(type);
	for (std::set<Point*>::iterator p = points.begin(); p != points.end(); p++) {
		glVertex2d((*p)->x, (*p)->y);
	}
	glVertex2d((*points.begin())->x, (*points.begin())->y);
	glEnd();
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-6, 6, -6, 6, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glScalef(0.25, 0.25, 0);

	// Center point
	glPointSize(4);
	glColor3ub(255, 255, 0);
	glBegin(GL_POINTS);
	glVertex2i(0, 0);
	glEnd();

	// Coordinate
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

	// Environment
	glColor3ub(255, 0, 0);
	draw(environment->space->get_points(), GL_LINE_STRIP);
	for (std::set<Polygon*>::iterator obstacle = environment->obstacles.begin();
			obstacle != environment->obstacles.end(); obstacle++) {
		draw((*obstacle)->get_points(), GL_POLYGON);
	}

	// Starting point
	glPointSize(4);
	glColor3ub(0, 255, 0);
	glBegin(GL_POINTS);
	glVertex2d((STARTING_POINT)->x, (STARTING_POINT)->y);
	glEnd();

	// Spiral STC covering path
	glColor3ub(0, 255, 0);
	draw(spiral_stc->get_path(), GL_LINE_STRIP);

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
	Cell *space = new Cell(new Point(0, 0), 32);
	std::set<Polygon*> obstacles;

//	std::cout << "[Obstacles]: ";
//	std::srand(std::time(0));
//	int r = std::rand() % 21 + 30;
//	for (int i = 0; i <= r; i++) {
//		Point *center = new Point((std::rand() % 19 - 9) * 2 + 1,
//				(std::rand() % 19 - 9) * 2 + 1);
//		bool valid = true;
//		for (std::set<Polygon*>::iterator p = obstacles.begin();
//				p != obstacles.end(); p++)
//			if (*(((Cell*) *p)->get_center()) == *center
//					|| (center->x == 1 && center->y == 1)) {
//				valid = false;
//				break;
//			};
//		if (valid) {
//			obstacles.insert(new Cell(center, 2));
//			std::cout << " " << center->x << "," << center->y << "\n";
//		}
//	}
//	std::cout << "\n";

	obstacles.insert(new Cell(new Point(19, 3), 2));
	obstacles.insert(new Cell(new Point(5, 1), 2));
	obstacles.insert(new Cell(new Point(9, 15), 2));
	obstacles.insert(new Cell(new Point(15, 15), 2));
	obstacles.insert(new Cell(new Point(9, -5), 2));
	obstacles.insert(new Cell(new Point(17, 11), 2));
	obstacles.insert(new Cell(new Point(1, -13), 2));
	obstacles.insert(new Cell(new Point(-15, -11), 2));
	obstacles.insert(new Cell(new Point(7, -3), 2));
	obstacles.insert(new Cell(new Point(-11, 17), 2));
	obstacles.insert(new Cell(new Point(-5, 7), 2));
	obstacles.insert(new Cell(new Point(-11, 5), 2));
	obstacles.insert(new Cell(new Point(3, -7), 2));
	obstacles.insert(new Cell(new Point(15, -7), 2));
	obstacles.insert(new Cell(new Point(-9, -1), 2));
	obstacles.insert(new Cell(new Point(-3, -17), 2));
	obstacles.insert(new Cell(new Point(-17, 13), 2));
	obstacles.insert(new Cell(new Point(-3, 3), 2));
	obstacles.insert(new Cell(new Point(3, -9), 2));
	obstacles.insert(new Cell(new Point(-1, -9), 2));
	obstacles.insert(new Cell(new Point(3, -11), 2));
	obstacles.insert(new Cell(new Point(13, 15), 2));
	obstacles.insert(new Cell(new Point(-7, 9), 2));
	obstacles.insert(new Cell(new Point(15, 13), 2));
	obstacles.insert(new Cell(new Point(-15, 15), 2));
	obstacles.insert(new Cell(new Point(9, -7), 2));
	obstacles.insert(new Cell(new Point(1, 15), 2));
	obstacles.insert(new Cell(new Point(9, -15), 2));
	obstacles.insert(new Cell(new Point(-5, -17), 2));

	environment = new Environment(space, obstacles);
	spiral_stc = new SpiralStc(environment, STARTING_POINT, SUB_CELL_SIZE);
	spiral_stc->cover();

	run(argc, argv);
	return 0;
}
