/*
 * test.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#include <GL/glut.h>
#include <limits>
#include <set>
#include <ctime>
#include <cstdlib>
#include <boost/next_prior.hpp>
#include "../include/plans/spiral_stc/spiral_stc.hpp"

#define R_SIZE 0.5 // robot size
#define E_SIZE 20.0 // environment size
//#define E_SIZE 4.0

using namespace wandrian::plans::spiral_stc;

EnvironmentPtr environment;
PointPtr starting_point;
SpiralStcPtr spiral_stc;

/**
 * Linked libraries to compile: -lglut -lGL (g++)
 */

void draw(std::list<PointPtr> points, int type) {
	glBegin(type);
	for (std::list<PointPtr>::iterator p = points.begin(); p != points.end();
			p++) {
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

	glScalef(0.5, 0.5, 0);

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
	for (int i = -E_SIZE; i <= E_SIZE; i++) {
		for (int j = -E_SIZE; j <= E_SIZE; j++) {
			if ((i != 0 || j != 0) && i % 2 == 0 && j % 2 == 0)
				glVertex2i((double) i / 2, (double) j / 2);
		}
	}
	glEnd();

	// Environment
	glColor3ub(255, 0, 0);
	draw(environment->space->get_bound(), GL_LINE_STRIP);
	for (std::list<PolygonPtr>::iterator obstacle =
			environment->obstacles.begin(); obstacle != environment->obstacles.end();
			obstacle++) {
		draw((*obstacle)->get_bound(), GL_POLYGON);
	}

	// Starting point
	glPointSize(4);
	glColor3ub(0, 255, 0);
	glBegin(GL_POINTS);
	glVertex2d(starting_point->x, starting_point->y);
	glEnd();

	// Spiral STC covering path
	glColor3ub(0, 255, 0);
	draw(spiral_stc->path, GL_LINE_STRIP);

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
	CellPtr space = CellPtr(new Cell(PointPtr(new Point(0, 0)), E_SIZE));
	std::list<PolygonPtr> obstacles;

//	std::cout << "[Obstacles]:\n";
	std::srand(std::time(0));

	starting_point = PointPtr(
			new Point(
					(std::rand() % (int) (E_SIZE / R_SIZE / 2.0)
							- (int) (E_SIZE / R_SIZE / 4.0)) + R_SIZE + R_SIZE / 2,
					(std::rand() % (int) (E_SIZE / R_SIZE / 2.0)
							- (int) (E_SIZE / R_SIZE / 4.0)) + R_SIZE - R_SIZE / 2));
//	starting_point = PointPtr(new Point(0.75, -0.75));

	int r = std::rand() % 31 + 40;
//	r = -1;
	for (int i = 0; i <= r; i++) {
		PointPtr center = PointPtr(
				new Point(
						(std::rand() % (int) (E_SIZE / R_SIZE / 2.0)
								- (int) (E_SIZE / R_SIZE / 4.0)) + R_SIZE,
						(std::rand() % (int) (E_SIZE / R_SIZE / 2.0)
								- (int) (E_SIZE / R_SIZE / 4.0)) + R_SIZE));
		bool valid = true;
		for (std::list<PolygonPtr>::iterator p = obstacles.begin();
				p != obstacles.end(); p++)
			if (*((boost::static_pointer_cast<Cell>(*p))->get_center()) == *center
					|| (center->x == starting_point->x - R_SIZE / 2
							&& center->y == starting_point->y + R_SIZE / 2)) {
				valid = false;
				break;
			};
		if (valid) {
			obstacles.insert(obstacles.end(), CellPtr(new Cell(center, 2 * R_SIZE)));
//			std::cout
//					<< "  obstacles.insert(obstacles.end(), CellPtr(new Cell(PointPtr(new Point("
//					<< center->x << ", " << center->y << ")), 2)));\n";
		}
	}
//	std::cout << "\n";

	environment = EnvironmentPtr(new Environment(space, obstacles));
	spiral_stc = SpiralStcPtr(new SpiralStc());
	spiral_stc->initialize(starting_point, R_SIZE);
	spiral_stc->set_environment(environment);
	spiral_stc->cover();

	run(argc, argv);
	return 0;
}
