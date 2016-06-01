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
#include <stdlib.h>
#include <boost/next_prior.hpp>
#include <sstream>
#include <fstream>
#include "../include/environment/boustrophedon/extended_map.hpp"
#include "../include/plans/boustrophedon/boustrophedon.hpp"

#define R_SIZE 0.5 // robot size
#define E_SIZE 4.0 // default environment size
#define WORLD_INSERT_OBSTACLE "<!-- INSERT: Boundary and Obstacles here -->" // flag at original world file to insert bound and obstacles into

using namespace wandrian::environment;
using namespace wandrian::plans::boustrophedon;

double e_size = 0;
double r_size = 0;
ExtendedMapPtr map;
PointPtr starting_point;
std::list<PointPtr> actual_path;
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

void draw_path(std::list<PointPtr> points, int type) {
  glBegin(type);
  for (std::list<PointPtr>::iterator p = points.begin(); p != points.end();
      p++) {
    glVertex2d((*p)->x, (*p)->y);
  }
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
  RectanglePtr boundary = map->get_boundary();
  for (double i = boundary->get_center()->x - boundary->get_width() / 2.0;
      i <= boundary->get_center()->x + boundary->get_width() / 2.0;
      i += r_size * 2.0) {
    for (double j = boundary->get_center()->y - boundary->get_height() / 2.0;
        j <= boundary->get_center()->y + boundary->get_height() / 2.0;
        j += r_size * 2.0) {
      if (i != 0 || j != 0)
        glVertex2d(i, j);
    }
  }
  glEnd();

  // Environment
  glColor3ub(255, 0, 0);
  draw(map->get_boundary()->get_boundary(), GL_LINE_STRIP);
  std::list<PolygonPtr> obstacles = map->get_extended_obstacles();
  for (std::list<PolygonPtr>::iterator obstacle = obstacles.begin();
      obstacle != obstacles.end(); obstacle++) {
    draw((*obstacle)->get_points(), GL_LINE_STRIP);
  }

  // Starting point
  glPointSize(4);
  glColor3ub(0, 255, 0);
  glBegin(GL_POINTS);
  glVertex2d(starting_point->x, starting_point->y);
  glEnd();

  // Boustrophedon covering path
  glColor3ub(0, 255, 0);
  draw_path(actual_path, GL_LINE_STRIP);

  glRasterPos2i(0, -11);
  std::stringstream ss;
  ss << starting_point->x << ", " << starting_point->y;
  for (int i = 0; i < ss.str().length(); i++)
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ss.str()[i]);

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

bool test_go_to(PointPtr position, bool flexibly) {
  actual_path.insert(actual_path.end(), position);
  return true;
}

int main(int argc, char **argv) {
  if (argc >= 2) {
    std::istringstream iss(argv[1]);
    if (!(iss >> e_size)
        || !(4 <= e_size && e_size <= 20 && (int) e_size % 2 == 0)) {
      e_size = E_SIZE;
    }
  } else {
    e_size = E_SIZE;
  }
  if (argc >= 3) {
    std::istringstream iss(argv[2]);
    if (!(iss >> r_size)) {
      r_size = R_SIZE;
    }
  } else {
    r_size = R_SIZE;
  }
  BoustrophedonPtr boustrophedon = BoustrophedonPtr(new Boustrophedon());
  boustrophedon->initialize(starting_point, r_size,
      "../../worlds/prefered_b.map");
  map = boustrophedon->get_map();
  CellPtr space = CellPtr(new Cell(PointPtr(new Point(0, 0)), e_size));
  std::srand(std::time(0));
  starting_point = PointPtr(
      new Point(
          (map->get_boundary()->get_center()->x * 2
              - map->get_boundary()->get_width() + r_size) / 2,
          (map->get_boundary()->get_center()->y * 2
              - map->get_boundary()->get_height() + r_size) / 2));
  int r = std::rand() % (int) (e_size * e_size / 16) + e_size * e_size / 8;
  std::ifstream world_in("../../worlds/empty.world");
  std::ofstream world_out("../../worlds/tmp.world");
  std::string line;

  while (std::getline(world_in, line, '\n')) {
    world_out << line << '\n';
    if (line.find(WORLD_INSERT_OBSTACLE) != std::string::npos) {
      int n;
      RectanglePtr boundary = map->get_boundary();
      // Upper boundary
      for (double i = boundary->get_center()->x - boundary->get_width() / 2
          + r_size / 2;
          i
              <= boundary->get_center()->x + boundary->get_width() / 2
                  - r_size / 2 + EPSILON; i += r_size / 2) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " "
            << (boundary->get_center()->y + boundary->get_height() / 2
                + r_size / 4) << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Right boundary
      for (double i = boundary->get_center()->y - boundary->get_height() / 2
          + r_size / 2;
          i
              <= boundary->get_center()->y + boundary->get_height() / 2
                  - r_size / 2 + EPSILON; i += r_size / 2) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>"
            << (boundary->get_center()->x + boundary->get_width() / 2
                + r_size / 4) << " " << i << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Lower boundary
      for (double i = boundary->get_center()->x - boundary->get_width() / 2
          + r_size / 2;
          i
              <= boundary->get_center()->x + boundary->get_width() / 2
                  - r_size / 2 + EPSILON; i += r_size / 2) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " "
            << (boundary->get_center()->y - boundary->get_height() / 2
                - r_size / 4) << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Left boundary
      for (double i = boundary->get_center()->y - boundary->get_height() / 2
          + r_size / 2;
          i
              <= boundary->get_center()->y + boundary->get_height() / 2
                  - r_size / 2 + EPSILON; i += r_size / 2) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>"
            << (boundary->get_center()->x - boundary->get_width() / 2
                - r_size / 4) << " " << i << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      n = 1;
//      double o_size = 4 * r_size;
      // Obstacles
      std::list<PolygonPtr> obstacles = map->get_extended_obstacles();
      for (std::list<PolygonPtr>::iterator o = obstacles.begin();
          o != obstacles.end(); o++) {
        std::list<PointPtr> points = (*o)->get_points();
        int c = 1;
        for (std::list<PointPtr>::iterator point = points.begin();
            point != points.end(); point++) {
          std::list<PointPtr>::iterator next;
          if (boost::next(point) != points.end())
            next = boost::next(point);
          else
            next = points.begin();
          if ((*next)->y == (*point)->y) { // Horizontal segment
            for (double x = (*point)->x;
                (*next)->x > (*point)->x ? (x < (*next)->x) : (x > (*next)->x);
                x += ((*next)->x > (*point)->x ? r_size / 2 : -r_size / 2)) {
              world_out << "    <model name='cinder_block_obstacle_" << n << "_"
                  << c << "'>\n";
              world_out << "      <include>\n";
              world_out << "        <uri>model://cinder_block</uri>\n";
              world_out << "      </include>\n";
              world_out << "      <pose>" << x << " " << (*point)->y
                  << " 0 0 0 0</pose>\n";
              world_out << "      <static>1</static>\n";
              world_out << "    </model>\n";
              c++;
            }
          }
          for (double y = (*point)->y;
              (*next)->y > (*point)->y ? (y < (*next)->y) : (y > (*next)->y);) {
            PointPtr p1 = PointPtr(
                new Point(
                    ((*point)->x > (*next)->x ? (*next)->x : (*point)->x)
                        - EPSILON, y));
            PointPtr p2 = PointPtr(
                new Point(
                    ((*point)->x < (*next)->x ? (*next)->x : (*point)->x)
                        + EPSILON, y));
            PointPtr p = SegmentPtr(new Segment(*point, *next))
                % SegmentPtr(new Segment(p1, p2));

            world_out << "    <model name='cinder_block_obstacle_" << n << "_"
                << c << "'>\n";
            world_out << "      <include>\n";
            world_out << "        <uri>model://cinder_block</uri>\n";
            world_out << "      </include>\n";
            world_out << "      <pose>" << p->x << " " << p->y
                << " 0 0 0 0</pose>\n";
            world_out << "      <static>1</static>\n";
            world_out << "    </model>\n";
            c++;
            int d = std::abs((*point)->x - (*next)->x)
                / std::abs((*point)->y - (*next)->y);
            if (d == 0)
              d = 1;
            if (d > 4)
              d = 4;
            y += ((*next)->y > (*point)->y ? r_size / 2 : -r_size / 2) / d;
          }
        }
        n++;
      }
    }
  }
  world_in.close();
  world_out.close();
  actual_path.insert(actual_path.end(), starting_point);
  boustrophedon->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));
  boustrophedon->cover();
  run(argc, argv);
  return 0;
}
