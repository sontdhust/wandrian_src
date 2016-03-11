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
#include <boost/bind.hpp>
#include <sstream>
#include <fstream>

#include "../include/common/space.hpp"
#include "../include/plans/boustrophedon_online/boustrophedon_online.hpp"

#define T_SIZE 0.5 // tool size
#define B_SIZE 4.0 // default space size
#define WORLD_INSERT_OBSTACLE "<!-- INSERT: Bound and Obstacles here -->" // flag at original world file to insert bound and obstacles into

using namespace wandrian::plans::boustrophedon_online;

double b_size = 0;

SpacePtr space;
PointPtr starting_point;
BoustrophedonOnlinePtr boustrophedon_online;
std::list<PointPtr> tmp_path;

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
  for (int i = -b_size; i <= b_size; i++) {
    for (int j = -b_size; j <= b_size; j++) {
      if ((i != 0 || j != 0) && i % 2 == 0 && j % 2 == 0)
        glVertex2i((double) i / 2, (double) j / 2);
    }
  }
  glEnd();

  // Environment
  glColor3ub(255, 0, 0);
  draw(space->boundary->get_bound(), GL_LINE_STRIP);
  for (std::list<PolygonPtr>::iterator obstacle = space->obstacles.begin();
      obstacle != space->obstacles.end(); obstacle++) {
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
  draw(tmp_path, GL_LINE_STRIP);

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
  tmp_path.insert(tmp_path.end(), position);
  return true;
}

bool test_see_obstacle(VectorPtr orientation, double distance) {
  // Simulator check obstacle
  PointPtr last_position = *(--tmp_path.end());
  PointPtr new_position = PointPtr(
      new Point(last_position + orientation * distance));
  if (space) {
    CellPtr boundary = boost::static_pointer_cast<Cell>(space->boundary);
    if (new_position->x >= boundary->get_center()->x + boundary->get_size() / 2
        || new_position->x
            <= boundary->get_center()->x - boundary->get_size() / 2
        || new_position->y
            >= boundary->get_center()->y + boundary->get_size() / 2
        || new_position->y
            <= boundary->get_center()->y - boundary->get_size() / 2) {
      return true;
    }
    for (std::list<PolygonPtr>::iterator o = space->obstacles.begin();
        o != space->obstacles.end(); o++) {
      CellPtr obstacle = boost::static_pointer_cast<Cell>(*o);
      if (new_position->x
          >= obstacle->get_center()->x - obstacle->get_size() / 2
          && new_position->x
              <= obstacle->get_center()->x + obstacle->get_size() / 2
          && new_position->y
              >= obstacle->get_center()->y - obstacle->get_size() / 2
          && new_position->y
              <= obstacle->get_center()->y + obstacle->get_size() / 2) {
        return true;
      }
    }
  }
  return false;
}

int main(int argc, char **argv) {
  if (argc >= 2) {
    std::istringstream iss(argv[1]);
    if (!(iss >> b_size)
        || !(4 <= b_size && b_size <= 20 && (int) b_size % 2 == 0)) {
      b_size = B_SIZE;
    }
  } else {
    b_size = B_SIZE;
  }

  CellPtr bound = CellPtr(new Cell(PointPtr(new Point(0, 0)), b_size));
  std::list<PolygonPtr> obstacles;

  std::srand(std::time(0));
  starting_point = PointPtr(
      new Point(
          (std::rand() % (int) (b_size / T_SIZE / 2.0)
              - (int) (b_size / T_SIZE / 4.0)) + T_SIZE + T_SIZE / 2,
          (std::rand() % (int) (b_size / T_SIZE / 2.0)
              - (int) (b_size / T_SIZE / 4.0)) + T_SIZE - T_SIZE / 2));

  int r = std::rand() % (int) (b_size * b_size / 16) + b_size * b_size / 8;
  for (int i = 0; i <= r; i++) {
    PointPtr center = PointPtr(
        new Point(
            (std::rand() % (int) (b_size / T_SIZE / 2.0)
                - (int) (b_size / T_SIZE / 4.0)) + T_SIZE,
            (std::rand() % (int) (b_size / T_SIZE / 2.0)
                - (int) (b_size / T_SIZE / 4.0)) + T_SIZE));
    bool valid = true;
    for (std::list<PolygonPtr>::iterator p = obstacles.begin();
        p != obstacles.end(); p++)
      if ((boost::static_pointer_cast<Cell>(*p))->get_center() == center
          || (center->x == starting_point->x - T_SIZE / 2
              && center->y == starting_point->y + T_SIZE / 2)) {
        valid = false;
        break;
      };
    if (valid) {
      obstacles.insert(obstacles.end(), CellPtr(new Cell(center, 2 * T_SIZE)));
    }
  }

  std::ifstream world_in("../../worlds/empty.world");
  std::ofstream world_out("../../worlds/tmp.world");
  std::string line;
  while (std::getline(world_in, line, '\n')) {
    world_out << line << '\n';
    if (line.find(WORLD_INSERT_OBSTACLE) != std::string::npos) {
      int n;
      n = 1;
      // Upper bound
      for (double i = -b_size / 2 + T_SIZE / 2; i <= b_size / 2 - T_SIZE / 2;
          i += T_SIZE) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " " << (b_size / 2 + T_SIZE / 4)
            << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      // Right bound
      for (double i = -b_size / 2 + T_SIZE / 2; i <= b_size / 2 - T_SIZE / 2;
          i += T_SIZE) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << (b_size / 2 + T_SIZE / 4) << " " << -i
            << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      // Lower bound
      for (double i = -b_size / 2 + T_SIZE / 2; i <= b_size / 2 - T_SIZE / 2;
          i += T_SIZE) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << -i << " " << -(b_size / 2 + T_SIZE / 4)
            << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      // Left bound
      for (double i = -b_size / 2 + T_SIZE / 2; i <= b_size / 2 - T_SIZE / 2;
          i += T_SIZE) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << -(b_size / 2 + T_SIZE / 4) << " " << i
            << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      n = 1;
      // Obstacles
      for (std::list<PolygonPtr>::iterator o = obstacles.begin();
          o != obstacles.end(); o++) {
        PointPtr p = (boost::static_pointer_cast<Cell>(*o))->get_center();
        int c = 1;
        for (double i = p->y - T_SIZE * 3 / 4; i <= p->y + T_SIZE * 3 / 4;
            i += T_SIZE / 2) {
          world_out << "    <model name='cinder_block_obstacle_" << n << "_"
              << c << "'>\n";
          world_out << "      <include>\n";
          world_out << "        <uri>model://cinder_block</uri>\n";
          world_out << "      </include>\n";
          world_out << "      <pose>" << p->x - T_SIZE / 2 << " " << i
              << " 0 0 0 0</pose>\n";
          world_out << "      <static>1</static>\n";
          world_out << "    </model>\n";
          c++;
        }
        for (double i = p->y - T_SIZE * 3 / 4; i <= p->y + T_SIZE * 3 / 4;
            i += T_SIZE / 2) {
          world_out << "    <model name='cinder_block_obstacle_" << n << "_"
              << c << "'>\n";
          world_out << "      <include>\n";
          world_out << "        <uri>model://cinder_block</uri>\n";
          world_out << "      </include>\n";
          world_out << "      <pose>" << p->x + T_SIZE / 2 << " " << i
              << " 0 0 0 0</pose>\n";
          world_out << "      <static>1</static>\n";
          world_out << "    </model>\n";
          c++;
        }
        n++;
      }
    }
  }
  world_in.close();
  world_out.close();

  space = SpacePtr(new Space(bound, obstacles));
  boustrophedon_online = BoustrophedonOnlinePtr(new BoustrophedonOnline());
  boustrophedon_online->initialize(starting_point, T_SIZE);
  tmp_path.insert(tmp_path.end(), starting_point);
  boustrophedon_online->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));
  boustrophedon_online->set_behavior_see_obstacle(
      boost::bind(&test_see_obstacle, _1, _2));
  boustrophedon_online->cover();

  run(argc, argv);
  return 0;
}
