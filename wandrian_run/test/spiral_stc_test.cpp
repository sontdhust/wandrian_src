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
#include "../include/plans/spiral_stc/full_spiral_stc.hpp"

#define T_SIZE 0.5 // Tool size
#define B_SIZE 4.0 // Default space size
#define WORLD_INSERT_OBSTACLE "<!-- INSERT: Bound and Obstacles here -->" // Flag at original world file to insert bound and obstacles into

using namespace wandrian::plans::spiral_stc;

double b_size = 0;

SpacePtr space;
PointPtr starting_point;
std::list<PointPtr> path;

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

  glScalef(b_size <= 20 ? 0.5 : 10.0 / b_size,
      b_size <= 20 ? 0.5 : 10.0 / b_size, 0);

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
  glRasterPos2i(0, b_size <= 20 ? -11 : -b_size / 2 - 1);
  std::stringstream ss;
  ss << starting_point->x << ", " << starting_point->y;
  for (int i = 0; i < ss.str().length(); i++)
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ss.str()[i]);

  // Spiral STC covering path
  glColor3ub(0, 255, 0);
  draw(path, GL_LINE_STRIP);

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

bool test_go_to(PointPtr position, bool) {
  path.insert(path.end(), position);
  return true;
}

bool test_see_obstacle(VectorPtr direction, double distance) {
  // Simulator check obstacle
  PointPtr last_position = *(--path.end());
  PointPtr new_position = last_position + direction * distance;
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
    if (!(iss >> b_size) || !((int) b_size % 2 == 0)) {
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
              - (int) (b_size / T_SIZE / 4.0)) + T_SIZE + T_SIZE / 2.0,
          (std::rand() % (int) (b_size / T_SIZE / 2.0)
              - (int) (b_size / T_SIZE / 4.0)) + T_SIZE - T_SIZE / 2.0));

  double o_size;
  int number_of_obstacles;
  double r = 0.2;
  if (argc >= 3) {
    std::istringstream iss(argv[2]);
    if (!(iss >> o_size)) {
      o_size = 2.0 * T_SIZE;
      number_of_obstacles = 0;
    } else {
      double n = 0.75 * r * (b_size * b_size) / (o_size * o_size);
      number_of_obstacles = n
          + ((int) (0.25 * n) != 0 ? std::rand() % (int) (0.25 * n) : 0);
    }
  } else {
    o_size = 2.0 * T_SIZE;
    double n = 0.75 * r * (b_size * b_size) / (o_size * o_size);
    number_of_obstacles = n
        + ((int) (0.25 * n) != 0 ? std::rand() % (int) (0.25 * n) : 0);
  }

  for (int i = 0; i <= number_of_obstacles; i++) {
    PointPtr center = PointPtr(
        new Point(
            (std::rand() % (int) (b_size / T_SIZE / (o_size / T_SIZE))
                - (int) (b_size / T_SIZE / (o_size / T_SIZE) / 2.0) + T_SIZE)
                * o_size,
            (std::rand() % (int) (b_size / T_SIZE / (o_size / T_SIZE))
                - (int) (b_size / T_SIZE / (o_size / T_SIZE) / 2.0) + T_SIZE)
                * o_size));
    bool valid = true;
    double EPS = std::numeric_limits<double>::epsilon();
    for (std::list<PolygonPtr>::iterator p = obstacles.begin();
        p != obstacles.end(); p++)
      if ((boost::static_pointer_cast<Cell>(*p))->get_center() == center
          || (std::abs(center->x - (starting_point->x - T_SIZE / 2)) < EPS
              && std::abs(center->y - (starting_point->y + T_SIZE / 2)) < EPS)) {
        valid = false;
        break;
      };
    if (valid) {
      obstacles.insert(obstacles.end(), CellPtr(new Cell(center, o_size)));
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
        CellPtr cell = boost::static_pointer_cast<Cell>(*o);
        PointPtr c = cell->get_center();
        double s = cell->get_size();
        double x = c->x - T_SIZE * (s / T_SIZE / 2.0 - 1.0 / 2.0);
        for (int i = 1; i <= (int) (s / T_SIZE); i++) {
          for (double y = c->y - T_SIZE * (s / T_SIZE / 2.0 - 1.0 / 4.0);
              y <= c->y + T_SIZE * (s / T_SIZE / 2.0 - 1.0 / 4.0);
              y += T_SIZE / 2.0) {
            world_out << "    <model name='cinder_block_obstacle_" << n << "_"
                << i << "'>\n";
            world_out << "      <include>\n";
            world_out << "        <uri>model://cinder_block</uri>\n";
            world_out << "      </include>\n";
            world_out << "      <pose>" << x << " " << y << " 0 0 0 0</pose>\n";
            world_out << "      <static>1</static>\n";
            world_out << "    </model>\n";
          }
          x += T_SIZE;
        }
        n++;
      }
    }
  }
  world_in.close();
  world_out.close();

  space = SpacePtr(new Space(bound, obstacles));
  if (argc >= 4) {
    if (std::string(argv[3]) == "spiral_stc") {
      SpiralStcPtr plan_spiral_stc = SpiralStcPtr(new SpiralStc());
      plan_spiral_stc->initialize(starting_point, T_SIZE);
      path.insert(path.end(), starting_point);
      plan_spiral_stc->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));
      plan_spiral_stc->set_behavior_see_obstacle(
          boost::bind(&test_see_obstacle, _1, _2));
      plan_spiral_stc->cover();
    } else if (std::string(argv[3]) == "full_spiral_stc") {
      FullSpiralStcPtr plan_full_spiral_stc = FullSpiralStcPtr(
          new FullSpiralStc());
      plan_full_spiral_stc->initialize(starting_point, T_SIZE);
      path.insert(path.end(), starting_point);
      plan_full_spiral_stc->set_behavior_go_to(
          boost::bind(&test_go_to, _1, _2));
      plan_full_spiral_stc->set_behavior_see_obstacle(
          boost::bind(&test_see_obstacle, _1, _2));
      plan_full_spiral_stc->cover();
    }
  }
  run(argc, argv);
  return 0;
}
