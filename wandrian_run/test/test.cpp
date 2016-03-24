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
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <fstream>
#include <stdlib.h>

#include "../include/environment/space.hpp"
#include "../include/plans/spiral_stc/full_spiral_stc.hpp"
#include "../include/plans/boustrophedon_online/boustrophedon_online.hpp"

#define T_SIZE 0.5 // Tool size
#define B_SIZE 4.0 // Default space boundary size
#define WORLD_INSERT_OBSTACLE "<!-- INSERT: Boundary and Obstacles here -->" // Flag at original world file to insert boundary and obstacles into

using namespace wandrian::common;
using namespace wandrian::plans::spiral_stc;
using namespace wandrian::plans::boustrophedon_online;

double b_size = 0;
double t_size;

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

  glScalef((b_size <= 20 ? 5.0 : 10.0) / b_size,
      (b_size <= 20 ? 5.0 : 10.0) / b_size, 0);

  // Center point
  glPointSize(4);
  glColor3ub(255, 255, 0);
  glBegin(GL_POINTS);
  glVertex2i(0, 0);
  glEnd();

  // Coordinate
  glPointSize(b_size <= 8 ? 2 : 1);
  glColor3ub(255, 255, 255);
  glBegin(GL_POINTS);
  RectanglePtr boundary = boost::static_pointer_cast<Rectangle>(
      space->boundary);
  for (double i = boundary->get_center()->x - boundary->get_width() / 2.0;
      i <= boundary->get_center()->x + boundary->get_width() / 2.0;
      i += t_size * 2.0) {
    for (double j = boundary->get_center()->y - boundary->get_height() / 2.0;
        j <= boundary->get_center()->y + boundary->get_height() / 2.0;
        j += t_size * 2.0) {
      if (i != 0 || j != 0)
        glVertex2d(i, j);
    }
  }
  glEnd();

  // Space
  glColor3ub(255, 0, 0);
  draw(space->boundary->get_boundary(), GL_LINE_STRIP);
  for (std::list<PolygonPtr>::iterator obstacle = space->obstacles.begin();
      obstacle != space->obstacles.end(); obstacle++) {
    draw((*obstacle)->get_boundary(), GL_POLYGON);
  }

  // Starting point
  glPointSize(b_size <= 8 ? 8 : 4);
  glColor3ub(0, 255, 0);
  glBegin(GL_POINTS);
  glVertex2d(starting_point->x, starting_point->y);
  glEnd();
  glRasterPos2i(0, -b_size / 2 - 1);
  std::stringstream ss;
  ss << starting_point->x << ", " << starting_point->y;
  for (int i = 0; i < ss.str().length(); i++)
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ss.str()[i]);

  // Covering path
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
    RectanglePtr boundary = boost::static_pointer_cast<Rectangle>(
        space->boundary);
    if (new_position->x
        >= boundary->get_center()->x + boundary->get_width() / 2 - EPSILON
        || new_position->x
            <= boundary->get_center()->x - boundary->get_width() / 2 + EPSILON
        || new_position->y
            >= boundary->get_center()->y + boundary->get_height() / 2 - EPSILON
        || new_position->y
            <= boundary->get_center()->y - boundary->get_height() / 2
                + EPSILON) {
      return true;
    }
    for (std::list<PolygonPtr>::iterator o = space->obstacles.begin();
        o != space->obstacles.end(); o++) {
      CellPtr obstacle = boost::static_pointer_cast<Cell>(*o);
      if (new_position->x
          >= obstacle->get_center()->x - obstacle->get_size() / 2 - EPSILON
          && new_position->x
              <= obstacle->get_center()->x + obstacle->get_size() / 2 + EPSILON
          && new_position->y
              >= obstacle->get_center()->y - obstacle->get_size() / 2 - EPSILON
          && new_position->y
              <= obstacle->get_center()->y + obstacle->get_size() / 2
                  + EPSILON) {
        return true;
      }
    }
  }
  return false;
}

int main(int argc, char **argv) {
  bool map_input = false;
  RectanglePtr boundary;
  std::vector<PointPtr> obstacle_centers;
  // Boundary size and obstacle center from input map
  if (argc >= 2) {
    std::istringstream iss(argv[1]);
    if (!(iss >> b_size)) { // Read from input map file
      b_size = B_SIZE;
      map_input = true;
      std::ifstream map(("../" + std::string(argv[1])).c_str());
      std::string line;
      // Boundary size
      std::getline(map, line, '\n');
      int delimiter_pos1;
      int delimiter_pos2;
      delimiter_pos1 = line.find(" ");
      double center_x = boost::lexical_cast<double>(
          boost::lexical_cast<double>(line.substr(0, delimiter_pos1)));
      delimiter_pos2 = line.find(" ", delimiter_pos1 + 1);
      double center_y = boost::lexical_cast<double>(
          boost::lexical_cast<double>(
              line.substr(delimiter_pos1 + 1,
                  delimiter_pos2 - delimiter_pos1 - 1)));
      delimiter_pos1 = delimiter_pos2;
      delimiter_pos2 = line.find(" ", delimiter_pos1 + 1);
      double width = boost::lexical_cast<double>(
          boost::lexical_cast<double>(
              line.substr(delimiter_pos1 + 1,
                  delimiter_pos2 - delimiter_pos1 - 1)));
      double height = boost::lexical_cast<double>(
          boost::lexical_cast<double>(
              line.substr(delimiter_pos2 + 1,
                  line.length() - delimiter_pos2 - 1)));
      boundary = RectanglePtr(
          new Rectangle(PointPtr(new Point(center_x, center_y)), width,
              height));
      // Starting point
      double starting_point_x;
      double starting_point_y;
      std::getline(map, line, '\n');
      int delimiter_pos = line.find(" ");
      starting_point_x = boost::lexical_cast<double>(
          line.substr(0, delimiter_pos));
      starting_point_y = boost::lexical_cast<double>(
          line.substr(delimiter_pos + 1, line.length() - delimiter_pos - 1));
      starting_point = PointPtr(new Point(starting_point_x, starting_point_y));
      // Center point of obstacles
      while (std::getline(map, line, '\n')) {
        if (line == "")
          continue;
        double center_point_x;
        double center_point_y;
        int delimiter_pos = line.find(" ");
        center_point_x = boost::lexical_cast<double>(
            line.substr(0, delimiter_pos));
        center_point_y = boost::lexical_cast<double>(
            line.substr(delimiter_pos + 1, line.length() - delimiter_pos - 1));
        obstacle_centers.push_back(
            PointPtr(new Point(center_point_x, center_point_y)));
      }
    } else {
      map_input = false;
      boundary = RectanglePtr(
          new Rectangle(PointPtr(new Point(0, 0)), b_size, b_size));
    }
  } else {
    b_size = B_SIZE;
  }
  std::list<PolygonPtr> obstacles;

  // Tool size
  if (argc >= 3) {
    std::istringstream iss(argv[2]);
    if (!(iss >> t_size)) {
      t_size = T_SIZE;
    }
  } else {
    t_size = T_SIZE;
  }

  std::srand(std::time(0));
  if (!map_input)
    starting_point = PointPtr(
        new Point(
            (std::rand() % (int) (b_size / t_size / 2.0)
                - (int) (b_size / t_size / 4.0)) * t_size / 0.5 + t_size
                + t_size / 2.0,
            (std::rand() % (int) (b_size / t_size / 2.0)
                - (int) (b_size / t_size / 4.0)) * t_size / 0.5 + t_size
                - t_size / 2.0));

  // Obstacle size
  double o_size;
  int number_of_obstacles;
  double r = 0.2;
  if (argc >= 4) {
    std::istringstream iss(argv[3]);
    if (!(iss >> o_size)) {
      o_size = 2.0 * t_size;
      number_of_obstacles = 0;
    } else {
      double n = 0.75 * r * (b_size * b_size) / (o_size * o_size);
      number_of_obstacles = n
          + ((int) (0.25 * n) != 0 ? std::rand() % (int) (0.25 * n) : 0);
    }
  } else {
    o_size = 2.0 * t_size;
    double n = 0.75 * r * (b_size * b_size) / (o_size * o_size);
    number_of_obstacles = n
        + ((int) (0.25 * n) != 0 ? std::rand() % (int) (0.25 * n) : 0);
  }

  for (int i = 0;
      i <= (map_input ? (int) obstacle_centers.size() - 1 : number_of_obstacles);
      i++) {
    PointPtr center =
        map_input ?
            obstacle_centers[i] :
            PointPtr(
                new Point(
                    ((std::rand() % (int) (b_size / t_size / (o_size / t_size))
                        - (int) (b_size / t_size / (o_size / t_size) / 2.0))
                        + 0.5) * o_size,
                    ((std::rand() % (int) (b_size / t_size / (o_size / t_size))
                        - (int) (b_size / t_size / (o_size / t_size) / 2.0))
                        + 0.5) * o_size));
    bool valid = true;
    for (std::list<PolygonPtr>::iterator p = obstacles.begin();
        p != obstacles.end(); p++)
      if ((boost::static_pointer_cast<Cell>(*p))->get_center() == center
          || (std::abs(center->x - (starting_point->x - t_size / 2))
              < SMALL_EPSILON
              && std::abs(center->y - (starting_point->y + t_size / 2))
                  < SMALL_EPSILON)) {
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
      // Upper boundary
      for (double i = boundary->get_center()->x - boundary->get_width() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->x + boundary->get_width() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " "
            << (boundary->get_center()->y + boundary->get_height() / 2
                + t_size / 4) << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Right boundary
      for (double i = boundary->get_center()->y - boundary->get_height() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->y + boundary->get_height() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>"
            << (boundary->get_center()->x + boundary->get_width() / 2
                + t_size / 4) << " " << i << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Lower boundary
      for (double i = boundary->get_center()->x - boundary->get_width() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->x + boundary->get_width() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " "
            << (boundary->get_center()->y - boundary->get_height() / 2
                - t_size / 4) << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }
      // Left boundary
      for (double i = boundary->get_center()->y - boundary->get_height() / 2
          + t_size / 2;
          i
              <= boundary->get_center()->y + boundary->get_height() / 2
                  - t_size / 2 + EPSILON; i += t_size) {
        world_out << "    <model name='cinder_block_boundary_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>"
            << (boundary->get_center()->x - boundary->get_width() / 2
                - t_size / 4) << " " << i << " 0 0 0 " << M_PI_2 << "</pose>\n";
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
        double x = c->x - t_size * (s / t_size / 2.0 - 1.0 / 2.0);
        int j = 1;
        for (int i = 1; i <= (int) (s / t_size); i++) {
          for (double y = c->y - t_size * (s / t_size / 2.0 - 1.0 / 4.0);
              y <= c->y + t_size * (s / t_size / 2.0 - 1.0 / 4.0) + EPSILON;
              y += t_size / 2.0) {
            world_out << "    <model name='cinder_block_obstacle_" << n << "_"
                << j << "'>\n";
            world_out << "      <include>\n";
            world_out << "        <uri>model://cinder_block</uri>\n";
            world_out << "      </include>\n";
            world_out << "      <pose>" << x << " " << y << " 0 0 0 0</pose>\n";
            world_out << "      <static>1</static>\n";
            world_out << "    </model>\n";
            j++;
          }
          x += t_size;
        }
        n++;
      }
    }
  }
  world_in.close();
  world_out.close();

  space = SpacePtr(new Space(boundary, obstacles));
  if (argc >= 5) {
    if (std::string(argv[4]) == "spiral_stc") {
      SpiralStcPtr plan_spiral_stc = SpiralStcPtr(new SpiralStc());
      plan_spiral_stc->initialize(starting_point, t_size);
      path.insert(path.end(), starting_point);
      plan_spiral_stc->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));
      plan_spiral_stc->set_behavior_see_obstacle(
          boost::bind(&test_see_obstacle, _1, _2));
      plan_spiral_stc->cover();
    } else if (std::string(argv[4]) == "full_spiral_stc") {
      FullSpiralStcPtr plan_full_spiral_stc = FullSpiralStcPtr(
          new FullSpiralStc());
      plan_full_spiral_stc->initialize(starting_point, t_size);
      path.insert(path.end(), starting_point);
      plan_full_spiral_stc->set_behavior_go_to(
          boost::bind(&test_go_to, _1, _2));
      plan_full_spiral_stc->set_behavior_see_obstacle(
          boost::bind(&test_see_obstacle, _1, _2));
      plan_full_spiral_stc->cover();
    } else if (std::string(argv[4]) == "boustrophedon_online") {
      BoustrophedonOnlinePtr plan_boustrophedon_online = BoustrophedonOnlinePtr(
          new BoustrophedonOnline());
      plan_boustrophedon_online->initialize(starting_point, t_size);
      path.insert(path.end(), starting_point);
      plan_boustrophedon_online->set_behavior_go_to(
          boost::bind(&test_go_to, _1, _2));
      plan_boustrophedon_online->set_behavior_see_obstacle(
          boost::bind(&test_see_obstacle, _1, _2));
      plan_boustrophedon_online->cover();
    }
  }
  run(argc, argv);
  return 0;
}
