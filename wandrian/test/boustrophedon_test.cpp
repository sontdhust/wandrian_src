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
#include "../include/environment/map.hpp"
#include "../include/plans/boustrophedon/boustrophedon.hpp"

#define R_SIZE 0.4 // robot size
#define E_SIZE 6.0 // default environment size
#define WORLD_INSERT_OBSTACLE "<!-- INSERT: Boundary and Obstacles here -->" // flag at original world file to insert bound and obstacles into

using namespace wandrian::environment;
using namespace wandrian::plans::boustrophedon;

double e_size = 0;
double r_size = 0;
MapPtr map;
PointPtr starting_point;
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
  std::list<RectanglePtr> obstacles = map->get_obstacles();
  for (std::list<RectanglePtr>::iterator obstacle = obstacles.begin();
      obstacle != obstacles.end(); obstacle++) {
    draw((*obstacle)->get_boundary(), GL_POLYGON);
  }

  // Starting point
  glPointSize(4);
  glColor3ub(0, 255, 0);
  glBegin(GL_POINTS);
  glVertex2d(starting_point->x, starting_point->y);
  glEnd();

  // Boustrophedon covering path
  glColor3ub(0, 255, 0);
  draw_path(tmp_path, GL_LINE_STRIP);

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

bool test_see_obstacle(VectorPtr direction, double step) {
  // Simulator check obstacle
  PointPtr last_position = *(--tmp_path.end());
  PointPtr new_position = PointPtr(
      new Point(last_position + direction * step * r_size / 2));
  if (map) {
    CellPtr space = boost::static_pointer_cast<Cell>(map->get_boundary());
    if (new_position->x >= space->get_center()->x + space->get_size() / 2
        || new_position->x <= space->get_center()->x - space->get_size() / 2
        || new_position->y >= space->get_center()->y + space->get_size() / 2
        || new_position->y <= space->get_center()->y - space->get_size() / 2) {
      return true;
    }
    std::list<RectanglePtr> obstacles = map->get_obstacles();
    for (std::list<RectanglePtr>::iterator o = obstacles.begin();
        o != obstacles.end(); o++) {
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
	//TODO: edit boustrophedon
  if (argc >= 2) {
    std::istringstream iss(argv[1]);
    if (!(iss >> e_size)
        || !(4 <= e_size && e_size <= 20 && (int) e_size % 2 == 0)) {
      e_size = E_SIZE;
    }
  } else {
    e_size = E_SIZE;
  }
  std::cout << "ROBOT_SIZE:1"<< r_size <<std::endl;
  if(argc >= 3){
  	std::istringstream iss(argv[2]);
  	if(!(iss >> r_size)){
  		std::cout << "ROBOT_SIZE2:"<< r_size <<std::endl;
  		r_size = R_SIZE;
  	}
  	std::cout << "ROBOT_SIZE3:"<< r_size <<std::endl;
  }else{
	  std::cout << "ROBOT_SIZE4:"<< r_size <<std::endl;
	  r_size = R_SIZE;
  }

  BoustrophedonPtr boustrophedon = BoustrophedonPtr(new Boustrophedon());
  boustrophedon->initialize(starting_point, r_size,
      "../../worlds/prefered_boustrophedon.map");
  map = boustrophedon->get_map();

  CellPtr space = CellPtr(new Cell(PointPtr(new Point(0, 0)), e_size));
  std::list<RectanglePtr> obstacles;
  std::srand(std::time(0));
  starting_point = PointPtr(
      new Point((map->get_boundary()->get_center()->x * 2 - map->get_boundary()->get_width()  + r_size) / 2,
    		  	(map->get_boundary()->get_center()->y *2 - map->get_boundary()->get_height() + r_size) / 2));
  int r = std::rand() % (int) (e_size * e_size / 16) + e_size * e_size / 8;

  std::ifstream world_in("../../worlds/empty.world");
  std::ofstream world_out("../../worlds/tmp.world");
  std::string line;

  obstacles = map->get_obstacles();

  while (std::getline(world_in, line, '\n')) {
    world_out << line << '\n';
    if (line.find(WORLD_INSERT_OBSTACLE) != std::string::npos) {
      int n;
      n = 1;
      // Upper bound
      for (double i = -e_size / 2 + r_size / 2; i <= e_size / 2 - r_size / 2;
          i += r_size) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << i << " " << (e_size / 2 + r_size / 4)
            << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      // Right bound
      for (double i = -e_size / 2 + r_size / 2; i <= e_size / 2 - r_size / 2;
          i += r_size) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << (e_size / 2 + r_size / 4) << " " << -i
            << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      // Lower bound
      for (double i = -e_size / 2 + r_size / 2; i <= e_size / 2 - r_size / 2;
          i += r_size) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << -i << " " << -(e_size / 2 + r_size / 4)
            << " 0 0 0 0</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      // Left bound
      for (double i = -e_size / 2 + r_size / 2; i <= e_size / 2 - r_size / 2;
          i += r_size) {
        world_out << "    <model name='cinder_block_bound_" << n << "'>\n";
        world_out << "      <include>\n";
        world_out << "        <uri>model://cinder_block</uri>\n";
        world_out << "      </include>\n";
        world_out << "      <pose>" << -(e_size / 2 + r_size / 4) << " " << i
            << " 0 0 0 " << M_PI_2 << "</pose>\n";
        world_out << "      <static>1</static>\n";
        world_out << "    </model>\n";
        n++;
      }

      n = 1;
//      double o_size = 4 * r_size;
      // Obstacles
      for (std::list<RectanglePtr>::iterator o = obstacles.begin();
          o != obstacles.end(); o++) {

        PointPtr p = (boost::static_pointer_cast<Rectangle>(*o))->get_center();
        double h = (boost::static_pointer_cast<Rectangle>(*o))->get_height();
        double w = (boost::static_pointer_cast<Rectangle>(*o))->get_width();
        int c = 1;

        // double x = p->x - r_size * (w / r_size / 2.0 - 1.0 / 2.0);
        double x = p->x - w / 2 + r_size / 2;

        for (int i = 1; i <= (int) (w / r_size); i++) {

          for (double y = p->y - r_size * (h / r_size / 2.0 - 1.0 / 4.0);
              y <= p->y + r_size * (h / r_size / 2.0 - 1.0 / 4.0);
              y += r_size / 2.0) {

            world_out << "    <model name='cinder_block_obstacle_" << n << "_"
                << c << "'>\n";
            world_out << "      <include>\n";
            world_out << "        <uri>model://cinder_block</uri>\n";
            world_out << "      </include>\n";
            world_out << "      <pose>" << x << " " << y << " 0 0 0 0</pose>\n";
            world_out << "      <static>1</static>\n";
            world_out << "    </model>\n";
            c++;

          }
          x += r_size;
        }
        n++;
      }
    }
  }
  world_in.close();
  world_out.close();

  //tmp_path.insert(tmp_path.end(), starting_point);
  boustrophedon->set_behavior_go_to(boost::bind(&test_go_to, _1, _2));

  boustrophedon->cover();

  run(argc, argv);
  return 0;
}
