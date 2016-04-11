/*
 * map.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#include <vector>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include "../../include/environment/cell.hpp"
#include "../../include/environment/map.hpp"

namespace wandrian {
namespace environment {

Map::Map(RectanglePtr boundary, std::list<RectanglePtr> obstacles) :
    boundary(boundary), obstacles(obstacles) {
}

Map::Map(std::string map_path) {
  this->map_path = map_path;
}

Map::~Map() {
}

RectanglePtr Map::get_boundary() {
  return boundary;
}

std::list<RectanglePtr> Map::get_obstacles() {
  return obstacles;
}

std::string Map::get_map_path() {
  return map_path;
}

void Map::build() {
  // Read center point of obstacles from input file
  std::vector<PointPtr> obstacle_centers;
  std::vector<double> obstacle_sizes;
  bool global_obstacle_size = true;
  double o_size;
  std::ifstream map(map_path.c_str());
  std::string line;
  // Boundary size
  std::getline(map, line, '\n');
  int delim_pos1;
  int delim_pos2;
  int delim_pos3;
  delim_pos1 = line.find(" ");
  delim_pos2 = line.find(" ", delim_pos1 + 1);
  double center_x = boost::lexical_cast<double>(
      boost::lexical_cast<double>(line.substr(0, delim_pos1)));
  double center_y = boost::lexical_cast<double>(
      boost::lexical_cast<double>(
          line.substr(delim_pos1 + 1, delim_pos2 - delim_pos1 - 1)));
  delim_pos1 = delim_pos2;
  delim_pos2 = line.find(" ", delim_pos1 + 1);
  delim_pos3 = line.find(" ", delim_pos2 + 1);
  global_obstacle_size = (delim_pos3 != std::string::npos);
  double width = boost::lexical_cast<double>(
      boost::lexical_cast<double>(
          line.substr(delim_pos1 + 1, delim_pos2 - delim_pos1 - 1)));
  double height = boost::lexical_cast<double>(
      boost::lexical_cast<double>(
          line.substr(delim_pos2 + 1,
              (global_obstacle_size ? delim_pos3 : line.length()) - delim_pos2
                  - 1)));
  boundary = RectanglePtr(
      new Rectangle(PointPtr(new Point(center_x, center_y)), width, height));
  if (global_obstacle_size)
    o_size = boost::lexical_cast<double>(
        boost::lexical_cast<double>(
            line.substr(delim_pos3 + 1, line.length() - delim_pos3 - 1)));
  // Center point of obstacles
  while (std::getline(map, line, '\n')) {
    if (line == "" || line.substr(0, 1) == "#")
      continue;
    double center_point_x;
    double center_point_y;
    int d_pos1 = line.find(" ");
    int d_pos2 = line.find(" ", d_pos1 + 1);
    center_point_x = boost::lexical_cast<double>(line.substr(0, d_pos1));
    center_point_y = boost::lexical_cast<double>(
        line.substr(d_pos1 + 1,
            (!global_obstacle_size ? d_pos2 : line.length()) - d_pos1 - 1));
    obstacle_centers.push_back(
        PointPtr(new Point(center_point_x, center_point_y)));
    if (!global_obstacle_size) {
      obstacle_sizes.push_back(
          boost::lexical_cast<double>(
              line.substr(d_pos2 + 1, line.length() - d_pos2 - 1)));
    }
  }

  // Generate obstacles
  for (int i = 0; i <= ((int) obstacle_centers.size() - 1); i++) {
    PointPtr center = obstacle_centers[i];
    bool valid = true;
    for (std::list<RectanglePtr>::iterator o = obstacles.begin();
        o != obstacles.end(); o++)
      if ((boost::static_pointer_cast<Cell>(*o))->get_center() == center) {
        valid = false;
        break;
      };
    if (valid) {
      obstacles.insert(obstacles.end(),
          CellPtr(
              new Cell(center,
                  global_obstacle_size ? o_size : obstacle_sizes[i])));
    }
  }
}

}
}
