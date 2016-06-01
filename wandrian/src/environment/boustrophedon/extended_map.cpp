/*
 * extended_map.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/environment/boustrophedon/extended_map.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

ExtendedMap::ExtendedMap(RectanglePtr boundary,
    std::list<RectanglePtr> obstacles) :
    Map(boundary, obstacles), number_space_need_visit(0) {
}

ExtendedMap::ExtendedMap(std::string map_path) :
    Map(map_path), number_space_need_visit(0) {
}

ExtendedMap::~ExtendedMap() {
}

void ExtendedMap::build() {
  std::string size;
  std::string position;
  std::string line;
  std::list<PointPtr> list_point_temp;
  PointPtr center, temp_point;
  double size_x, size_y;
  int i, flag;
  std::fstream read_file;
  std::size_t flag_start, flag_end;
  if (this->map_path.compare("") != 0) {
    read_file.open(this->map_path.c_str());
    if (read_file.is_open()) {
      int i = 1;
      int flag = 0;
      while (getline(read_file, line) != NULL) {
        flag = 0;
        position = "";
        size = "";
        flag_start = -1;
        while (line.find("(", flag_start + 1) < line.length()) {
          flag_start = line.find("(", flag_start + 1);
          flag_end = line.find(")", flag_start + 1);
          temp_point = create_point_from_string(
              line.substr(flag_start + 1, flag_end - flag_start - 1));
          list_point_temp.push_back(temp_point);
        }
        if (!this->boundary) {
          i = 0;
          for (std::list<PointPtr>::iterator u = list_point_temp.begin();
              u != list_point_temp.end(); ++u) {
            if (i == 0) {
              size_y = (*u)->y;
            }
            if (i == 1) {
              size_x = (*u)->x;
              size_y = (*u)->y - size_y;
            }
            if (i == 2) {
              size_x = (*u)->x - size_x;
            }
            i++;
            if (i == 4) {
              center = PointPtr(
                  new Point((*u)->x - size_x / 2, (*u)->y + size_y / 2));
            }
          }
          this->boundary = RectanglePtr(new Rectangle(center, size_x, size_y));
        } else {
          this->extended_obstacles.push_back(
              PolygonPtr(new Polygon(list_point_temp)));
        }
        list_point_temp.clear();
      }
    } else {
      std::cout << "Can't open file " << std::endl;
    }
    read_file.close();
  }
}

std::list<PolygonPtr> ExtendedMap::get_extended_obstacles() {
  return extended_obstacles;
}

int ExtendedMap::comma_position(std::string str) {
  for (unsigned int position = 0; position < str.length(); ++position) {
    if (str[position] == ',')
      return position;
  }
  return 0;
}

PointPtr ExtendedMap::create_point_from_string(std::string str) {
  int flag;
  flag = str.find(",");
  return PointPtr(
      new Point(strtod(str.substr(0, flag).c_str(), NULL),
          strtod(str.substr(flag + 1, str.length()).c_str(),
          NULL)));
}

}
}
}
