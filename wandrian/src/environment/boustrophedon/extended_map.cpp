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
    Map(boundary, obstacles) {
}

ExtendedMap::ExtendedMap(std::string file_name) :
    Map(file_name) {
  build();
}

ExtendedMap::~ExtendedMap() {
}

void ExtendedMap::build() {
  std::string size;
  std::string position;
  std::string line;

  PointPtr center;
  double size_x, size_y;
  int i, flag;
  std::fstream myReadFile;
  if (this->map_path.compare("") != 0) {
    myReadFile.open(this->map_path.c_str());
    std::cout << "out" << this->map_path << std::endl;
    if (myReadFile.is_open()) {
      std::cout << "Out2" << this->map_path << std::endl;
      int i = 0;
      int flag = 0;
      while (getline(myReadFile, line) != NULL) {
        i = 0;
        flag = 0;
        position = "";
        size = "";
        for (unsigned int var = 0; var < line.length(); ++var) {
          if (line[var] == '(') {
            if (position.compare("") == 0)
              i = 1;
            else
              i = 2;
            flag = var + 1;
            continue;
          }
          if (line[var] == ')') {
            i = 0;
            continue;
          }
          if (i == 1) {
            position.insert(var - flag, 1, line[var]);
          }
          if (i == 2) {
            size.insert(var - flag, 1, line[var]);
          }
        }
        flag = 0;
        std::cout << "Position: " << position << std::endl;
        std::cout << "Size: " << size << std::endl;
        for (unsigned int var = 0; var < position.length(); ++var) {
          if (position[var] == ',') {
            flag = var;
            break;
          }
        }
        flag = comma_position(position);
        center = PointPtr(
            new Point(strtod(position.substr(0, flag).c_str(), NULL),
                strtod(position.substr(flag + 1, position.length()).c_str(),
                NULL)));

        flag = comma_position(size);
        size_x = strtod(size.substr(0, flag).c_str(), NULL);
        size_y = strtod(size.substr(flag + 1, size.length()).c_str(), NULL);
        std::cout << "Position (" << center->x << " ," << center->y << " )"
            << std::endl;
        std::cout << "Size : x =" << size_x << " y = " << size_y << std::endl;
        if (!this->boundary) {
          this->boundary = RectanglePtr(new Rectangle(center, size_x, size_y));
          std::cout << "ADD en " << std::endl;
        } else {
          this->obstacles.push_back(
              RectanglePtr(new Rectangle(center, size_x, size_y)));
          std::cout << "ADD obstacles " << std::endl;
        }
      }
    } else {
      std::cout << "Can't open file " << std::endl;
    }

    myReadFile.close();
  }
}

int ExtendedMap::comma_position(std::string str) {
  for (unsigned int position = 0; position < str.length(); ++position) {
    if (str[position] == ',')
      return position;
  }
  return 0;
}

}
}
}
