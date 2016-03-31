/*
 * map.hpp
 *
 *  Created on: Jul 28, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_MAP_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_MAP_HPP_

#include "../common/rectangle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {

class Map {

public:
  Map(RectanglePtr, std::list<RectanglePtr>);
  Map(std::string);
  virtual ~Map();

  RectanglePtr get_boundary();
  std::list<RectanglePtr> get_obstacles();
  std::string get_file_name();

protected:
  RectanglePtr boundary;
  std::list<RectanglePtr> obstacles;
  std::string file_name;

  virtual void build();

private:
  void initialize();
};

typedef boost::shared_ptr<Map> MapPtr;

}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_MAP_HPP_ */
