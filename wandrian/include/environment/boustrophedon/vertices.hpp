/*
 * vertices.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_

#include "../../common/rectangle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Vertices {

public:
  Vertices(PointPtr, RectanglePtr);
  bool left_compared_center();
  bool upon_compared_center();

  static bool compare_vertices(boost::shared_ptr<Vertices>, boost::shared_ptr<Vertices>);
  static bool compare_positions_x(boost::shared_ptr<Vertices>, boost::shared_ptr<Vertices>);
  static bool compare_positions_y(boost::shared_ptr<Vertices>, boost::shared_ptr<Vertices>);

  PointPtr get_position();
  RectanglePtr get_polygon();
  void set_positions(PointPtr);
  void set_polygon(RectanglePtr);

private:
  PointPtr position;
  RectanglePtr polygon;
};

typedef boost::shared_ptr<Vertices const> VerticesConstPtr;
typedef boost::shared_ptr<Vertices> VerticesPtr;
}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_ */
