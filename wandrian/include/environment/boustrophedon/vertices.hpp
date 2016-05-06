/*
 * vertices.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_

#include "../../common/rectangle.hpp"
#include "../../common/segment.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Vertices {

public:
  Vertices(PointPtr ,PointPtr ,PointPtr , bool);
  bool left_compared_center();
  bool upon_compared_center();
  void set_is_of_obstacles(bool);
  static bool compare_vertices(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);
  static bool compare_positions_x(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);
  static bool compare_positions_y(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);

  PointPtr get_position();
  PolygonPtr get_polygon();
  void set_positions(PointPtr);
  void set_polygon(RectanglePtr);

  PointPtr get_below_point();
  void set_below_point(PointPtr below_point);
  PointPtr get_upper_point();
  void set_upper_point(PointPtr uppper_point);
  int type_vertice;
  bool is_of_obstacles;
  bool is_obstacles_upper;
  bool is_obstacles_below;
  void set_is_obstacles_upper(PolygonPtr , double);
  void set_is_obstacles_below(PolygonPtr , double);
  void set_type_vertices();
private:
  PointPtr position;
  PointPtr upper_point;
  PointPtr below_point;
};

typedef boost::shared_ptr<Vertices const> VerticesConstPtr;
typedef boost::shared_ptr<Vertices> VerticesPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_ */
