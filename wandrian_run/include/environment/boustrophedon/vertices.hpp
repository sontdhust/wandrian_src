#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_

#include "obstacle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Vertices {

public:
  Vertices(PointPtr, ObstaclePtr);
  bool left_compared_center();
  bool upon_compared_center();

  static bool compare_vertices(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);
  static bool compare_positions_x(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);
  static bool compare_positions_y(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);

  PointPtr get_position();
  ObstaclePtr get_polygon();
  void set_positions(PointPtr);
  void set_polygon(ObstaclePtr);

private:
  PointPtr position;
  ObstaclePtr polygon;
};

typedef boost::shared_ptr<Vertices const> VerticesConstPtr;
typedef boost::shared_ptr<Vertices> VerticesPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_ */

