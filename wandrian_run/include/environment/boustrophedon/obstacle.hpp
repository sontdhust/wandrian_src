#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_OBSTACLE_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_OBSTACLE_HPP_

#include "../../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Obstacle: public Polygon {

public:
  Obstacle(PointPtr, double, double);

  PointPtr get_center();
  double get_size_x();
  double get_size_y();

private:
  PointPtr center;
  double size_x;
  double size_y;

};

typedef boost::shared_ptr<Obstacle const> ObstacleConstPtr;
typedef boost::shared_ptr<Obstacle> ObstaclePtr;

inline bool operator<(const Obstacle &c1, const Obstacle &c2) {
  ObstaclePtr Obstacle1 = ObstaclePtr(new Obstacle(c1));
  ObstaclePtr Obstacle2 = ObstaclePtr(new Obstacle(c2));
  return
      std::abs(Obstacle1->get_center()->x - Obstacle2->get_center()->x)
          > EPSILON ?
          (Obstacle1->get_center()->x < Obstacle2->get_center()->x) :
          (Obstacle1->get_center()->y < Obstacle2->get_center()->y);
}

struct ObstacleComp {
  bool operator()(ObstacleConstPtr c1, ObstacleConstPtr c2) const {
    return *c1 < *c2;
  }
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_Obstacle_HPP_ */
