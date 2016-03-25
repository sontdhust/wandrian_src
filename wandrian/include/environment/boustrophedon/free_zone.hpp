/*
 * free_zone.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_FREE_ZONE_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_FREE_ZONE_HPP_

#include "../../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class FreeZone: public Polygon {

public:
  std::list<boost::shared_ptr<FreeZone> > children;
  bool status_visited;
  PointPtr point_backtrack;

  FreeZone(PointPtr, double, double);
  static bool compare_positions_x(boost::shared_ptr<FreeZone>,
      boost::shared_ptr<FreeZone>);
  static bool is_parent(boost::shared_ptr<FreeZone>,
      boost::shared_ptr<FreeZone>);

  PointPtr get_center();
  double get_size_x();
  double get_size_y();
  boost::shared_ptr<FreeZone> get_parent();
  void set_parent(boost::shared_ptr<FreeZone>);
  void set_point_backtrack(PointPtr);

private:
  PointPtr center;
  double size_x;
  double size_y;
  boost::shared_ptr<FreeZone> parent;
};

typedef boost::shared_ptr<FreeZone const> FreeZoneConstPtr;
typedef boost::shared_ptr<FreeZone> FreeZonePtr;

inline bool operator<(const FreeZone &c1, const FreeZone &c2) {
  FreeZonePtr Space1 = FreeZonePtr(new FreeZone(c1));
  FreeZonePtr Space2 = FreeZonePtr(new FreeZone(c2));
  return
      std::abs(Space1->get_center()->x - Space2->get_center()->x) > EPSILON ?
          (Space1->get_center()->x < Space2->get_center()->x) :
          (Space1->get_center()->y < Space2->get_center()->y);
}

struct SpaceComp {
  bool operator()(FreeZoneConstPtr c1, FreeZoneConstPtr c2) const {
    return *c1 < *c2;
  }
};

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_FREE_ZONE_HPP_ */
