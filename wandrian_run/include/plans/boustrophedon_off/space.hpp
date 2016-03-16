

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_Space_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_Space_HPP_

#include "../../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace boustrophedon_off {



class Space: public Polygon {
public:
  std::list<boost::shared_ptr<Space> > children;

  Space(PointPtr, double, double);
  PointPtr get_center();
  double get_sizex();
  double get_sizey();
  bool status_visited;
  PointPtr point_backtrack;
  void set_point_backtrack(PointPtr);
  void set_parent(boost::shared_ptr<Space>);
  boost::shared_ptr<Space> get_parent();
  static bool compare_positionsx(boost::shared_ptr<Space>, boost::shared_ptr<Space>);
  static bool is_parent(boost::shared_ptr<Space>, boost::shared_ptr<Space>);

private:
  PointPtr center;
  double sizex;
  double sizey;
  boost::shared_ptr<Space> parent;
};

typedef boost::shared_ptr<Space const> SpaceConstPtr;
typedef boost::shared_ptr<Space> SpacePtr;

inline bool operator<(const Space &c1, const Space &c2) {
  // TODO Choose relevant epsilon value
  double EPS = 20 * std::numeric_limits<double>::epsilon();
  SpacePtr Space1 = SpacePtr(new Space(c1));
  SpacePtr Space2 = SpacePtr(new Space(c2));
  return
      std::abs(Space1->get_center()->x - Space2->get_center()->x) > EPS ?
          (Space1->get_center()->x < Space2->get_center()->x) :
          (Space1->get_center()->y < Space2->get_center()->y);
}

struct SpaceComp {
  bool operator()(SpaceConstPtr c1, SpaceConstPtr c2) const {
    return *c1 < *c2;
  }
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_Space_HPP_ */
