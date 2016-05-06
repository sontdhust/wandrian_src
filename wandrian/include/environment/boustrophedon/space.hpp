/*
 * space.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_SPACE_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_SPACE_HPP_

#include "../../common/polygon.hpp"
#include "vertices.hpp"
using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Space: public Polygon {

public:
  Space(std::list<PointPtr>);
  std::list<boost::shared_ptr<Space> > children;
  bool status_visited;
  PointPtr point_backtrack;
  void print_list_space(std::list< boost::shared_ptr<Space> >);
  static bool compare_positions_x(boost::shared_ptr<Space>,
      boost::shared_ptr<Space>);
  static bool is_parent(boost::shared_ptr<Space>, boost::shared_ptr<Space>);
  boost::shared_ptr<Space> get_parent();
  void set_parent(boost::shared_ptr<Space>);
  void set_point_backtrack(boost::shared_ptr<Space>, boost::shared_ptr<Space>,
		  double robot_size);
  VerticesPtr get_vertices_upper();
  void set_vertices_upper(VerticesPtr);
  VerticesPtr get_vertices_below();
  void set_vertices_below(VerticesPtr);
private:
  VerticesPtr vertices_below, vertices_upper;
  boost::shared_ptr<Space> parent;
};

typedef boost::shared_ptr<Space const> SpaceConstPtr;
typedef boost::shared_ptr<Space> SpacePtr;

inline bool operator<(const Space &c1, const Space &c2) {
  SpacePtr Space1 = SpacePtr(new Space(c1));
  SpacePtr Space2 = SpacePtr(new Space(c2));
  return true;
//      std::abs(Space1->get_center()->x - Space2->get_center()->x) > EPSILON ?
//          (Space1->get_center()->x < Space2->get_center()->x) :
//          (Space1->get_center()->y < Space2->get_center()->y);
}

struct SpaceComp {
  bool operator()(SpaceConstPtr c1, SpaceConstPtr c2) const {
    return *c1 < *c2;
  }
};

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_SPACE_HPP_ */
