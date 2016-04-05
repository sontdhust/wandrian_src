/*
 * polygon.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_COMMON_POLYGON_HPP_
#define WANDRIAN_INCLUDE_COMMON_POLYGON_HPP_

#include <map>
#include <set>
#include <list>
#include "point.hpp"

namespace wandrian {
namespace common {

class Polygon {

public:
  Polygon();
  Polygon(std::list<PointPtr>);
  ~Polygon();
  std::list<PointPtr> get_points();
  std::list<PointPtr> get_boundary();

protected:
  std::list<PointPtr> points;

  void build();

private:
  std::map<PointPtr, std::set<PointPtr, PointComp>, PointComp> graph;

  PointPtr get_leftmost();
  PointPtr get_rightmost();
  std::list<PointPtr> get_upper_boundary(); // List of points
  std::list<PointPtr> get_lower_boundary(); // List of points
  std::list<PointPtr> get_partial_boundary(bool); // List of points
};

typedef boost::shared_ptr<Polygon> PolygonPtr;

}
}

#endif /* WANDRIAN_INCLUDE_COMMON_POLYGON_HPP_ */
