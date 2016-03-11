/*
 * point.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_

#include <stdlib.h>
#include <cmath>
#include <limits>
#include <boost/shared_ptr.hpp>

namespace wandrian {
namespace common {

struct Point {

  double x, y;

  Point();
  Point(double, double);
  Point(const Point&);
  Point(const boost::shared_ptr<Point>);
};

typedef boost::shared_ptr<Point> PointPtr;
typedef boost::shared_ptr<Point const> PointConstPtr;

inline double operator%(PointPtr p1, PointPtr p2) {
  return sqrt(std::pow(p1->x - p2->x, 2) + std::pow(p1->y - p2->y, 2));
}

inline bool operator<(PointConstPtr p1, PointConstPtr p2) {
  // TODO: Choose relevant epsilon value
  double EPS = 20 * std::numeric_limits<double>::epsilon();
  return
      std::abs(p1->x - p2->x) > EPS ?
          p1->x - p2->x < -EPS : p1->y - p2->y < -EPS;
}

inline bool operator!=(PointPtr p1, PointPtr p2) {
  return p1 < p2 || p2 < p1;
}

inline bool operator==(PointPtr p1, PointPtr p2) {
  return !(p1 != p2);
}

inline bool operator>(PointPtr p1, PointPtr p2) {
  return p1 != p2 && !(p1 < p2);
}

struct PointComp {
  bool operator()(PointConstPtr p1, PointConstPtr p2) const {
    return p1 < p2;
  }
};

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_POINT_HPP_ */
