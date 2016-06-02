/*
 * segment.hpp
 *
 *  Created on: Jun 23, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_COMMON_SEGMENT_HPP_
#define WANDRIAN_INCLUDE_COMMON_SEGMENT_HPP_

#include "point.hpp"

namespace wandrian {
namespace common {

struct Segment {

  PointPtr p1, p2;

  Segment(PointPtr, PointPtr);
  Segment(double, double, double, double);
  ~Segment();

private:
  void construct(PointPtr, PointPtr);
};

typedef boost::shared_ptr<Segment> SegmentPtr;
typedef boost::shared_ptr<Segment const> SegmentConstPtr;

/**
 * Find intersect between 2 segments
 */
inline PointPtr operator%(SegmentPtr s1, SegmentPtr s2) {
  double dx;
  double dy;

  dx = s1->p2->x - s1->p1->x;
  dy = s1->p2->y - s1->p1->y;
  double am = dy / dx;
  double ac = s1->p1->y - am * s1->p1->x;
  // Equation of line a: y = am * x + ac

  dx = s2->p2->x - s2->p1->x;
  dy = s2->p2->y - s2->p1->y;
  double bm = dy / dx;
  double bc = s2->p1->y - bm * s2->p1->x;
  // Equation of line b: y = bm * x + bc

  double INF = std::numeric_limits<double>::infinity();
  if ((am == INF && bm == INF) || std::abs(am - bm) < SMALL_EPSILON) // Line a is parallel to line b
    return PointPtr();
  else {
    double xi;
    double yi;
    if (am == INF) { // Line a is parallel to vertical axis, but not b
      xi = s1->p1->x; // = a.p2->x
      yi = bm * xi + bc;
    } else if (bm == INF) { // Line b is parallel to vertical axis, but not a
      xi = s2->p1->x; // = b.p2->x
      yi = am * xi + ac;
    } else { // Both are not parallel to vertical axis
      xi = (bc - ac) / (am - bm);
      yi = am * xi + ac; // = bm * xi + bc
    }
    if ((s1->p1->x - xi) * (s1->p2->x - xi) <= EPSILON
        && (s2->p1->x - xi) * (s2->p2->x - xi) <= EPSILON
        && (s1->p1->y - yi) * (s1->p2->y - yi) <= EPSILON
        && (s2->p1->y - yi) * (s2->p2->y - yi) <= EPSILON)
      return PointPtr(new Point(xi, yi));
    else
      return PointPtr();
  }
}

inline bool operator==(SegmentPtr s1, SegmentPtr s2) {
  return (((s1->p1 == s2->p1) && (s1->p2 == s2->p2))
      || ((s1->p1 == s2->p2) && (s1->p2 == s2->p1)));
}

inline bool operator<(SegmentConstPtr s1, SegmentConstPtr s2) {
  return s1->p1 != s2->p1 ? s1->p1 < s2->p1 : s1->p2 < s2->p2;
}

struct SegmentComp {
  bool operator()(SegmentConstPtr s1, SegmentConstPtr s2) const {
    return s1 < s2;
  }
};

}
}

#endif /* WANDRIAN_INCLUDE_COMMON_SEGMENT_HPP_ */
