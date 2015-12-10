/*
 * square.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_

#include "../../common/polygon.hpp"

#define OLD true
#define NEW false

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

enum Quadrant {
  //  _____________
  // |      |      |
  // |  II  |  I   |
  // |______|______|
  // |      |      |
  // | III  |  IV  |
  // |______|______|

  I,
  II,
  III,
  IV
};

inline Quadrant operator+(Quadrant q) {
  switch (q) {
  case I:
    return II;
  case II:
    return III;
  case III:
    return IV;
  case IV:
    return I;
  }
  return q;
}

inline Quadrant operator++(Quadrant &q) {
  q = +q;
  return q;
}

class Cell: public Polygon {

public:
  Cell(PointPtr, double);
  PointPtr get_center() const;
  double get_size() const;
  boost::shared_ptr<Cell> get_parent();
  PointPtr get_current_position();
  Quadrant get_current_quadrant();
  bool* get_quadrants();

  void set_parent(boost::shared_ptr<Cell>);
  void set_current_quadrant(Quadrant);

private:
  PointPtr center;
  double size;
  boost::shared_ptr<Cell> parent;
  Quadrant current_quadrant;
  bool quadrants[4];
};

typedef boost::shared_ptr<Cell> CellPtr;
typedef boost::shared_ptr<Cell const> CellConstPtr;

inline bool operator<(CellConstPtr c1, CellConstPtr c2) {
  // TODO: Choose relevant epsilon value
  double EPS = 20 * std::numeric_limits<double>::epsilon();
  return
      std::abs(c1->get_center()->x - c2->get_center()->x) > EPS ?
          (c1->get_center()->x < c2->get_center()->x) :
          (std::abs(c1->get_center()->y - c2->get_center()->y) > EPS ?
              (c1->get_center()->y < c2->get_center()->y) :
              c1->get_size() < c2->get_size());
}

inline bool operator!=(CellPtr c1, CellPtr c2) {
  return c1 < c2 || c2 < c1 || c1->get_size() != c2->get_size();
}

inline bool operator==(CellPtr c1, CellPtr c2) {
  return !(c1 != c2);
}

struct CellComp {
  bool operator()(CellConstPtr c1, CellConstPtr c2) const {
    return c1 < c2;
  }
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_ */
