/*
 * partially_occupiable_cell.hpp
 *
 *  Created on: Dec 12, 2015
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_PARTIALLY_OCCUPIABLE_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_PARTIALLY_OCCUPIABLE_CELL_HPP_

#include "cell.hpp"

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

inline Quadrant operator-(Quadrant q) {
  switch (q) {
  case I:
    return IV;
  case IV:
    return III;
  case III:
    return II;
  case II:
    return I;
  }
  return q;
}

inline Quadrant operator--(Quadrant &q) {
  q = -q;
  return q;
}

class PartiallyOccupiableCell: public Cell {

public:
  PartiallyOccupiableCell(PointPtr, double);
  ~PartiallyOccupiableCell();

  PointPtr get_current_position();
  Quadrant get_current_quadrant();
  State* get_quadrants();
  void set_current_quadrant(Quadrant);
  void set_quadrants_state(Quadrant, State);

private:
  Quadrant current_quadrant;
  State quadrants[4];
};

typedef boost::shared_ptr<PartiallyOccupiableCell> PartiallyOccupiableCellPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_PARTIALLY_OCCUPIABLE_CELL_HPP_ */
