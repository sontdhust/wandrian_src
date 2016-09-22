/*
 * partially_occupiable.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_HPP_

#include "../common/vector.hpp"
#include "cell.hpp"

namespace wandrian {
namespace environment {

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

inline Quadrant operator&(Orientation o) {
  switch (o) {
  case AT_RIGHT_SIDE:
    return IV;
  case IN_FRONT:
    return I;
  case AT_LEFT_SIDE:
    return II;
  case IN_BACK:
    return III;
  }
  return I;
}

class PartiallyOccupiable {

public:
  PartiallyOccupiable();
  virtual ~PartiallyOccupiable();
  PointPtr find_position(Quadrant);

  PointPtr get_current_position();
  Quadrant get_current_quadrant();
  State *get_quadrants();
  void set_current_quadrant(Quadrant);
  void set_quadrants_state(Quadrant, State);

protected:
  virtual PointPtr _center() = 0;
  virtual double _size() = 0;

private:
  Quadrant current_quadrant;
  State quadrants[4];
};
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_HPP_ */
