/*
 * partially_occupiable.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_HPP_

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

class PartiallyOccupiable {

public:
  PartiallyOccupiable();
  ~PartiallyOccupiable();

  Quadrant get_current_quadrant();
  State* get_quadrants();
  void set_current_quadrant(Quadrant);
  void set_quadrants_state(Quadrant, State);

protected:
  PointPtr current_position(PointPtr, double);

private:
  Quadrant current_quadrant;
  State quadrants[4];
};

}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_HPP_ */
