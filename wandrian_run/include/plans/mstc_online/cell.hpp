/*
 * cell.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_CELL_HPP_

#include "../../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace mstc_online {

enum State {
  NEW, OLD, OBSTACLE
};

class Cell: public Polygon {

public:
  Cell(PointPtr, double);
  virtual ~Cell();
  PointPtr get_center() const;
  double get_size() const;
  boost::shared_ptr<Cell> get_parent();

  void set_parent(boost::shared_ptr<Cell>);

protected:
  PointPtr center;
  double size;

private:
  boost::shared_ptr<Cell> parent;
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

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_CELL_HPP_ */
