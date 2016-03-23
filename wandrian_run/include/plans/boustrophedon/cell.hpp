/*
 * square.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_CELL_HPP_

#include "../../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace boustrophedon{

#define OLD_CELL false
#define NEW_CELL true

class Cell: public Polygon {

public:
  std::list<boost::shared_ptr<Cell> > neighbors;

  Cell(PointPtr, double);
  PointPtr get_center();
  double get_size();
  void set_parent(boost::shared_ptr<Cell>);
  boost::shared_ptr<Cell> get_parent();

private:
  PointPtr center;
  double size;
  boost::shared_ptr<Cell> parent;
};

typedef boost::shared_ptr<Cell const> CellConstPtr;
typedef boost::shared_ptr<Cell> CellPtr;

inline bool operator<(const Cell &c1, const Cell &c2) {
  // TODO Choose relevant epsilon value
  double EPS = 20 * std::numeric_limits<double>::epsilon();
  CellPtr cell1 = CellPtr(new Cell(c1));
  CellPtr cell2 = CellPtr(new Cell(c2));
  return
      std::abs(cell1->get_center()->x - cell2->get_center()->x) > EPS ?
          (cell1->get_center()->x < cell2->get_center()->x) :
          (cell1->get_center()->y < cell2->get_center()->y);
}

struct CellComp {
  bool operator()(CellConstPtr c1, CellConstPtr c2) const {
    return *c1 < *c2;
  }
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_CELL_HPP_ */
