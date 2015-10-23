/*
 * square.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_

#include "../../common/polygon.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace spiral_stc {

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

typedef boost::shared_ptr<Cell> CellPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_SPIRAL_STC_CELL_HPP_ */
