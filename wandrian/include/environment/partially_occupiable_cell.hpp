/*
 * partially_occupiable_cell.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_CELL_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_CELL_HPP_

#include "cell.hpp"
#include "partially_occupiable.hpp"

namespace wandrian {
namespace environment {

class PartiallyOccupiableCell : public Cell, public PartiallyOccupiable {

public:
  PartiallyOccupiableCell(PointPtr, double);
  ~PartiallyOccupiableCell();

  PointPtr _center();
  double _size();
};

typedef boost::shared_ptr<PartiallyOccupiableCell> PartiallyOccupiableCellPtr;
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_PARTIALLY_OCCUPIABLE_CELL_HPP_ */
