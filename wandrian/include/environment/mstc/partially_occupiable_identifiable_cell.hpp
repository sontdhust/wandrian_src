/*
 * partially_occupiable_identifiable_cell.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_PARTIALLY_OCCUPIABLE_IDENTIFIABLE_CELL_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_PARTIALLY_OCCUPIABLE_IDENTIFIABLE_CELL_HPP_

#include "../partially_occupiable.hpp"
#include "identifiable_cell.hpp"

namespace wandrian {
namespace environment {
namespace mstc {

class PartiallyOccupiableIdentifiableCell : public IdentifiableCell, public PartiallyOccupiable {

public:
  PartiallyOccupiableIdentifiableCell(PointPtr, double, std::string);
  ~PartiallyOccupiableIdentifiableCell();

  PointPtr _center();
  double _size();
};

typedef boost::shared_ptr<PartiallyOccupiableIdentifiableCell> PartiallyOccupiableIdentifiableCellPtr;
typedef boost::shared_ptr<PartiallyOccupiableIdentifiableCell const> PartiallyOccupiableIdentifiableCellConstPtr;
}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_PARTIALLY_OCCUPIABLE_IDENTIFIABLE_CELL_HPP_ */
