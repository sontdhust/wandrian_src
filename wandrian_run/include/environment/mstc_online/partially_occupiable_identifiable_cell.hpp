/*
 * partially_occupiable_identifiable_cell.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_PARTIALLY_OCCUPIABLE_IDENTIFIABLE_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_PARTIALLY_OCCUPIABLE_IDENTIFIABLE_CELL_HPP_

#include "identifiable_cell.hpp"
#include "../partially_occupiable.hpp"

namespace wandrian {
namespace environment {
namespace mstc_online {

class PartiallyOccupiableIdentifiableCell: public IdentifiableCell,
    public PartiallyOccupiable {

public:
  PartiallyOccupiableIdentifiableCell(PointPtr, double, std::string);
  ~PartiallyOccupiableIdentifiableCell();

  PointPtr get_current_position();
};

typedef boost::shared_ptr<PartiallyOccupiableIdentifiableCell> PartiallyOccupiableIdentifiableCellPtr;
typedef boost::shared_ptr<PartiallyOccupiableIdentifiableCell const> PartiallyOccupiableIdentifiableCellConstPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_PARTIALLY_OCCUPIABLE_IDENTIFIABLE_CELL_HPP_ */
