/*
 * identifiable_cell.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_IDENTIFIABLE_CELL_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_IDENTIFIABLE_CELL_HPP_

#include "../cell.hpp"
#include "identifiable.hpp"

namespace wandrian {
namespace environment {
namespace mstc {

class IdentifiableCell: public Cell, public Identifiable {

public:
  IdentifiableCell(PointPtr, double, std::string);
  ~IdentifiableCell();
};

typedef boost::shared_ptr<IdentifiableCell> IdentifiableCellPtr;
typedef boost::shared_ptr<IdentifiableCell const> IdentifiableCellConstPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_IDENTIFIABLE_CELL_HPP_ */
