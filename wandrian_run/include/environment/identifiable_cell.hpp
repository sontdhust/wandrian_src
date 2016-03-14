/*
 * identifiable_cell.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_IDENTIFIABLE_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_IDENTIFIABLE_CELL_HPP_

#include "cell.hpp"
#include "identifiable.hpp"

namespace wandrian {
namespace environment {

class IdentifiableCell: public Cell, public Identifiable {

public:
  IdentifiableCell(PointPtr, double, std::string);
  ~IdentifiableCell();
};

typedef boost::shared_ptr<IdentifiableCell> IdentifiableCellPtr;
typedef boost::shared_ptr<IdentifiableCell const> IdentifiableCellConstPtr;

}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_IDENTIFIABLE_CELL_HPP_ */
