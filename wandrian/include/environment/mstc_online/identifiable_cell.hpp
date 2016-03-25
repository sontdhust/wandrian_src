/*
 * identifiable_cell.hpp
 *
 *  Created on: Mar 11, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_IDENTIFIABLE_CELL_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_IDENTIFIABLE_CELL_HPP_

#include "../../environment/cell.hpp"
#include "../../environment/mstc_online/identifiable.hpp"

namespace wandrian {
namespace environment {
namespace mstc_online {

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

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_ONLINE_IDENTIFIABLE_CELL_HPP_ */
