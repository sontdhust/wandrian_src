/*
 * old_cell.hpp
 *
 *  Created on: Feb 19, 2016
 *      Author: manhnh
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_OLD_CELL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_OLD_CELL_HPP_

#include "cell.hpp"

namespace wandrian {
namespace plans {
namespace mstc_online {

class OldCell: public Cell {

public:
  OldCell(CellPtr, std::string);
  ~OldCell();

  const std::string& get_robot_name() const;
  const CellPtr& get_old_cell() const;
  void set_robot_name(const std::string&);
  void set_old_cell(const CellPtr&);

private:
  CellPtr old_cell;
  std::string robot_name;
};

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_OLD_CELL_HPP_ */
