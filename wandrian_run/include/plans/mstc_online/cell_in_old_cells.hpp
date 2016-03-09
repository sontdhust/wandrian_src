/*
 * cell_in_old_cells.hpp
 *
 *  Created on: Feb 19, 2016
 *      Author: manhnh
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_CELL_IN_OLD_CELLS_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_CELL_IN_OLD_CELLS_HPP_

#include "cell.hpp"

namespace wandrian {
namespace plans {
namespace mstc_online {
class CellInOldCells: public Cell {

public:
  CellInOldCells(CellPtr, std::string);
  ~CellInOldCells();

  const std::string& get_robot_name() const;
  const CellPtr& get_moved_cell() const;
  void set_robot_name(const std::string&);
  void set_moved_cell(const CellPtr&);

private:

  CellPtr moved_cell;
  std::string robot_name;

};
}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_CELL_IN_OLD_CELLS_HPP_ */
