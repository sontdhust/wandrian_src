/*
 * cell_in_old_cells.hpp
 *
 *  Created on: Feb 19, 2016
 *      Author: manhnh
 */

#ifndef WANDRIAN_MSTC_ONLINE_INCLUDE_PLANS_SPIRAL_STC_CELL_IN_OLD_CELLS_HPP_
#define WANDRIAN_MSTC_ONLINE_INCLUDE_PLANS_SPIRAL_STC_CELL_IN_OLD_CELLS_HPP_

#include "cell.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {
class CellInOldCells: public Cell {
public:
//  CellInOldCells::CellInOldCells();
  CellInOldCells(CellPtr, std::string);
  ~CellInOldCells();

  const std::string& get_robot_name() const;
  void set_robot_name(const std::string&);

  const CellPtr& get_moved_cell() const;
  void set_moved_cell(const CellPtr&);

//  const std::set<CellPtr, CellComp>& get_old_cells() const;
//  void set_old_cells(const std::set<CellPtr, CellComp>&);
//  void insert(CellPtr);
//  void begin();

private:

  CellPtr moved_cell;
  std::string robot_name;

//  std::set<CellPtr, CellComp> old_cells;
//  std::string robot_name;
};
}
}
}



#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_PLANS_SPIRAL_STC_CELL_IN_OLD_CELLS_HPP_ */
