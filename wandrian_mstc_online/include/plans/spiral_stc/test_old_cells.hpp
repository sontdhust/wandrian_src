/*
 * old_cells.hpp
 *
 *  Created on: Jan 28, 2016
 *      Author: manhnh
 */

#ifndef WANDRIAN_MSTC_ONLINE_INCLUDE_PLANS_SPIRAL_STC_TEST_OLD_CELLS_HPP_
#define WANDRIAN_MSTC_ONLINE_INCLUDE_PLANS_SPIRAL_STC_TEST_OLD_CELLS_HPP_

#include "../../plans/spiral_stc/cell.hpp"

namespace wandrian {
namespace plans {
namespace spiral_stc {
class TestOldCells: public Cell {
public:
  TestOldCells();
  ~TestOldCells();

  const std::set<CellPtr, CellComp>& get_old_cells() const;
  void set_old_cells(const std::set<CellPtr, CellComp>&);
  const std::string& get_robot_name() const;
  void set_robot_name(const std::string&);
  void insert(CellPtr);
  void begin();

private:
  std::set<CellPtr, CellComp> old_cells;
  std::string robot_name;
};
}
}
}

#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_PLANS_SPIRAL_STC_TEST_OLD_CELLS_HPP_ */
