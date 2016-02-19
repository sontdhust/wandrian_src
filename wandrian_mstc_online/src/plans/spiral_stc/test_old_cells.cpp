/*
 * old_cells.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: manhnh
 */
#include "../../../include/plans/spiral_stc/test_old_cells.hpp"
namespace wandrian {
namespace plans {
namespace spiral_stc {

TestOldCells::TestOldCells() {
}
TestOldCells::~TestOldCells() {
}
const std::set<CellPtr, CellComp>& TestOldCells::get_old_cells() const {
  return TestOldCells::old_cells;
}

void TestOldCells::set_old_cells(const std::set<CellPtr, CellComp>& old_cells) {
  TestOldCells::old_cells = old_cells;
}

const std::string& TestOldCells::get_robot_name() const {
  return TestOldCells::robot_name;
}

void TestOldCells::set_robot_name(const std::string& robot_name) {
  TestOldCells::robot_name = robot_name;
}

void TestOldCells::insert(CellPtr temp){
  TestOldCells::old_cells.insert(temp);
}
}
}
}
