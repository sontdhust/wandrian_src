/*
 * cell_in_old_cells.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: manhnh
 */
#include "../../../include/plans/spiral_stc/cell_in_old_cells.hpp"
namespace wandrian {
namespace plans {
namespace spiral_stc {

//CellInOldCells::CellInOldCells(){
//}
CellInOldCells::CellInOldCells(CellPtr moved_cell, std::string robot_name)
{
  CellInOldCells::moved_cell = moved_cell;
  CellInOldCells::robot_name = robot_name;
}
CellInOldCells::~CellInOldCells() {
}

const std::string& CellInOldCells::get_robot_name() const {
  return CellInOldCells::robot_name;
}

void CellInOldCells::set_robot_name(const std::string& robot_name) {
  CellInOldCells::robot_name = robot_name;
}

const CellPtr& CellInOldCells::get_moved_cell() const {
  return CellInOldCells::moved_cell;
}

void CellInOldCells::set_moved_cell(const CellPtr& movedCell) {
  CellInOldCells::moved_cell = movedCell;
}

//const std::set<CellPtr, CellComp>& get_old_cells() const {
//  return OldCells::old_cells;
//}
//
//void set_old_cells(const std::set<CellPtr, CellComp>& old_cells) {
//  OldCells::old_cells = old_cells;
//}
//
//
//void insert(CellPtr temp){
//  OldCells::old_cells.insert(temp);
//}
}
}
}

