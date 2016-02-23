/*
 * global.hpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_
#define WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_

#include "plans/spiral_stc/cell.hpp"
#include "plans/spiral_stc/cell_in_old_cells.hpp"
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>
#include <list>

using namespace wandrian::plans::spiral_stc;

namespace wandrian {

class Global {

public:
  Global();
  ~Global();
  std::set<CellPtr, CellComp> old_cells;                        //old cells with set
  std::list<CellInOldCells> list_old_cells;  //old cells with list

  static boost::shared_ptr<Global> get_instance();

  void write_message(std::string);
//  void read_message();
  void read_message_with_set_data();
  void read_message_with_list_data();

  const std::string& get_robot_name() const;
  void set_robot_name(const std::string&);

  double get_robot_size() const;
  void set_robot_size(double);

//BEGIN OLD CELLS WITH SET
  std::string create_message_from_old_cells();
  void update_old_cells_from_message(std::string);
//END OLD CELLS WITH SET

//BEGIN OLD CELLS WITH LIST
  std::string create_message_from_list_old_cells();
  void update_list_old_cells_from_message(std::string);
  bool find_cell_in_list(CellPtr);
  void insert_my_moved_cell_to_list(CellPtr);
//END OLD CELLS WITH LIST

private:
  static boost::shared_ptr<Global> instance;
  std::string robot_name;
  double robot_size;
  std::string message;
};

typedef boost::shared_ptr<Global> GlobalPtr;

}

#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_ */
