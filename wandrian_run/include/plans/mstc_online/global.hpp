/*
 * global.hpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_GLOBAL_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_GLOBAL_HPP_

#include "plans/mstc_online/cell.hpp"
#include "plans/mstc_online/cell_in_old_cells.hpp"
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>
#include <list>

namespace wandrian {
namespace plans {
namespace mstc_online {

class Global {

public:
  Global();
  ~Global();
  std::set<CellPtr, CellComp> old_cells;                   // Old cells with set
  std::list<CellInOldCells> list_old_cells;               // Old cells with list

  static boost::shared_ptr<Global> get_instance();

  void write_message(std::string); // Write old cells
  void write_status(std::string);
//  void read_message();
  void read_message_with_set_data(); // Read old cells data, update to local old cells
  void read_message_with_list_data(); // Read old cells data, update to local old cells
  std::string read_message_with_set_data_no_update(); // Read old cells data, no update to local old cells
  std::string read_message_with_list_data_no_update(); // Read old cells data, no update to local old cells

  const std::string& get_robot_name() const;
  void set_robot_name(const std::string&);

  double get_tool_size() const;
  void set_tool_size(double);

  std::string read_status_from_ros_bag();       // Read status data from ros bag
  bool ask_other_robot_still_alive(std::string);
  std::string create_status_message(CellPtr); // robot_name, last x, last y, last time update, status
  void clear_robots_dead_old_cells(std::string, std::string, std::string);
  std::string find_robot_name(CellPtr);

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
  double tool_size;
  std::string message;
};

typedef boost::shared_ptr<Global> GlobalPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_MSTC_ONLINE_GLOBAL_HPP_ */
