/*
 * communicator.hpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_RUN_INCLUDE_ENVIRONMENT_COMMUNICATOR_HPP_
#define WANDRIAN_RUN_INCLUDE_ENVIRONMENT_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>
#include <list>

#include "identifiable_cell.hpp"

namespace wandrian {
namespace environment {

class Communicator {

public:
  std::set<IdentifiableCellPtr, CellComp> cells; // Old cells with set. TODO: Remove????

  Communicator();
  ~Communicator();

  void write_old_cells_message(std::string); // Write old cells
  void write_status_message(std::string);
  std::string create_old_cells_message();
  std::string create_status_message(IdentifiableCellPtr); // robot_name, last x, last y, last time update, status
  void read_message_then_update_old_cells(); // Read old cells data, update to local old cells

  bool ask_other_robot_still_alive(std::string);
  std::string find_robot_name(IdentifiableCellPtr);
  bool find_old_cell(IdentifiableCellPtr);
  void insert_old_cell(IdentifiableCellPtr);

  std::string get_robot_name() const;
  void set_robot_name(const std::string&);
  void set_tool_size(double);

  CellPtr& get_current_cell();
  void set_current_cell(const CellPtr&);

private:
  std::string robot_name;
  double tool_size;
  CellPtr current_cell;
  std::list<IdentifiableCellPtr> old_cells; // Old cells with list

  std::string read_old_cells_message(); // Read old cells data, no update to local old cells
  void update_old_cells_from_message(std::string);
  std::string read_status_message(); // Read status data from ros bag
  void clear_robots_dead_old_cells(std::string, std::string, std::string);
};

typedef boost::shared_ptr<Communicator> CommunicatorPtr;

}
}

#endif /* WANDRIAN_RUN_INCLUDE_ENVIRONMENT_COMMUNICATOR_HPP_ */
