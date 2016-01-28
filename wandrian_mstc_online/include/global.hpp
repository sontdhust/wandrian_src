/*
 * global.hpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_
#define WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_

#include "plans/spiral_stc/cell.hpp"
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>

using namespace wandrian::plans::spiral_stc;

namespace wandrian {

class Global {

public:
  Global();
  ~Global();
  std::set<CellPtr, CellComp> old_cells;

  void write_message(std::string);
  void read_message();
  std::string create_message_from_old_cells();
  void update_old_cells_from_message(std::string);
  static boost::shared_ptr<Global> get_instance();

  const std::string& get_robot_name() const;
  void set_robot_name(const std::string&);

  double get_robot_size() const;
  void set_robot_size(double);

private:
  static boost::shared_ptr<Global> instance;
  std::string robot_name;
  double robot_size;
  std::string message;
};

typedef boost::shared_ptr<Global> GlobalPtr;

}

#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_ */
