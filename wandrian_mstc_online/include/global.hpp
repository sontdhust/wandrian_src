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
  double robot_size;

  void write_message(std::string);
  void read_message();
  std::string create_message_from_old_cells();
  void update_old_cells_from_message(std::string);
  static boost::shared_ptr<Global> get_instance();

private:
  static boost::shared_ptr<Global> instance;
};

typedef boost::shared_ptr<Global> GlobalPtr;

}

#endif /* WANDRIAN_MSTC_ONLINE_INCLUDE_GLOBAL_HPP_ */
