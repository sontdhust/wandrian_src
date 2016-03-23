/*
 * communicator.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <algorithm>
#include <sstream>
#include "../../../include/environment/mstc_online/communicator.hpp"

#define foreach BOOST_FOREACH

namespace wandrian {
namespace environment {
namespace mstc_online {

Communicator::Communicator() {
  tool_size = 0.5;
}

Communicator::~Communicator() {
}

void Communicator::write_old_cells_message(std::string message) {
  // Write old cells
  ROS_INFO("[Writing]My old cells: %s", message.data());
  rosbag::Bag bag;
  bag.open("message.bag", rosbag::bagmode::Write);
  std_msgs::String str;
  str.data = message.data();
  bag.write("publisher_communication", ros::Time::now(), str);
  bag.close();
}

void Communicator::write_status_message(std::string status) {
  // Write status
  rosbag::Bag status_bag;
  ROS_INFO("[Writing status]Status: %s", status.data());
  status_bag.open("status.bag", rosbag::bagmode::Write);
  std_msgs::String str_status;
  str_status.data = status.data();
  status_bag.write("status_publisher", ros::Time::now(), str_status);
  status_bag.close();
}

std::string Communicator::create_old_cells_message() {
  std::string messages;
  for (std::list<IdentifiableCellPtr>::iterator item = old_cells.begin();
      item != old_cells.end(); ++item) {
    std::stringstream messsage;
    messsage << (*item)->get_center()->x << "," << (*item)->get_center()->y
        << "," << (*item)->get_robot_name() << ";";
    messages.append(messsage.str());
  }
  return messages;
}

std::string Communicator::create_status_message(IdentifiableCellPtr last_cell) {
  bool check_added = false;
  std::string my_status;
  std::string all_robots_new_status = "";
  std::stringstream status;
  std::string status_from_ros_bag = read_status_message();
  status << get_robot_name() << "," << last_cell->get_center()->x << ","
      << last_cell->get_center()->y << "," << ros::Time::now() << ","
      << "[ALIVE];";
  my_status.append(status.str());
  if (status_from_ros_bag == "") {
    all_robots_new_status = my_status;
  } else {
    boost::char_separator<char> split_status(";");
    boost::tokenizer<boost::char_separator<char> > tokens(status_from_ros_bag,
        split_status);
    foreach (const std::string& status, tokens) {
      if (status.find(get_robot_name()) != std::string::npos) {
        // Found
        all_robots_new_status.append(my_status);
        check_added = true;
      } else {
        // Not found
        all_robots_new_status.append(status);
        all_robots_new_status.append(";");
      }
    }
    if (!check_added) {
      all_robots_new_status.append(my_status);
    }
  }
  return all_robots_new_status;
}

void Communicator::read_message_then_update_old_cells() {
  update_old_cells_from_message(read_old_cells_message());
}

bool Communicator::ask_other_robot_still_alive(
    std::string robot_name_want_ask) {
  bool result = true;
  std::string cell_string;
  int i;
  std::string status_string; // When robot is dead, change robot's status to [DEAD] and store to this variable
  if (robot_name_want_ask == get_robot_name()) {
    result = true;
  } else {
    boost::char_separator<char> split_status(";");
    boost::char_separator<char> split_information(",");
    boost::tokenizer<boost::char_separator<char> > tokens(read_status_message(),
        split_status);
    foreach (const std::string& status, tokens) {
      if (status.find(robot_name_want_ask) != std::string::npos) {
        // Found
        boost::tokenizer<boost::char_separator<char> > tokens(status,
            split_information);
        i = 1;
        foreach (const std::string& information, tokens) {
          if (i == 1) {
            status_string.append(information);
            status_string.append(",");
          } else if (i == 2) {
            status_string.append(information);
            status_string.append(",");
            cell_string.append(information);
            cell_string.append(",");
          } else if (i == 3) {
            status_string.append(information);
            status_string.append(",");
            cell_string.append(information);
            cell_string.append(",");
          } else if (i == 4) {
            status_string.append(information);

            int old_time = atoi(information.c_str());
            int current;
            std::stringstream ss;
            ss << ros::Time::now();
            current = atoi(ss.str().c_str());

            std::cout << "DHBKHNHEDSPIK56 <<" << current - old_time << ">>";
            if (current - old_time > 45) {
              // Robot was dead
              result = false;
              status_string.append("[DEAD];");
              cell_string.append(get_robot_name());
              cell_string.append(";");

              // Update all status
              boost::char_separator<char> split(";");
              boost::tokenizer<boost::char_separator<char> > tokens(
                  read_status_message(), split);
              foreach (const std::string& dead_robot_status, tokens) {
                if (dead_robot_status.find(robot_name_want_ask)
                    != std::string::npos) {
                  // Found
                  continue;
                } else {
                  // Not found
                  status_string.append(dead_robot_status);
                  status_string.append(";");
                }
              }
              clear_robots_dead_old_cells(robot_name_want_ask, cell_string,
                  status_string);
            }
          } else if (i == 5) {
            if (information == "[DEAD];") {
              status_string.append(information);
              // Robot was dead
              result = false;
              cell_string.append(get_robot_name());
              cell_string.append(";");

              // Update all status
              boost::char_separator<char> split(";");
              boost::tokenizer<boost::char_separator<char> > tokens(
                  read_status_message(), split);
              foreach (const std::string& dead_robot_status, tokens) {
                if (dead_robot_status.find(robot_name_want_ask)
                    != std::string::npos) {
                  // Found
                  continue;
                } else {
                  // Not found
                  status_string.append(dead_robot_status);
                  status_string.append(";");
                }
              }
              clear_robots_dead_old_cells(robot_name_want_ask, cell_string,
                  status_string);
            }
          }
          i++;
        }
      } else {
        // Not found
      }
    }
  }
  return result;
}

std::string Communicator::find_robot_name(IdentifiableCellPtr cell_to_find) {
  std::string robot_name = "NOT FOUND";
  int i;
  double x;
  double y;
  boost::char_separator<char> split_old_cells(";");
  boost::char_separator<char> split_point(",");
  boost::tokenizer<boost::char_separator<char> > tokens(
      create_old_cells_message(), split_old_cells);
  foreach (const std::string& cell, tokens) {
    if (robot_name != "NOT FOUND")
      break;
    boost::tokenizer<boost::char_separator<char> > tokens(cell, split_point);
    i = 1;
    foreach (const std::string& coordinates, tokens) {
      if (i == 1) {
        x = atof(coordinates.c_str());
      } else if (i == 2) {
        y = atof(coordinates.c_str());
      } else if (i == 3) {
        if ((x == cell_to_find->get_center()->x)
            && (y == cell_to_find->get_center()->y))
          robot_name = coordinates.c_str();
        break;
      }
      i++;
    }
  }
  return robot_name;
}

bool Communicator::find_old_cell(IdentifiableCellPtr cell) {
  bool value = false;
  for (std::list<IdentifiableCellPtr>::iterator item = old_cells.begin();
      item != old_cells.end(); ++item) {
    if (((*item)->get_center()->x == cell->get_center()->x)
        && ((*item)->get_center()->y == cell->get_center()->y)) {
      value = true;
      break;
    }
  }
  return value;
}

void Communicator::insert_old_cell(IdentifiableCellPtr cell) {
  old_cells.push_back(
      IdentifiableCellPtr(
          new IdentifiableCell(cell->get_center(), cell->get_size(),
              get_robot_name())));
}

std::string Communicator::get_robot_name() const {
  return robot_name;
}

void Communicator::set_robot_name(const std::string& robot_name) {
  this->robot_name = robot_name;
}

void Communicator::set_tool_size(double tool_size) {
  this->tool_size = tool_size;
}

std::string Communicator::read_old_cells_message() {
  rosbag::Bag bag;
  std::string msg;
  bag.open("message.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("publisher_communication"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL) {
      msg.append(s->data.c_str());
    }
  }
  bag.close();
  ROS_INFO("[Reading]Ros bag old cells: %s", msg.data());
  return msg;
}

void Communicator::update_old_cells_from_message(std::string msg) {
  old_cells.clear();
  double x = 0.0;
  double y = 0.0;
  std::string robot_name;
  int i;
  boost::char_separator<char> split_old_cells(";");
  boost::char_separator<char> split_point(",");
  boost::tokenizer<boost::char_separator<char> > tokens(msg, split_old_cells);
  foreach (const std::string& cell, tokens) {
    boost::tokenizer<boost::char_separator<char> > tokens(cell, split_point);
    i = 1;
    foreach (const std::string& coordinates, tokens) {
      if (i == 1) {
        x = atof(coordinates.c_str());
      } else if (i == 2) {
        y = atof(coordinates.c_str());
      } else if (i == 3) {
        robot_name = coordinates.c_str();
      }
      i++;
    }
    IdentifiableCellPtr old_cell = IdentifiableCellPtr(
        new IdentifiableCell(PointPtr(new Point(x, y)), 2 * tool_size,
            robot_name));
    old_cells.push_back(old_cell);
  }
  ROS_INFO("[Reading]My old cells: %s", create_old_cells_message().data());
}

std::string Communicator::read_status_message() {
  rosbag::Bag bag;
  std::string msg;
  bag.open("status.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("status_publisher"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL) {
      msg.append(s->data.c_str());
    }
  }
  bag.close();
  return msg;
}

void Communicator::clear_robots_dead_old_cells(std::string dead_robot_name,
    std::string last_cell, std::string last_status) {
  // Read old cells data from ros bag
  std::string old_old_cells = read_old_cells_message();
  std::string new_old_cells;
  // Clear robot dead old cells
  boost::char_separator<char> split_old_cell(";");
  boost::tokenizer<boost::char_separator<char> > tokens(old_old_cells,
      split_old_cell);
  foreach (const std::string& cell, tokens) {
    if (cell.find(dead_robot_name) != std::string::npos) {
      // Found
      continue;
    } else {
      // Not found
      new_old_cells.append(cell);
      new_old_cells.append(";");
    }
  }
  new_old_cells.append(last_cell);
  // Write new old cells to ros bag and update status all robots
  write_old_cells_message(new_old_cells);
  write_status_message(last_status);
}

}
}
}
