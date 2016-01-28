/*
 * global.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: manhnh
 */

#include "../include/global.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#define foreach BOOST_FOREACH
#include <algorithm>
#include <sstream>

namespace wandrian {

GlobalPtr Global::instance;

//Global::Global() {
//  robot_size = 0.5;
//}

Global::~Global() {
}

GlobalPtr Global::get_instance() {
  if (instance == NULL)
    instance = GlobalPtr(new Global());
  return instance;
}

const std::string& Global::get_robot_name() const {
  return robot_name;
}

void Global::set_robot_name(const std::string& robotName) {
  robot_name = robotName;
}

double Global::get_robot_size() const {
  return robot_size;
}

void Global::set_robot_size(double robotSize) {
  robot_size = robotSize;
}

void Global::write_message(std::string message) {
  ROS_INFO("[Writing]My old cells: %s", message.data());
  rosbag::Bag bag;
  bag.open("message.bag", rosbag::bagmode::Write);
  std_msgs::String str;
  str.data = message.data();
  bag.write("communication_publisher", ros::Time::now(), str);
  bag.close();
}

void Global::read_message() {
  rosbag::Bag bag;
  std::string msg;

  bag.open("message.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("communication_publisher"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL) {
//      ROS_INFO("Ros bag old cells: %s", s->data.c_str());
      msg.append(s->data.c_str());
    }
  }

  bag.close();
  ROS_INFO("[Reading]Ros bag old cells: %s", msg.data());
  update_old_cells_from_message(msg);

}

std::string Global::create_message_from_old_cells() {
  std::string msg;
  std::set<CellPtr, CellComp> temp_old_cells = this->old_cells;
  // for (int i = 0; i <= temp_old_cells.size(); i++) {
  while (temp_old_cells.size() != 0) {
    CellPtr temp_cell = *temp_old_cells.begin();
    std::stringstream tmp;
    tmp << temp_cell->get_center()->x << "," << temp_cell->get_center()->y
        << get_robot_name() << ";";
    msg.append(tmp.str());
    temp_old_cells.erase(temp_cell);
  }
  return msg;
}

void Global::update_old_cells_from_message(std::string msg) {
  double x = 0.0;
  double y = 0.0;
  CellPtr temp_cell;
  int i;
  boost::char_separator<char> split_old_cells(";");
  boost::char_separator<char> split_point(",");
  boost::tokenizer<boost::char_separator<char> > tokens(msg, split_old_cells);
  BOOST_FOREACH (const std::string& cell, tokens) {
    boost::tokenizer<boost::char_separator<char> > tokens(cell, split_point);
    i = 1;
    BOOST_FOREACH (const std::string& coordinates, tokens) {
      if (i == 1) {
        x = atof(coordinates.c_str());
      } else if (i == 2) {
        y = atof(coordinates.c_str());
      }
      i++;
    }
    temp_cell = CellPtr(new Cell(PointPtr(new Point(x, y)), 2 * robot_size));
    this->old_cells.insert(temp_cell);
  }
  ROS_INFO("[Reading]My old cells: %s", create_message_from_old_cells().data());
}
}
