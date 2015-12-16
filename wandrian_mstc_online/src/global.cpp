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
#define foreach BOOST_FOREACH

namespace wandrian {

GlobalPtr Global::instance;

Global::Global() {
}

Global::~Global() {
}

GlobalPtr Global::get_instance() {
  if (instance == NULL)
    instance = GlobalPtr(new Global());
  return instance;
}

void Global::write_message(std::string message) {
  rosbag::Bag bag;
  bag.open("message.bag", rosbag::bagmode::Write);
  std_msgs::String str;
  str.data = message.data();
  bag.write("communication_publisher", ros::Time::now(), str);
  bag.close();
}

void Global::read_message() {
  rosbag::Bag bag;
  bag.open("message.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("communication_publisher"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL) {
      ROS_INFO("%s", s->data.c_str());
    }
  }

  bag.close();

}

}
