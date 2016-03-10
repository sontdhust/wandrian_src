/*
 * main.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: anhnt
 */

#include "../include/wandrian.hpp"

using namespace wandrian;

int main(int argc, char **argv) {
  ros::init(argc, argv, "run");
  WandrianPtr wandrian = WandrianPtr(new Wandrian());
  if (wandrian->initialize()) {
    wandrian->spin();
  } else {
    ROS_ERROR_STREAM("Couldn't initialize Core!");
  }

  ROS_INFO_STREAM("Program exiting");
  return 0;
}
