/*
 * main.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#include "../include/core.hpp"

using namespace wandrian;

int main(int argc, char **argv) {
	ros::init(argc, argv, "run");
	Core core;
	if (core.init()) {
		core.spin();
	} else {
		ROS_ERROR_STREAM("Couldn't initialize Core!");
	}

	ROS_INFO_STREAM("Program exiting");
	return 0;
}
