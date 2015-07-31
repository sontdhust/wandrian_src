/*
 * core.hpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_CORE_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_CORE_HPP_

#include <termios.h> // for keyboard input
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h> // for velocity commands

namespace wandrian {

class Core {

public:
	Core();
	~Core();
	bool init();
	void spin();

private:
	bool last_zero_vel_sent; // avoid zero-vel messages from the beginning
	bool power_status;
	bool quit_requested;
	int key_file_descriptor;
	double linear_vel, angular_vel;

	struct termios original_terminal_state;
	geometry_msgs::TwistPtr cmd;
	ros::Publisher motor_power_publisher;
	ros::Publisher velocity_publisher;
	ecl::Thread thread;

	void keyboardInputLoop();
	void processKeyboardInput(char);

	void enable();
	void disable();
};

}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_CORE_HPP_ */
