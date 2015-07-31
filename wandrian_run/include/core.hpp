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
#include <kobuki_msgs/KeyboardInput.h> // keycodes from remote teleops.

namespace wandrian {

class Core {

public:
	Core();
	~Core();
	bool init();
	void spin();

private:
	bool last_zero_vel_sent; // avoid zero-vel messages from the beginning
	bool accept_incoming;
	bool power_status;
	geometry_msgs::TwistPtr cmd;
	bool quit_requested;
	int key_file_descriptor;
	struct termios original_terminal_state;

	ecl::Thread thread;
	ros::Publisher velocity_publisher;
	ros::Publisher motor_power_publisher;
	void keyboardInputLoop();
	void remoteKeyInputReceived(const kobuki_msgs::KeyboardInput&);
	void processKeyboardInput(char);
	void disable();
};

}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_CORE_HPP_ */
