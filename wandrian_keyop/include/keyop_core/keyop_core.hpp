#ifndef KEYOP_CORE_NODE_HPP_
#define KEYOP_CORE_NODE_HPP_

#include <ros/ros.h>
#include <termios.h> // for keyboard input
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h>  // for velocity commands
#include <geometry_msgs/TwistStamped.h>  // for velocity commands
#include <kobuki_msgs/KeyboardInput.h> // keycodes from remote teleops.

namespace keyop_core {

class KeyOpCore {
public:

	KeyOpCore();
	~KeyOpCore();
	bool init();
	void spin();

private:
	ros::Subscriber keyinput_subscriber;
	ros::Publisher velocity_publisher_;
	ros::Publisher motor_power_publisher_;
	bool last_zero_vel_sent;
	bool accept_incoming;
	bool power_status;
	bool wait_for_connection_;
	geometry_msgs::TwistPtr cmd;
	geometry_msgs::TwistStampedPtr cmd_stamped;
	double linear_vel_step, linear_vel_max;
	double angular_vel_step, angular_vel_max;
	std::string name;

	void enable();
	void disable();
	void incrementLinearVelocity();
	void decrementLinearVelocity();
	void incrementAngularVelocity();
	void decrementAngularVelocity();
	void resetVelocity();

	void keyboardInputLoop();
	void processKeyboardInput(char c);
	void remoteKeyInputReceived(const kobuki_msgs::KeyboardInput& key);
	void restoreTerminal();
	bool quit_requested;
	int key_file_descriptor;
	struct termios original_terminal_state;
	ecl::Thread thread;
};

}

#endif /* KEYOP_CORE_NODE_HPP_ */
