#include <ros/ros.h>
#include <ecl/time.hpp>
#include <ecl/exceptions.hpp>
#include <std_srvs/Empty.h>
#include <kobuki_msgs/MotorPower.h>
#include "../include/keyop_core/keyop_core.hpp"

namespace keyop_core {

KeyOpCore::KeyOpCore() :
		last_zero_vel_sent(true), // avoid zero-vel messages from the beginning
		accept_incoming(true), power_status(false), wait_for_connection_(true), cmd(
				new geometry_msgs::Twist()), cmd_stamped(
				new geometry_msgs::TwistStamped()), linear_vel_step(0.1), linear_vel_max(
				3.4), angular_vel_step(0.02), angular_vel_max(1.2), quit_requested(
				false), key_file_descriptor(0) {
	tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

KeyOpCore::~KeyOpCore() {
	tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

bool KeyOpCore::init() {
	ros::NodeHandle nh("~");

	name = nh.getUnresolvedNamespace();

	nh.getParam("linear_vel_step", linear_vel_step);
	nh.getParam("linear_vel_max", linear_vel_max);
	nh.getParam("angular_vel_step", angular_vel_step);
	nh.getParam("angular_vel_max", angular_vel_max);
	nh.getParam("wait_for_connection", wait_for_connection_);

	ROS_INFO_STREAM(
			"KeyOpCore : using linear  vel step [" << linear_vel_step << "].");
	ROS_INFO_STREAM(
			"KeyOpCore : using linear  vel max  [" << linear_vel_max << "].");
	ROS_INFO_STREAM(
			"KeyOpCore : using angular vel step [" << angular_vel_step << "].");
	ROS_INFO_STREAM(
			"KeyOpCore : using angular vel max  [" << angular_vel_max << "].");

	keyinput_subscriber = nh.subscribe("teleop", 1,
			&KeyOpCore::remoteKeyInputReceived, this);

	velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	motor_power_publisher_ = nh.advertise<kobuki_msgs::MotorPower>(
			"motor_power", 1);

	cmd->linear.x = 0.0;
	cmd->linear.y = 0.0;
	cmd->linear.z = 0.0;
	cmd->angular.x = 0.0;
	cmd->angular.y = 0.0;
	cmd->angular.z = 0.0;

	if (!wait_for_connection_) {
		return true;
	}
	ecl::MilliSleep millisleep;
	int count = 0;
	bool connected = false;
	while (!connected) {
		if (motor_power_publisher_.getNumSubscribers() > 0) {
			connected = true;
			break;
		}
		if (count == 6) {
			connected = false;
			break;
		} else {
			ROS_WARN_STREAM(
					"KeyOp: could not connect, trying again after 500ms...");
			try {
				millisleep(500);
			} catch (ecl::StandardException &e) {
				ROS_ERROR_STREAM("Waiting has been interrupted.");
				ROS_DEBUG_STREAM(e.what());
				return false;
			}
			++count;
		}
	}
	if (!connected) {
		ROS_ERROR("KeyOp: could not connect.");
		ROS_ERROR("KeyOp: check remappings for enable/disable topics).");
	} else {
		kobuki_msgs::MotorPower power_cmd;
		power_cmd.state = kobuki_msgs::MotorPower::ON;
		motor_power_publisher_.publish(power_cmd);
		ROS_INFO("KeyOp: connected.");
		power_status = true;
	}

	// start keyboard input thread
	thread.start(&KeyOpCore::keyboardInputLoop, *this);
	return true;
}

void KeyOpCore::spin() {
	ros::Rate loop_rate(10);

	while (!quit_requested && ros::ok()) {
		// Avoid spamming robot with continuous zero-velocity messages
		if ((cmd->linear.x != 0.0) || (cmd->linear.y != 0.0)
				|| (cmd->linear.z != 0.0) || (cmd->angular.x != 0.0)
				|| (cmd->angular.y != 0.0) || (cmd->angular.z != 0.0)) {
			velocity_publisher_.publish(cmd);
			last_zero_vel_sent = false;
		} else if (last_zero_vel_sent == false) {
			velocity_publisher_.publish(cmd);
			last_zero_vel_sent = true;
		}
		accept_incoming = true;
		ros::spinOnce();
		loop_rate.sleep();
	}
	if (quit_requested) { // ros node is still ok, send a disable command
		disable();
	} else {
		// just in case we got here not via a keyboard quit request
		quit_requested = true;
		thread.cancel();
	}
	thread.join();
}

void KeyOpCore::keyboardInputLoop() {
	struct termios raw;
	memcpy(&raw, &original_terminal_state, sizeof(struct termios));

	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(key_file_descriptor, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Forward/back arrows : linear velocity incr/decr.");
	puts("Right/left arrows : angular velocity incr/decr.");
	puts("Spacebar : reset linear/angular velocities.");
	puts("d : disable motors.");
	puts("e : enable motors.");
	puts("q : quit.");
	char c;
	while (!quit_requested) {
		if (read(key_file_descriptor, &c, 1) < 0) {
			perror("read char failed():");
			exit(-1);
		}
		processKeyboardInput(c);
	}
}

void KeyOpCore::remoteKeyInputReceived(const kobuki_msgs::KeyboardInput& key) {
	processKeyboardInput(key.pressedKey);
}

void KeyOpCore::processKeyboardInput(char c) {

	switch (c) {
	case kobuki_msgs::KeyboardInput::KeyCode_Left: {
		incrementAngularVelocity();
		break;
	}
	case kobuki_msgs::KeyboardInput::KeyCode_Right: {
		decrementAngularVelocity();
		break;
	}
	case kobuki_msgs::KeyboardInput::KeyCode_Up: {
		incrementLinearVelocity();
		break;
	}
	case kobuki_msgs::KeyboardInput::KeyCode_Down: {
		decrementLinearVelocity();
		break;
	}
	case kobuki_msgs::KeyboardInput::KeyCode_Space: {
		resetVelocity();
		break;
	}
	case 'q': {
		quit_requested = true;
		break;
	}
	case 'd': {
		disable();
		break;
	}
	case 'e': {
		enable();
		break;
	}
	default: {
		break;
	}
	}
}

void KeyOpCore::disable() {
	cmd->linear.x = 0.0;
	cmd->angular.z = 0.0;
	velocity_publisher_.publish(cmd);
	accept_incoming = false;

	if (power_status) {
		ROS_INFO(
				"KeyOp: die, die, die (disabling power to the device's motor system).");
		kobuki_msgs::MotorPower power_cmd;
		power_cmd.state = kobuki_msgs::MotorPower::OFF;
		motor_power_publisher_.publish(power_cmd);
		power_status = false;
	} else {
		ROS_WARN("KeyOp: Motor system has already been powered down.");
	}
}

void KeyOpCore::enable() {
	accept_incoming = false;

	cmd->linear.x = 0.0;
	cmd->angular.z = 0.0;
	velocity_publisher_.publish(cmd);

	if (!power_status) {
		ROS_INFO("KeyOp: Enabling power to the device subsystem.");
		kobuki_msgs::MotorPower power_cmd;
		power_cmd.state = kobuki_msgs::MotorPower::ON;
		motor_power_publisher_.publish(power_cmd);
		power_status = true;
	} else {
		ROS_WARN("KeyOp: Device has already been powered up.");
	}
}

void KeyOpCore::incrementLinearVelocity() {
	if (power_status) {
		if (cmd->linear.x <= linear_vel_max) {
			cmd->linear.x += linear_vel_step;
		}
		ROS_INFO_STREAM(
				"KeyOp: linear  velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
	} else {
		ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
	}
}

void KeyOpCore::decrementLinearVelocity() {
	if (power_status) {
		if (cmd->linear.x >= -linear_vel_max) {
			cmd->linear.x -= linear_vel_step;
		}
		ROS_INFO_STREAM(
				"KeyOp: linear  velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
	} else {
		ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
	}
}

void KeyOpCore::incrementAngularVelocity() {
	if (power_status) {
		if (cmd->angular.z <= angular_vel_max) {
			cmd->angular.z += angular_vel_step;
		}
		ROS_INFO_STREAM(
				"KeyOp: angular velocity incremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
	} else {
		ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
	}
}

void KeyOpCore::decrementAngularVelocity() {
	if (power_status) {
		if (cmd->angular.z >= -angular_vel_max) {
			cmd->angular.z -= angular_vel_step;
		}
		ROS_INFO_STREAM(
				"KeyOp: angular velocity decremented [" << cmd->linear.x << "|" << cmd->angular.z << "]");
	} else {
		ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
	}
}

void KeyOpCore::resetVelocity() {
	if (power_status) {
		cmd->angular.z = 0.0;
		cmd->linear.x = 0.0;
		ROS_INFO_STREAM("KeyOp: reset linear/angular velocities.");
	} else {
		ROS_WARN_STREAM("KeyOp: motors are not yet powered up.");
	}
}

}
