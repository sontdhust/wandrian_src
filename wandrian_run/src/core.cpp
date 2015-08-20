/*
 * core.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#include "../include/core.hpp"
#include <kobuki_msgs/MotorPower.h>
#include <ecl/time.hpp>

namespace wandrian {

Core::Core() :
		is_quitting(false), is_powered(false), is_zero_vel(true), is_logging(false), file_descriptor(
				0), linear_vel(0), angular_vel(0), twist(new geometry_msgs::Twist()) {
	tcgetattr(file_descriptor, &terminal); // get terminal properties
}

Core::~Core() {
	tcsetattr(file_descriptor, TCSANOW, &terminal);
}

bool Core::init() {
	ros::NodeHandle nh("~");

	nh.getParam("linear_vel", linear_vel);
	nh.getParam("angular_vel", angular_vel);

	ROS_INFO_STREAM("[Launch]: Using param 'linear vel'(" << linear_vel << ")");
	ROS_INFO_STREAM("[Launch]: Using param 'angular vel'(" << angular_vel << ")");

	motor_power_publisher = nh.advertise<kobuki_msgs::MotorPower>("motor_power",
			1);
	velocity_publisher = nh.advertise<geometry_msgs::Twist>("velocity", 1);
	odom_subscriber = nh.subscribe<nav_msgs::Odometry>("odom", 1,
			&Core::getOdometry, this);

	twist->linear.x = linear_vel;
	twist->linear.y = 0.0;
	twist->linear.z = 0.0;
	twist->angular.x = 0.0;
	twist->angular.y = 0.0;
	twist->angular.z = angular_vel;

	ecl::MilliSleep millisleep;
	int count = 0;
	bool connected = false;
	while (!connected) {
		if (motor_power_publisher.getNumSubscribers() > 0) {
			connected = true;
			break;
		}
		if (count == 6) {
			connected = false;
			break;
		} else {
			ROS_WARN_STREAM(
					"[Connection]: Could not connect, trying again after 500ms...");
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
		ROS_ERROR("[Connection]: Could not connect.");
	} else {
		kobuki_msgs::MotorPower motor_power;
		motor_power.state = kobuki_msgs::MotorPower::ON;
		motor_power_publisher.publish(motor_power);
		ROS_INFO("[Connection]: Connected.");
		is_powered = true;
	}

	// start keyboard input thread
	thread.start(&Core::keyboardInputLoop, *this);
	return true;
}

void Core::spin() {
	ros::Rate loop_rate(10);

	while (!is_quitting && ros::ok()) {
		// Avoid spamming robot with continuous zero-velocity messages
		if ((twist->linear.x != 0.0) || (twist->linear.y != 0.0)
				|| (twist->linear.z != 0.0) || (twist->angular.x != 0.0)
				|| (twist->angular.y != 0.0) || (twist->angular.z != 0.0)) {
			velocity_publisher.publish(twist);
			is_zero_vel = false;
		} else if (!is_zero_vel) {
			velocity_publisher.publish(twist);
			is_zero_vel = true;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	if (is_quitting) { // ros node is still ok, send a disable command
		disablePower();
	} else {
		// just in case we got here not via a keyboard quit request
		is_quitting = true;
		thread.cancel();
	}
	thread.join();
}

void Core::keyboardInputLoop() {
	struct termios raw;
	memcpy(&raw, &terminal, sizeof(struct termios));

	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(file_descriptor, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("p: Toggle motor power.");
	puts("l: Toggle logging.");
	puts("q: Quit.");
	char c;
	while (!is_quitting) {
		if (read(file_descriptor, &c, 1) < 0) {
			perror("Read char failed():");
			exit(-1);
		}
		processKeyboardInput(c);
	}
}

void Core::processKeyboardInput(char c) {
	switch (c) {
	case 'p':
		if (is_powered)
			disablePower();
		else
			enablePower();
		break;
	case 'l':
		is_logging = !is_logging;
		break;
	case 'q':
		is_quitting = true;
		break;
	default:
		break;
	}
}

void Core::enablePower() {
	twist->linear.x = linear_vel;
	twist->angular.z = angular_vel;
	velocity_publisher.publish(twist);
	ROS_INFO("[Power]: Enabling power to the device subsystem.");
	kobuki_msgs::MotorPower power;
	power.state = kobuki_msgs::MotorPower::ON;
	motor_power_publisher.publish(power);
	is_powered = true;
}

void Core::disablePower() {
	twist->linear.x = 0.0;
	twist->angular.z = 0.0;
	velocity_publisher.publish(twist);
	ROS_INFO("[Power]: Disabling power to the device's motor system.");
	kobuki_msgs::MotorPower power;
	power.state = kobuki_msgs::MotorPower::OFF;
	motor_power_publisher.publish(power);
	is_powered = false;
}

void Core::getOdometry(const nav_msgs::Odometry::ConstPtr& odom) {
	double px = odom->pose.pose.position.x;
	double py = odom->pose.pose.position.y;
	double ow = odom->pose.pose.orientation.w;
	double ox = odom->pose.pose.orientation.y;
	double oy = odom->pose.pose.orientation.z;
	double oz = odom->pose.pose.orientation.w;
	double angle = atan2(2 * (oy * ox + ow * oz),
			ow * ow + ox * ox - oy * oy - oz * oz);
	if (is_logging)
		ROS_INFO_STREAM(
				"[Odom]: pos(" << px << "," << py << "); angle(" << angle << ")");
}

}
