/*
 * core.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#include "../include/core.hpp"
#include <kobuki_driver/kobuki.hpp>
#include <kobuki_msgs/MotorPower.h>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/linear_algebra.hpp>

namespace wandrian {

Core::Core() :
		last_zero_vel_sent(true), power_status(false), quit_requested(false), key_file_descriptor(
				0), linear_vel(0), angular_vel(0), cmd(new geometry_msgs::Twist()), slot_stream_data(
				&Core::processStreamData, *this), dx(0), dth(0) {
	tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties

	kobuki::Parameters parameters;
	parameters.sigslots_namespace = "/kobuki";
	parameters.device_port = "/dev/kobuki";
	parameters.enable_acceleration_limiter = false;
	kobuki.init(parameters);
	if (kobuki.enable())
		ROS_INFO_STREAM("[Power]: Enable power to the motors");
	else
		ROS_WARN_STREAM("[Power]: Could not enable power");
	slot_stream_data.connect("/kobuki/stream_data");
}

Core::~Core() {
	tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
	kobuki.setBaseControl(0, 0); //linear_velocity, angular_velocity in (m/s), (rad/s)
	kobuki.disable();
}

bool Core::init() {
	ros::NodeHandle nh("~");

	nh.getParam("linear_vel", linear_vel);
	nh.getParam("angular_vel", angular_vel);

	ROS_INFO_STREAM("[Launch]: Using param 'linear vel': " << linear_vel);
	ROS_INFO_STREAM("[Launch]: Using param 'angular vel': " << angular_vel);

	motor_power_publisher = nh.advertise<kobuki_msgs::MotorPower>("motor_power",
			1);
	velocity_publisher = nh.advertise<geometry_msgs::Twist>("velocity", 1);

	cmd->linear.x = linear_vel;
	cmd->linear.y = 0.0;
	cmd->linear.z = 0.0;
	cmd->angular.x = 0.0;
	cmd->angular.y = 0.0;
	cmd->angular.z = angular_vel;

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
		kobuki_msgs::MotorPower power_cmd;
		power_cmd.state = kobuki_msgs::MotorPower::ON;
		motor_power_publisher.publish(power_cmd);
		ROS_INFO("[Connection]: Connected.");
		power_status = true;
	}

	// start keyboard input thread
	thread.start(&Core::keyboardInputLoop, *this);
	return true;
}

void Core::spin() {
	ros::Rate loop_rate(10);

	while (!quit_requested && ros::ok()) {
		// Avoid spamming robot with continuous zero-velocity messages
		if ((cmd->linear.x != 0.0) || (cmd->linear.y != 0.0)
				|| (cmd->linear.z != 0.0) || (cmd->angular.x != 0.0)
				|| (cmd->angular.y != 0.0) || (cmd->angular.z != 0.0)) {
			velocity_publisher.publish(cmd);
			last_zero_vel_sent = false;
		} else if (!last_zero_vel_sent) {
			velocity_publisher.publish(cmd);
			last_zero_vel_sent = true;
		}
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

ecl::Pose2D<double> Core::getPose() {
	return pose;
}

void Core::keyboardInputLoop() {
	struct termios raw;
	memcpy(&raw, &original_terminal_state, sizeof(struct termios));

	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(key_file_descriptor, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("e: Enable motor power.");
	puts("d: Disable motor power.");
	puts("i: Display information.");
	puts("q: Quit.");
	char c;
	while (!quit_requested) {
		if (read(key_file_descriptor, &c, 1) < 0) {
			perror("Read char failed():");
			exit(-1);
		}
		processKeyboardInput(c);
	}
}

void Core::processKeyboardInput(char c) {
	switch (c) {
	case 'e':
		enable();
		break;
	case 'd':
		disable();
		break;
	case 'i':
		displayInformation();
		break;
	case 'q':
		quit_requested = true;
		break;
	default:
		break;
	}
}

void Core::enable() {
	cmd->linear.x = linear_vel;
	cmd->angular.z = angular_vel;
	velocity_publisher.publish(cmd);

	if (!power_status) {
		ROS_INFO("[Power]: Enabling power to the device subsystem.");
		kobuki_msgs::MotorPower power_cmd;
		power_cmd.state = kobuki_msgs::MotorPower::ON;
		motor_power_publisher.publish(power_cmd);
		power_status = true;
	} else {
		ROS_WARN("[Power]: Device has already been powered up.");
	}
}

void Core::disable() {
	cmd->linear.x = 0.0;
	cmd->angular.z = 0.0;
	velocity_publisher.publish(cmd);

	if (power_status) {
		ROS_INFO("[Power]: Disabling power to the device's motor system.");
		kobuki_msgs::MotorPower power_cmd;
		power_cmd.state = kobuki_msgs::MotorPower::OFF;
		motor_power_publisher.publish(power_cmd);
		power_status = false;
	} else {
		ROS_WARN("[Power]: Motor system has already been powered down.");
	}
}

void Core::processStreamData() {
	ecl::Pose2D<double> pose_update;
	ecl::linear_algebra::Vector3d pose_update_rates;
	kobuki.updateOdometry(pose_update, pose_update_rates);
	pose *= pose_update;
	dx += pose_update.x();
	dth += pose_update.heading();
	processMotion();
}

void Core::processMotion() {
	if (dx >= 1.0 && dth >= ecl::pi / 2.0) {
		dx = 0.0;
		dth = 0.0;
		kobuki.setBaseControl(0.0, 0.0);
		return;
	} else if (dx >= 1.0) {
		kobuki.setBaseControl(0.0, 3.3);
		return;
	} else {
		kobuki.setBaseControl(0.3, 0.0);
		return;
	}
}

void Core::displayInformation() {
	processStreamData();
	ROS_INFO_STREAM(
			"[Info]: pose(" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "); " << "dx(" << dx << "); dth(" << dth << ")");
}

}
