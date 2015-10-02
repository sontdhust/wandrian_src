/*
 * core.hpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_CORE_HPP_
#define WANDRIAN_RUN_INCLUDE_CORE_HPP_

#include <termios.h> // for keyboard input
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h> // for velocity commands
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/BumperEvent.h>
#include <yocs_controllers/default_controller.hpp> // not use but need for bumper event subscriber
#include "../include/common/point.hpp"
#include "../include/common/vector.hpp"

using namespace wandrian::common;

namespace wandrian {

class Core {

public:
	Core();
	virtual ~Core();
	bool initialize();
	void spin();

protected:
	bool is_bumper_pressed;
	PointPtr current_position;
	VectorPtr current_orientation;
	geometry_msgs::TwistPtr twist;
	double linear_vel_step, linear_vel_max, angular_vel_step, angular_vel_max;
	std::string plan; // arg
	double robot_size; // arg
	double starting_point_x; // arg
	double starting_point_y; // arg

	virtual void run();
	void stop();

private:
	bool is_verbose; // arg

	bool is_quitting;
	bool is_powered;
	bool is_zero_vel; // avoid zero-vel messages from the beginning
	bool is_logging;
	int file_descriptor;

	struct termios terminal;
	ecl::Thread threadKeyboard;
	ecl::Thread threadRun;

	ros::Publisher motor_power_publisher;
	ros::Publisher velocity_publisher;
	ros::Subscriber odom_subscriber;
	ros::Subscriber bumper_subscriber;

	// Thread handlers
	void startThreadKeyboard();
	void processKeyboardInput(char);
	void startThreadRun();

	// Helpers
	void enablePower();
	void disablePower();
	void subscribeOdometry(const nav_msgs::OdometryConstPtr&);
	void subscribeBumper(const kobuki_msgs::BumperEventConstPtr&);
};

}

#endif /* WANDRIAN_RUN_INCLUDE_CORE_HPP_ */
