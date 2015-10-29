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
#include <sensor_msgs/LaserScan.h>
#include "../include/common/point.hpp"
#include "../include/common/vector.hpp"

#define AT_RIGHT_SIDE 0
#define IN_FRONT 1
#define AT_LEFT_SIDE 2

using namespace wandrian::common;

namespace wandrian {

class Core {

public:
  Core();
  virtual ~Core();
  bool initialize();
  void spin();

protected:
  PointPtr current_position; // odom subscriber
  VectorPtr current_orientation; // odom subscriber
  bool see_obstacle[3]; // laser subscriber
  geometry_msgs::TwistPtr velocity;
  double linear_vel_step, linear_vel_max, angular_vel_step, angular_vel_max;
  std::string plan; // arg
  double robot_size; // arg
  double starting_point_x; // arg
  double starting_point_y; // arg

  virtual void run();
  void stop();

private:
  bool is_quitting;
  bool is_powered;
  bool is_zero_vel; // avoid zero-vel messages from the beginning
  bool is_logging;
  int file_descriptor;

  struct termios terminal;
  ecl::Thread thread_keyboard;
  ecl::Thread thread_run;

  ros::Publisher motor_power_publisher;
  ros::Publisher velocity_publisher;
  ros::Subscriber odom_subscriber;
  ros::Subscriber laser_subscriber;

  // Thread handlers
  void start_thread_keyboard();
  void process_keyboard_input(char);
  void start_thread_run();

  // Helpers
  void enable_power();
  void disable_power();
  void subscribe_odometry(const nav_msgs::OdometryConstPtr&);
  void subscribe_laser(const sensor_msgs::LaserScanConstPtr&);
};

}

#endif /* WANDRIAN_RUN_INCLUDE_CORE_HPP_ */
