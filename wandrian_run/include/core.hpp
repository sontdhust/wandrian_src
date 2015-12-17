/*
 * core.hpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_RUN_INCLUDE_CORE_HPP_
#define WANDRIAN_RUN_INCLUDE_CORE_HPP_

#include <termios.h> // For keyboard input
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h> // For velocity commands
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
  ~Core();
  bool initialize();
  void spin();
  void stop();

  std::string get_plan_name();
  double get_starting_point_x();
  double get_starting_point_y();
  
  double get_robot_size();

  double get_environment_size();

  PointPtr get_current_position();
  VectorPtr get_current_orientation();
  bool* get_obstacles();
  double get_linear_velocity_step();
  double get_angular_velocity_step();

  void set_behavior_run(boost::function<void()>);
  void set_linear_velocity(double);
  void set_angular_velocity(double);

private:
  std::string plan_name; // arg
  double starting_point_x; // arg
  double starting_point_y; // arg
  double robot_size; // arg

  double environment_size; //arg

  PointPtr current_position; // odom subscriber
  VectorPtr current_orientation; // odom subscriber
  bool obstacles[3]; // laser subscriber
  double linear_velocity_step; // param
  double linear_velocity_max; // param
  double angular_velocity_step; // param
  double angular_velocity_max; // param

  boost::function<void()> behavior_run;
  geometry_msgs::TwistPtr velocity;

  bool is_quitting;
  bool is_powered;
  bool is_zero_vel; // Avoid zero-vel messages from the beginning
  bool is_logging;
  int file_descriptor;

  struct termios terminal;
  ecl::Thread thread_keyboard;
  ecl::Thread thread_run;

  ros::Publisher motor_power_publisher;
  ros::Publisher velocity_publisher;
  ros::Subscriber odom_subscriber;
  ros::Subscriber laser_subscriber;


  void run();

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
