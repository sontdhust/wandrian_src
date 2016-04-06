/*
 * robot.hpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#ifndef WANDRIAN_INCLUDE_ROBOT_HPP_
#define WANDRIAN_INCLUDE_ROBOT_HPP_

#include <termios.h> // For keyboard input
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <geometry_msgs/Twist.h> // For velocity commands
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include "common/point.hpp"
#include "common/rectangle.hpp"
#include "common/vector.hpp"
#include "environment/mstc/communicator.hpp"

#define NUM_SECONDS 5  // Loop after 5 seconds

using namespace wandrian::common;
using namespace wandrian::environment::mstc;

namespace wandrian {

enum ObstacleMovement {
  STOPPING, COMING, LEAVING
};

class Robot {

public:
  Robot();
  ~Robot();
  bool initialize();
  void spin();
  void stop();

  std::string get_map_name();
  RectanglePtr get_map_boundary();
  double get_tool_size();
  double get_starting_point_x();
  double get_starting_point_y();
  std::string get_plan_name();
  PointPtr get_current_position();
  VectorPtr get_current_direction();
  bool* get_obstacles();
  ObstacleMovement get_obstacle_movement();
  double get_linear_velocity();
  double get_positive_angular_velocity();
  double get_negative_angular_velocity();
  double get_epsilon_rotational_direction();
  double get_epsilon_motional_direction();
  double get_epsilon_position();
  CommunicatorPtr get_communicator();
  void set_behavior_run(boost::function<void()>);
  void set_linear_velocity(double);
  void set_angular_velocity(double);
  void set_laser_range(double);

private:
  std::string map_name; // arg
  double map_center_x; // arg
  double map_center_y; // arg
  double map_boundary_width; // arg
  double map_boundary_height; // arg
  double tool_size; // arg
  double starting_point_x; // arg
  double starting_point_y; // arg
  std::string plan_name; // arg
  std::string robot_name; // arg
  double linear_velocity; // arg
  double positive_angular_velocity; // arg
  double negative_angular_velocity; // arg
  double proportion_ranges_count; // arg
  double proportion_ranges_sum; // arg
  double augmentation_factor_range; // arg
  double epsilon_rotational_direction; // arg
  double epsilon_motional_direction; // arg
  double epsilon_position; // arg

  PointPtr current_position; // odometry subscriber
  VectorPtr current_direction; // odometry subscriber
  bool obstacles[3]; // laser subscriber
  ObstacleMovement obstacle_movement; // laser timer
  double linear_velocity_step; // param
  double linear_velocity_max; // param
  double angular_velocity_step; // param
  double angular_velocity_max; // param
  CommunicatorPtr communicator;

  boost::function<void()> behavior_run;
  geometry_msgs::TwistPtr velocity;
  double laser_range;

  bool is_quitting;
  bool is_powered;
  bool is_zero_vel; // Avoid zero-vel messages from the beginning
  bool is_logging;
  int file_descriptor;
  PointPtr last_position;
  VectorPtr last_direction;
  sensor_msgs::LaserScan::_ranges_type laser_ranges;
  sensor_msgs::LaserScan::_ranges_type last_laser_ranges;
  double laser_ray;

  struct termios terminal;
  ecl::Thread thread_keyboard;
  ecl::Thread thread_run;
  ecl::Thread thread_status; // mstc_online only
  ros::Timer timer_laser;

  ros::Publisher publisher_power;
  ros::Publisher publisher_velocity;
  ros::Subscriber subscriber_odometry;
  ros::Subscriber subscriber_laser;

  void run();

  // Thread handlers
  void start_thread_keyboard();
  void process_keyboard_input(char);
  void start_thread_run();
  void start_timer_laser(const ros::TimerEvent&);
  void start_thread_status(); // mstc_online only, loop after NUM_SECOND second(s)

  // Helpers
  void enable_power();
  void disable_power();
  void subscribe_odometry(const nav_msgs::OdometryConstPtr&);
  void subscribe_laser(const sensor_msgs::LaserScanConstPtr&);
};

typedef boost::shared_ptr<Robot> RobotPtr;

}

#endif /* WANDRIAN_INCLUDE_ROBOT_HPP_ */
