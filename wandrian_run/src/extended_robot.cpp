/*
 * ExtendedCore.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: manhnh
 */

#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/KeyboardInput.h>
#include <ecl/time.hpp>

#include "../include/extended_robot.hpp"

// TODO: Choose relevant threshold values
#define THRESHOLD_COUNT 0.5
#define THRESHOLD_RANGE 0.9

namespace wandrian {

ExtendedRobot::ExtendedRobot() :
    starting_point_x(0), starting_point_y(0), tool_size(0), current_position(
        new Point(0, 0)), current_orientation(new Vector(0, 1)), linear_velocity_step(
        0), linear_velocity_max(0), angular_velocity_step(0), angular_velocity_max(
        0), velocity(new geometry_msgs::Twist()), is_quitting(false), is_powered(
        false), is_zero_vel(true), is_logging(false), file_descriptor(0) {
  tcgetattr(file_descriptor, &terminal); // get terminal properties
}

ExtendedRobot::~ExtendedRobot() {
  tcsetattr(file_descriptor, TCSANOW, &terminal);
}

bool ExtendedRobot::initialize() {
  ros::NodeHandle nh("~");

  nh.getParam("plan_name", plan_name);
  nh.getParam("starting_point_x", starting_point_x);
  nh.getParam("starting_point_y", starting_point_y);
  nh.getParam("tool_size", tool_size);
  nh.getParam("robot_name", robot_name);

  Global::get_instance()->set_tool_size(tool_size);
  Global::get_instance()->set_robot_name(robot_name);

  std::cout << "1. My name is " << Global::get_instance()->get_robot_name();
  std::cout << "2. My name is " << robot_name;
  std::cout << "3. Other information: " << plan_name << starting_point_x
      << starting_point_y << tool_size;

  nh.getParam("linear_velocity_step", linear_velocity_step);
  nh.getParam("linear_velocity_max", linear_velocity_max);
  nh.getParam("angular_velocity_step", angular_velocity_step);
  nh.getParam("angular_velocity_max", angular_velocity_max);

  obstacles[AT_RIGHT_SIDE] = false;
  obstacles[IN_FRONT] = false;
  obstacles[AT_LEFT_SIDE] = false;

  motor_power_publisher = nh.advertise<kobuki_msgs::MotorPower>("motor_power",
      1);
  velocity_publisher = nh.advertise<geometry_msgs::Twist>("velocity", 1);

  odom_subscriber = nh.subscribe<nav_msgs::Odometry>("odom", 1,
      &ExtendedRobot::subscribe_odometry, this);
  laser_subscriber = nh.subscribe<sensor_msgs::LaserScan>("laser", 1,
      &ExtendedRobot::subscribe_laser, this);

  velocity->linear.x = 0.0;
  velocity->linear.y = 0.0;
  velocity->linear.z = 0.0;
  velocity->angular.x = 0.0;
  velocity->angular.y = 0.0;
  velocity->angular.z = 0.0;

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
      ROS_FATAL_STREAM(
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

  // Start keyboard input thread
  thread_keyboard.start(&ExtendedRobot::start_thread_keyboard, *this);
  return true;
}

void ExtendedRobot::spin() {
  ros::Rate loop_rate(10);

  while (!is_quitting && ros::ok()) {
    // Avoid spamming robot with continuous zero-velocity messages
    if ((velocity->linear.x != 0.0) || (velocity->linear.y != 0.0)
        || (velocity->linear.z != 0.0) || (velocity->angular.x != 0.0)
        || (velocity->angular.y != 0.0) || (velocity->angular.z != 0.0)) {
      velocity_publisher.publish(velocity);
      is_zero_vel = false;
    } else if (!is_zero_vel) {
      velocity_publisher.publish(velocity);
      is_zero_vel = true;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  if (is_quitting) { // Ros node is still ok, send a disable command
    disable_power();
  } else {
    // Just in case we got here not via a keyboard quit request
    is_quitting = true;
    thread_keyboard.cancel();
    thread_run.cancel();
  }
  thread_keyboard.join();
}

void ExtendedRobot::stop() {
  velocity->linear.x = 0.0;
  velocity->angular.z = 0.0;
  velocity_publisher.publish(velocity);
}

std::string ExtendedRobot::get_plan_name() {
  return plan_name;
}

double ExtendedRobot::get_starting_point_x() {
  return starting_point_x;
}

double ExtendedRobot::get_starting_point_y() {
  return starting_point_y;
}

double ExtendedRobot::get_tool_size() {
  return tool_size;
}

PointPtr ExtendedRobot::get_current_position() {
  return current_position;
}

VectorPtr ExtendedRobot::get_current_orientation() {
  return current_orientation;
}

bool* ExtendedRobot::get_obstacles() {
  return obstacles;
}

double ExtendedRobot::get_linear_velocity_step() {
  return linear_velocity_step;
}

double ExtendedRobot::get_angular_velocity_step() {
  return angular_velocity_step;
}

void ExtendedRobot::set_behavior_run(boost::function<void()> behavior_run) {
  this->behavior_run = behavior_run;
}

void ExtendedRobot::set_linear_velocity(double linear_velocity) {
  velocity->linear.x = linear_velocity;
}

void ExtendedRobot::set_angular_velocity(double angular_velocity) {
  velocity->angular.z = angular_velocity;
}

//std::set<CellPtr, CellComp> ExtendedCore::get_old_cells(){
//  return *old_cells;
//}
//
//void ExtendedCore::insert_old_cells(CellPtr new_cell){
//  old_cells.insert(new_cell);
//}

void ExtendedRobot::run() {
  if (behavior_run)
    behavior_run();
}

void ExtendedRobot::start_thread_keyboard() {
  struct termios raw;
  memcpy(&raw, &terminal, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(file_descriptor, TCSANOW, &raw);

  puts("Available commands");
  puts("---------------------------");
  puts("p: Toggle motor power.");
  puts("l: Toggle logging.");
  puts("i: Get information");
  puts("r: Start running.");
  puts("c: Clear ros bag.");
  puts("q: Quit.");
  char c;
  while (!is_quitting) {
    if (read(file_descriptor, &c, 1) < 0) {
      perror("Read char failed():");
      exit(-1);
    }
    process_keyboard_input(c);
  }
}

void ExtendedRobot::process_keyboard_input(char c) {
  switch (c) {
  case kobuki_msgs::KeyboardInput::KeyCode_Down:
  case kobuki_msgs::KeyboardInput::KeyCode_Up:
  case kobuki_msgs::KeyboardInput::KeyCode_Right:
  case kobuki_msgs::KeyboardInput::KeyCode_Left:
    if (is_powered) {
      if (c == kobuki_msgs::KeyboardInput::KeyCode_Down
          && velocity->linear.x >= -linear_velocity_max) { // Decrease linear vel
        velocity->linear.x -= linear_velocity_step;
      } else if (c == kobuki_msgs::KeyboardInput::KeyCode_Up
          && velocity->linear.x <= linear_velocity_max) { // Increase linear vel
        velocity->linear.x += linear_velocity_step;
      } else if (c == kobuki_msgs::KeyboardInput::KeyCode_Right
          && velocity->angular.z >= -angular_velocity_max) { // Decrease angular vel
        velocity->angular.z -= angular_velocity_step;
      } else if (c == kobuki_msgs::KeyboardInput::KeyCode_Left
          && velocity->angular.z <= angular_velocity_max) { // Increase angular vel
        velocity->angular.z += angular_velocity_step;
      }
      ROS_INFO_STREAM(
          "[Vel]: (" << velocity->linear.x << ", " << velocity->angular.z << ")");
    } else {
      ROS_FATAL_STREAM("[Power]: Disabled");
    }
    break;
  case 'p':
    if (is_powered)
      disable_power();
    else
      enable_power();
    break;
  case 'l':
    is_logging = !is_logging;
    ROS_INFO_STREAM("[Logging]: " << (is_logging ? "On" : "Off"));
    break;
  case 'i':
    ROS_INFO_STREAM(
        "[Odom]: Pos(" << current_position->x << "," << current_position->y << "); " << "Ori(" << current_orientation->x << "," << current_orientation->y << "). [Laser]: Obs(" << obstacles[AT_RIGHT_SIDE] << "," << obstacles[IN_FRONT] << "," << obstacles[AT_LEFT_SIDE] << ")");
    break;
  case 'r':
    ROS_INFO_STREAM("[Run]: " << "Start running");
    thread_run.start(&ExtendedRobot::start_thread_run, *this);
    break;
  case 'c':
    Global::get_instance()->write_message("");
    Global::get_instance()->write_status("");
    break;
  case 'q':
//    Global::get_instance()->write_message("", "");
    is_quitting = true;
    break;
  default:
    break;
  }
}

void ExtendedRobot::start_thread_run() {
  run();
}

void ExtendedRobot::enable_power() {
  stop();
  ROS_INFO("[Power]: Enabled");
  kobuki_msgs::MotorPower power;
  power.state = kobuki_msgs::MotorPower::ON;
  motor_power_publisher.publish(power);
  is_powered = true;
}

void ExtendedRobot::disable_power() {
  stop();
  ROS_INFO("[Power]: Disabled");
  kobuki_msgs::MotorPower power;
  power.state = kobuki_msgs::MotorPower::OFF;
  motor_power_publisher.publish(power);
  is_powered = false;
}

void ExtendedRobot::subscribe_odometry(const nav_msgs::OdometryConstPtr& odom) {
  double px = odom->pose.pose.position.x;
  double py = odom->pose.pose.position.y;
  double ow = odom->pose.pose.orientation.w;
  double oz = odom->pose.pose.orientation.z;
  current_position->x = px + starting_point_x;
  current_position->y = py + starting_point_y;
  // FIXME: [Tmp]: Set initial orientation to (1, 0)
  current_orientation->x = ow * ow - oz * oz;
  current_orientation->y = 2 * oz * ow;
}

void ExtendedRobot::subscribe_laser(
    const sensor_msgs::LaserScanConstPtr& laser) {
  int range_size = laser->ranges.size();
  int range_right_size =
      -M_PI_2 - laser->angle_min >= 0 ?
          2 * range_size * (-M_PI_2 - laser->angle_min)
              / (laser->angle_max - laser->angle_min) :
          1;
  int range_left_size =
      laser->angle_max - M_PI_2 >= 0 ?
          2 * range_size * (laser->angle_max - M_PI_2)
              / (laser->angle_max - laser->angle_min) :
          1;

  int count;
  count = 0;
  for (int i = 0; i <= range_right_size - 1; i++) {
    if (laser->ranges[i] <= tool_size * THRESHOLD_RANGE)
      count++;
  }
  obstacles[AT_RIGHT_SIDE] = (count >= range_right_size * THRESHOLD_COUNT);

  count = 0;
  for (int i = range_right_size; i <= range_size - range_left_size - 1; i++) {
    if (laser->ranges[i] <= tool_size * THRESHOLD_RANGE)
      count++;
  }
  obstacles[IN_FRONT] = (count
      >= (range_size - range_right_size - range_left_size) * THRESHOLD_COUNT);

  count = 0;
  for (int i = range_size - range_left_size; i <= range_size - 1; i++) {
    if (laser->ranges[i] <= tool_size * THRESHOLD_RANGE)
      count++;
  }
  obstacles[AT_LEFT_SIDE] = (count >= range_left_size * THRESHOLD_COUNT);
  if (is_logging) {
    std::string obs = "";
    if (obstacles[AT_RIGHT_SIDE])
      obs += "Right,";
    if (obstacles[IN_FRONT])
      obs += "Ahead,";
    if (obstacles[AT_LEFT_SIDE])
      obs += "Left,";
    if (obs.length() > 0)
      obs = obs.substr(0, obs.length() - 1);
    ROS_WARN_STREAM("[Laser]: Obs(" << obs << ")");
  }
}

}
