/*
 * robot.cpp
 *
 *  Created on: Jul 31, 2015
 *      Author: sontd
 */

#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/KeyboardInput.h>
#include <ecl/time.hpp>
#include "../include/robot.hpp"

// FIXME: Choose relevant values
#define LASER_COUNT_RATE 0.4
#define LASER_AUGMENTATION_FACTOR 1.0
#define LASER_SCANNING_ANGLE (1.0 / 3.0 * M_PI)

namespace wandrian {

Robot::Robot() :
    map_center_x(0), map_center_y(0), map_boundary_width(0), map_boundary_height(
        0), tool_size(0), starting_point_x(0), starting_point_y(0), linear_velocity(
        0), positive_angular_velocity(0), negative_angular_velocity(0), laser_count_rate(
        0), laser_augmentation_factor(0), laser_scanning_angle(0), epsilon_rotational_direction(
        0), epsilon_motional_direction(0), epsilon_position(0), deviation_linear_position(
        0), deviation_angular_position(0), threshold_linear_step_count(0), threshold_angular_step_count(
        0), delay(0), current_position(new Point()), current_direction(
        new Vector()), linear_velocity_step(0), linear_velocity_max(0), angular_velocity_step(
        0), angular_velocity_max(0), velocity(new geometry_msgs::Twist()), is_quitting(
        false), is_powered(false), is_zero_vel(true), is_logging(false), file_descriptor(
        0) {
  tcgetattr(file_descriptor, &terminal); // get terminal properties
}

Robot::~Robot() {
  tcsetattr(file_descriptor, TCSANOW, &terminal);
}

bool Robot::initialize() {
  ros::NodeHandle nh("~");

  nh.getParam("map_name", map_name);
  nh.getParam("map_center_x", map_center_x);
  nh.getParam("map_center_y", map_center_y);
  nh.getParam("map_boundary_width", map_boundary_width);
  nh.getParam("map_boundary_height", map_boundary_height);
  nh.getParam("tool_size", tool_size);
  nh.getParam("starting_point_x", starting_point_x);
  nh.getParam("starting_point_y", starting_point_y);
  nh.getParam("plan_name", plan_name);
  nh.getParam("robot_name", robot_name);
  nh.getParam("linear_velocity", linear_velocity);
  nh.getParam("positive_angular_velocity", positive_angular_velocity);
  nh.getParam("negative_angular_velocity", negative_angular_velocity);
  nh.getParam("laser_count_rate", laser_count_rate);
  nh.getParam("laser_augmentation_factor", laser_augmentation_factor);
  nh.getParam("laser_scanning_angle", laser_scanning_angle);
  nh.getParam("epsilon_rotational_direction", epsilon_rotational_direction);
  nh.getParam("epsilon_motional_direction", epsilon_motional_direction);
  nh.getParam("epsilon_position", epsilon_position);
  nh.getParam("deviation_linear_position", deviation_linear_position);
  nh.getParam("deviation_angular_position", deviation_angular_position);
  nh.getParam("threshold_linear_step_count", threshold_linear_step_count);
  nh.getParam("threshold_angular_step_count", threshold_angular_step_count);
  nh.getParam("delay", delay);

  nh.getParam("linear_velocity_step", linear_velocity_step);
  nh.getParam("linear_velocity_max", linear_velocity_max);
  nh.getParam("angular_velocity_step", angular_velocity_step);
  nh.getParam("angular_velocity_max", angular_velocity_max);

  if (plan_name == "mstc_online") {
    communicator = CommunicatorPtr(new Communicator());
    communicator->set_robot_name(robot_name);
    communicator->set_tool_size(tool_size);
    std::cout << "1. My name is " << communicator->get_robot_name();
    std::cout << "2. My name is " << robot_name;
    std::cout << "3. Other information: " << plan_name << starting_point_x
        << starting_point_y << tool_size;
  }

  if (laser_count_rate <= 0 || laser_count_rate >= 1)
    laser_count_rate = LASER_COUNT_RATE;
  if (laser_augmentation_factor <= 0)
    laser_augmentation_factor = LASER_AUGMENTATION_FACTOR;
  if (laser_scanning_angle <= 0)
    laser_scanning_angle = LASER_SCANNING_ANGLE;

  publisher_power = nh.advertise<kobuki_msgs::MotorPower>("power", 1);
  publisher_velocity = nh.advertise<geometry_msgs::Twist>("velocity", 1);
  subscriber_odometry = nh.subscribe<nav_msgs::Odometry>("odometry", 1,
      &Robot::subscribe_odometry, this);
  subscriber_laser = nh.subscribe<sensor_msgs::LaserScan>("laser", 1,
      &Robot::subscribe_laser, this);

  velocity->linear.x = 0.0;
  velocity->linear.y = 0.0;
  velocity->linear.z = 0.0;
  velocity->angular.x = 0.0;
  velocity->angular.y = 0.0;
  velocity->angular.z = 0.0;

  ecl::MilliSleep milliSleep;
  int count = 0;
  bool connected = false;
  while (!connected) {
    if (publisher_power.getNumSubscribers() > 0) {
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
        milliSleep(500);
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
    kobuki_msgs::MotorPower power;
    power.state = kobuki_msgs::MotorPower::ON;
    publisher_power.publish(power);
    ROS_INFO("[Connection]: Connected.");
    is_powered = true;
  }

  // Start keyboard input thread
  thread_keyboard.start(&Robot::start_thread_keyboard, *this);
  return true;
}

void Robot::spin() {
  ros::Rate loop_rate(10);
  while (!is_quitting && ros::ok()) {
    // Avoid spamming robot with continuous zero-velocity messages
    if ((velocity->linear.x != 0.0) || (velocity->linear.y != 0.0)
        || (velocity->linear.z != 0.0) || (velocity->angular.x != 0.0)
        || (velocity->angular.y != 0.0) || (velocity->angular.z != 0.0)) {
      publisher_velocity.publish(velocity);
      is_zero_vel = false;
    } else if (!is_zero_vel) {
      publisher_velocity.publish(velocity);
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
    thread_status.cancel();
  }
  thread_keyboard.join();
}

void Robot::stop() {
  velocity->linear.x *= 0;
  velocity->angular.z *= 0;
  // Force stop
  publisher_velocity.publish(velocity);
}

bool Robot::see_obstacle(double angle, double distance) {
  double laser_ray = (laser->angle_max - laser->angle_min)
      / (double) laser->ranges.size();
  double angle_min = angle - laser_scanning_angle / 2.0;
  double angle_max = angle + laser_scanning_angle / 2.0;

  // Detect obstacle
  if (angle_min <= laser->angle_max || angle_max >= laser->angle_min) {
    int count = 0;
    int range_min = (angle_min - laser->angle_min) / laser_ray;
    int range_max = (angle_max - laser->angle_min) / laser_ray;
    range_min = range_min >= 0 ? range_min : 0;
    range_max =
        range_max <= laser->ranges.size() ? range_max : laser->ranges.size();
    for (int i = range_min; i <= range_max - 1; i++) {
      if (laser->ranges[i] >= laser_augmentation_factor * distance / 2.25
          && laser->ranges[i] <= laser_augmentation_factor * distance)
        count++;
    }
    return (count >= (range_max - range_min) * laser_count_rate);
  }
  return false;
}

std::string Robot::get_map_name() {
  return map_name;
}

RectanglePtr Robot::get_map_boundary() {
  return RectanglePtr(
      new Rectangle(PointPtr(new Point(map_center_x, map_center_y)),
          map_boundary_width, map_boundary_height));
}

double Robot::get_tool_size() {
  return tool_size;
}

double Robot::get_starting_point_x() {
  return starting_point_x;
}

double Robot::get_starting_point_y() {
  return starting_point_y;
}

std::string Robot::get_plan_name() {
  return plan_name;
}

PointPtr Robot::get_current_position() {
  return current_position;
}

VectorPtr Robot::get_current_direction() {
  return current_direction;
}

double Robot::get_linear_velocity() {
  return linear_velocity;
}

double Robot::get_positive_angular_velocity() {
  return positive_angular_velocity;
}

double Robot::get_negative_angular_velocity() {
  return negative_angular_velocity;
}

double Robot::get_epsilon_rotational_direction() {
  return epsilon_rotational_direction;
}

double Robot::get_epsilon_motional_direction() {
  return epsilon_motional_direction;
}

double Robot::get_epsilon_position() {
  return epsilon_position;
}

double Robot::get_deviation_linear_position() {
  return deviation_linear_position;
}

double Robot::get_deviation_angular_position() {
  return deviation_angular_position;
}

int Robot::get_threshold_linear_step_count() {
  return threshold_linear_step_count;
}

int Robot::get_threshold_angular_step_count() {
  return threshold_angular_step_count;
}

CommunicatorPtr Robot::get_communicator() {
  return communicator;
}

void Robot::set_behavior_run(boost::function<void()> behavior_run) {
  this->behavior_run = behavior_run;
}

void Robot::set_linear_velocity(double linear_velocity) {
  velocity->linear.x = linear_velocity;
}

void Robot::set_angular_velocity(double angular_velocity) {
  velocity->angular.z = angular_velocity;
}

void Robot::run() {
  ecl::MilliSleep milliSleep;
  milliSleep(delay * 1000); // Delay by seconds
  if (behavior_run)
    behavior_run();
}

void Robot::start_thread_keyboard() {
  struct termios raw;
  memcpy(&raw, &terminal, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(file_descriptor, TCSANOW, &raw);

  puts("Available commands");
  puts("---------------------------");
  puts("p: Toggle power.");
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

void Robot::process_keyboard_input(char c) {
  switch (c) {
  case kobuki_msgs::KeyboardInput::KeyCode_Down:
  case kobuki_msgs::KeyboardInput::KeyCode_Up:
  case kobuki_msgs::KeyboardInput::KeyCode_Right:
  case kobuki_msgs::KeyboardInput::KeyCode_Left:
    if (is_powered) {
      if (c == kobuki_msgs::KeyboardInput::KeyCode_Down
          && velocity->linear.x >= -linear_velocity_max) { // Decrease linear velocity
        velocity->linear.x -= linear_velocity_step;
      } else if (c == kobuki_msgs::KeyboardInput::KeyCode_Up
          && velocity->linear.x <= linear_velocity_max) { // Increase linear velocity
        velocity->linear.x += linear_velocity_step;
      } else if (c == kobuki_msgs::KeyboardInput::KeyCode_Right
          && velocity->angular.z >= -angular_velocity_max) { // Decrease angular velocity
        velocity->angular.z -= angular_velocity_step;
      } else if (c == kobuki_msgs::KeyboardInput::KeyCode_Left
          && velocity->angular.z <= angular_velocity_max) { // Increase angular velocity
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
  case 'i': {
    std::ostringstream info;
    info << "[Odom]: Pos(" << current_position->x << "," << current_position->y
        << "); " << "Ori(" << current_direction->x << ","
        << current_direction->y << ")";
    // TODO: Print obstacle detection
    ROS_INFO_STREAM(info.str());
    break;
  }
  case 'r':
  case 'z':
    ROS_INFO_STREAM("[Run]: " << "Start running");
    thread_run.start(&Robot::start_thread_run, *this);
    if (plan_name == "mstc_online") {
      thread_status.start(&Robot::start_thread_status, *this);
    }
    break;
  case 'c':
    if (plan_name == "mstc_online") {
      communicator->write_old_cells_message("");
      communicator->write_status_message("");
    }
    break;
  case 'q':
  case ' ':
    is_quitting = true;
    break;
  default:
    break;
  }
}

void Robot::start_thread_status() {
//  int count = 1;
  double time_counter = 0;
  clock_t this_time = clock();
  clock_t last_time = this_time;
  while (true) {
    this_time = clock();
    time_counter += (double) (this_time - last_time);
    last_time = this_time;
    if (time_counter > (double) (NUM_SECONDS * CLOCKS_PER_SEC)) {
      time_counter -= (double) (NUM_SECONDS * CLOCKS_PER_SEC);
      std::string status = communicator->create_status_message(
          boost::static_pointer_cast<IdentifiableCell>(
              communicator->get_current_cell()));
      communicator->write_status_message(status);
    }
  }
}

void Robot::start_thread_run() {
  run();
}

void Robot::enable_power() {
  stop();
  ROS_INFO("[Power]: Enabled");
  kobuki_msgs::MotorPower power;
  power.state = kobuki_msgs::MotorPower::ON;
  publisher_power.publish(power);
  is_powered = true;
}

void Robot::disable_power() {
  stop();
  ROS_INFO("[Power]: Disabled");
  kobuki_msgs::MotorPower power;
  power.state = kobuki_msgs::MotorPower::OFF;
  publisher_power.publish(power);
  is_powered = false;
}

void Robot::subscribe_odometry(const nav_msgs::OdometryConstPtr& odometry) {
  double px = odometry->pose.pose.position.x;
  double py = odometry->pose.pose.position.y;
  double ow = odometry->pose.pose.orientation.w;
  double oz = odometry->pose.pose.orientation.z;
  current_position->x = px + starting_point_x;
  current_position->y = py + starting_point_y;
  // Set initial direction to (1, 0)
  current_direction->x = ow * ow - oz * oz;
  current_direction->y = 2 * oz * ow;
}

void Robot::subscribe_laser(const sensor_msgs::LaserScanConstPtr& laser) {
  this->laser = laser;
}

}
