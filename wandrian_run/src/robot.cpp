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
#define PROPORTION_RANGES_COUNT 0.5
#define PROPORTION_RANGES_SUM 0.2
#define AUGMENTATION_FACTOR_RANGE 2.0

namespace wandrian {

Robot::Robot() :
    tool_size(0), starting_point_x(0), starting_point_y(0), space_center_x(0), space_center_y(
        0), space_boundary_width(0), space_boundary_height(0), linear_velocity(
        0), angular_velocity(0), proportion_ranges_count(0), proportion_ranges_sum(
        0), augmentation_factor_range(0), epsilon_rotational_orientation(0), epsilon_motional_orientation(
        0), epsilon_position(0), current_position(new Point()), current_direction(
        new Vector()), obstacle_movement(STOPPING), linear_velocity_step(0), linear_velocity_max(
        0), angular_velocity_step(0), angular_velocity_max(0), velocity(
        new geometry_msgs::Twist()), laser_range(0), is_quitting(false), is_powered(
        false), is_zero_vel(true), is_logging(false), file_descriptor(0), last_position(
        new Point()), last_direction(new Vector()), laser_ray(0) {
  tcgetattr(file_descriptor, &terminal); // get terminal properties
}

Robot::~Robot() {
  tcsetattr(file_descriptor, TCSANOW, &terminal);
}

bool Robot::initialize() {
  ros::NodeHandle nh("~");

  nh.getParam("plan_name", plan_name);
  nh.getParam("robot_name", robot_name);
  nh.getParam("tool_size", tool_size);
  nh.getParam("starting_point_x", starting_point_x);
  nh.getParam("starting_point_y", starting_point_y);
  nh.getParam("space_center_x", space_center_x);
  nh.getParam("space_center_y", space_center_y);
  nh.getParam("space_boundary_width", space_boundary_width);
  nh.getParam("space_boundary_height", space_boundary_height);
  nh.getParam("linear_velocity", linear_velocity);
  nh.getParam("angular_velocity", angular_velocity);
  nh.getParam("proportion_ranges_count", proportion_ranges_count);
  nh.getParam("proportion_ranges_sum", proportion_ranges_sum);
  nh.getParam("augmentation_factor_range", augmentation_factor_range);
  nh.getParam("epsilon_rotational_orientation", epsilon_rotational_orientation);
  nh.getParam("epsilon_motional_orientation", epsilon_motional_orientation);
  nh.getParam("epsilon_position", epsilon_position);

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

  if (proportion_ranges_count <= 0 || proportion_ranges_count >= 1)
    proportion_ranges_count = PROPORTION_RANGES_COUNT;
  if (augmentation_factor_range <= 0)
    augmentation_factor_range = AUGMENTATION_FACTOR_RANGE;

  obstacles[AT_RIGHT_SIDE] = false;
  obstacles[IN_FRONT] = false;
  obstacles[AT_LEFT_SIDE] = false;

  timer_laser = nh.createTimer(ros::Duration(0.5),
      boost::bind(&Robot::start_timer_laser, this, _1));

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
  laser_range = tool_size / 2;

  ecl::MilliSleep millisleep;
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
  }
  thread_keyboard.join();
}

void Robot::stop() {
  decelerate(0);
}

void Robot::decelerate(double linear_proportion, double angular_proportion) {
  velocity->linear.x *= linear_proportion;
  velocity->angular.z *= angular_proportion;
  // Force decelerate
  publisher_velocity.publish(velocity);
}

std::string Robot::get_plan_name() {
  return plan_name;
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

RectanglePtr Robot::get_space_boundary() {
  return RectanglePtr(
      new Rectangle(PointPtr(new Point(space_center_x, space_center_y)),
          space_boundary_width, space_boundary_height));
}

PointPtr Robot::get_current_position() {
  return current_position;
}

VectorPtr Robot::get_current_direction() {
  return current_direction;
}

bool* Robot::get_obstacles() {
  return obstacles;
}

ObstacleMovement Robot::get_obstacle_movement() {
  return obstacle_movement;
}

double Robot::get_linear_velocity() {
  return linear_velocity;
}

double Robot::get_angular_velocity() {
  return angular_velocity;
}

double Robot::get_epsilon_rotational_orientation() {
  return epsilon_rotational_orientation;
}

double Robot::get_epsilon_motional_orientation() {
  return epsilon_motional_orientation;
}

double Robot::get_epsilon_position() {
  return epsilon_position;
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

void Robot::set_laser_range(double laser_range) {
  this->laser_range = laser_range;
}

void Robot::run() {
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
    info << "[Laser]: Obs(" << obstacles[AT_RIGHT_SIDE] << ","
        << obstacles[IN_FRONT] << "," << obstacles[AT_LEFT_SIDE] << ")";
    info << " " << ros::Time::now();
    ROS_INFO_STREAM(info.str());
    break;
  }
  case 'r':
  case ' ':
    ROS_INFO_STREAM("[Run]: " << "Start running");
    thread_run.start(&Robot::start_thread_run, *this);
    break;
  case 'c':
    if (plan_name == "mstc_online") {
      communicator->write_old_cells_message("");
      communicator->write_status_message("");
    }
    break;
  case 'q':
    is_quitting = true;
    break;
  default:
    is_quitting = true;
    break;
  }
}

void Robot::start_thread_run() {
  run();
}

void Robot::start_timer_laser(const ros::TimerEvent&) {
  if (laser_ranges.size() <= 0 || last_laser_ranges.size() <= 0) {
    last_laser_ranges = laser_ranges;
    return;
  }
  // Estimate laser ranges
  std::vector<float> estimated_laser_ranges;
  VectorPtr translation = current_position - last_position;
  for (int i = 0; i <= last_laser_ranges.size() - 1; i++) {
    VectorPtr angle = VectorPtr(
        new Vector(
            (i - last_laser_ranges.size() / 2) * laser_ray
                + last_direction->get_angle()));
    estimated_laser_ranges.push_back(
        (float) (last_laser_ranges[i]
            + translation->get_magnitude() * cos(angle ^ +(+translation))));
  }
  double rotation = current_direction ^ last_direction;
  int rays = rotation / laser_ray;
  double ranges_sum, estimated_ranges_sum = 0;
  for (int i = rotation > 0 ? 0 : estimated_laser_ranges.size() - 1;
      rotation > 0 ? i <= estimated_laser_ranges.size() - 1 - rays : i >= -rays;
      rotation > 0 ? i++ : i--) {
    estimated_laser_ranges[i] = estimated_laser_ranges[i + rays];
    if (isnan(laser_ranges[i]) || isnan(estimated_laser_ranges[i])
        || isinf(laser_ranges[i]) || isinf(estimated_laser_ranges[i]))
      continue;
    ranges_sum += laser_ranges[i];
    estimated_ranges_sum += estimated_laser_ranges[i];
  }
  if (ranges_sum - estimated_ranges_sum
      >= proportion_ranges_sum * estimated_ranges_sum)
    obstacle_movement = LEAVING;
  else if (estimated_ranges_sum - ranges_sum
      >= proportion_ranges_sum * ranges_sum)
    obstacle_movement = COMING;
  else
    obstacle_movement = STOPPING;
  last_position->x = current_position->x;
  last_position->y = current_position->y;
  last_direction->x = current_direction->x;
  last_direction->y = current_direction->y;
  last_laser_ranges = laser_ranges;
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
  const double ANGLE_RIGHT_MIN = -2.0 / 3.0 * M_PI;
  const double ANGLE_RIGHT_MAX = -1.0 / 3.0 * M_PI;
  const double ANGLE_IN_FRONT_MIN = -1.0 / 6.0 * M_PI;
  const double ANGLE_IN_FRONT_MAX = 1.0 / 6.0 * M_PI;
  const double ANGLE_LEFT_MIN = 1.0 / 3.0 * M_PI;
  const double ANGLE_LEFT_MAX = 2.0 / 3.0 * M_PI;

  sensor_msgs::LaserScan::_ranges_type laser_ranges = laser->ranges;
  if (laser_ray == 0.0)
    laser_ray = (laser->angle_max - laser->angle_min)
        / (double) laser_ranges.size();
  this->laser_ranges = laser_ranges;
  int range_right_min = (ANGLE_RIGHT_MIN - laser->angle_min) / laser_ray;
  int range_right_max = (ANGLE_RIGHT_MAX - laser->angle_min) / laser_ray;
  int range_in_front_min = (ANGLE_IN_FRONT_MIN - laser->angle_min) / laser_ray;
  int range_in_front_max = (ANGLE_IN_FRONT_MAX - laser->angle_min) / laser_ray;
  int range_left_min = (ANGLE_LEFT_MIN - laser->angle_min) / laser_ray;
  int range_left_max = (ANGLE_LEFT_MAX - laser->angle_min) / laser_ray;

  // Detect obstacle
  if (laser->angle_min <= ANGLE_RIGHT_MAX
      && laser->angle_max >= ANGLE_RIGHT_MIN) {
    int count = 0;
    range_right_min = range_right_min >= 0 ? range_right_min : 0;
    range_right_max =
        range_right_max <= laser_ranges.size() ?
            range_right_max : laser_ranges.size();
    for (int i = range_right_min; i <= range_right_max - 1; i++) {
      if (laser_ranges[i] <= augmentation_factor_range * laser_range)
        count++;
    }
    obstacles[AT_RIGHT_SIDE] = (count
        >= (range_right_max - range_right_min) * proportion_ranges_count);
  }
  if (laser->angle_min <= ANGLE_IN_FRONT_MAX
      && laser->angle_max >= ANGLE_IN_FRONT_MIN) {
    int count = 0;
    range_in_front_min = range_in_front_min >= 0 ? range_in_front_min : 0;
    range_in_front_max =
        range_in_front_max <= laser_ranges.size() ?
            range_in_front_max : laser_ranges.size();
    for (int i = range_in_front_min; i <= range_in_front_max - 1; i++) {
      if (laser_ranges[i] <= augmentation_factor_range * laser_range)
        count++;
    }
    obstacles[IN_FRONT] = (count
        >= (range_in_front_max - range_in_front_min) * proportion_ranges_count);
  }
  if (laser->angle_min <= ANGLE_LEFT_MAX
      && laser->angle_max >= ANGLE_LEFT_MIN) {
    int count = 0;
    range_left_min = range_left_min >= 0 ? range_left_min : 0;
    range_left_max =
        range_left_max <= laser_ranges.size() ?
            range_left_max : laser_ranges.size();
    for (int i = range_left_min; i <= range_left_max - 1; i++) {
      if (laser_ranges[i] <= augmentation_factor_range * laser_range)
        count++;
    }
    obstacles[AT_LEFT_SIDE] = (count
        >= (range_left_max - range_left_min) * proportion_ranges_count);
  }

  if (is_logging) {
    std::string ori;
    if (obstacles[AT_RIGHT_SIDE])
      ori += "Right,";
    if (obstacles[IN_FRONT])
      ori += "Front,";
    if (obstacles[AT_LEFT_SIDE])
      ori += "Left,";
    if (ori.length() > 0)
      ori = ori.substr(0, ori.length() - 1);
    std::string mov;
    if (obstacle_movement == LEAVING)
      mov += " LEAVING";
    else if (obstacle_movement == COMING)
      mov += " COMING";
    ROS_WARN_STREAM("[Laser]: Obs(" << ori << ")" << mov);
  }
}

}
