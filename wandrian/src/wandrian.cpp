/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: cslab
 */

#include <ros/package.h>
#include "../include/plans/stc/full_spiral_stc.hpp"
#include "../include/plans/stc/full_scan_stc.hpp"
#include "../include/plans/mstc/mstc_online.hpp"
#include "../include/plans/boustrophedon_online/boustrophedon_online.hpp"
#include "../include/plans/boustrophedon/boustrophedon.hpp"
#include "../include/plans/random_walk.hpp"
#include "../include/wandrian.hpp"

#define CLOCKWISE true
#define COUNTERCLOCKWISE false

// FIXME: Choose relevant values
#define EPSILON_ROTATIONAL_DIRECTION 0.06
#define EPSILON_MOTIONAL_DIRECTION 0.24
#define EPSILON_POSITION 0.06
#define TIME 0.5  // Loop after 0.5 seconds

using namespace wandrian::plans::stc;
using namespace wandrian::plans::mstc;
using namespace wandrian::plans::boustrophedon_online;
using namespace wandrian::plans::boustrophedon;

namespace wandrian {

Wandrian::Wandrian() :
    step_count(0), deviation_linear_count(0), deviation_angular_count(0) {
}

Wandrian::~Wandrian() {
}

bool Wandrian::initialize() {
  robot = RobotPtr(new Robot());
  return robot->initialize();
}

void Wandrian::spin() {
  if (robot->get_map_name() != "") { // Offline map
    map = MapPtr(new Map(find_map_path()));
    map->build();
  }
  robot->set_behavior_run(boost::bind(&Wandrian::wandrian_run, this));
  robot->spin();
}

void Wandrian::wandrian_run() {
  PointPtr starting_point = PointPtr(
      new Point(robot->get_starting_point_x(), robot->get_starting_point_y()));
  path.insert(path.end(), starting_point);
  actual_path.insert(actual_path.end(), starting_point);
  if (robot->get_plan_name() == "ss") {
    SpiralStcPtr spiral_stc = SpiralStcPtr(new SpiralStc());
    spiral_stc->initialize(starting_point, robot->get_tool_size());
    spiral_stc->set_behavior_go_to(
        boost::bind(&Wandrian::wandrian_go_to, this, _1, _2));
    spiral_stc->set_behavior_see_obstacle(
        boost::bind(&Wandrian::wandrian_see_obstacle, this, _1, _2));
    spiral_stc->cover();
  } else if (robot->get_plan_name() == "fss") {
    FullSpiralStcPtr full_spiral_stc = FullSpiralStcPtr(new FullSpiralStc());
    full_spiral_stc->initialize(starting_point, robot->get_tool_size());
    full_spiral_stc->set_behavior_go_to(
        boost::bind(&Wandrian::wandrian_go_to, this, _1, _2));
    full_spiral_stc->set_behavior_see_obstacle(
        boost::bind(&Wandrian::wandrian_see_obstacle, this, _1, _2));
    full_spiral_stc->cover();
  } else if (robot->get_plan_name() == "fss2") {
    FullScanStcPtr full_scan_stc = FullScanStcPtr(new FullScanStc());
    full_scan_stc->initialize(starting_point, robot->get_tool_size());
    full_scan_stc->set_behavior_go_to(
        boost::bind(&Wandrian::wandrian_go_to, this, _1, _2));
    full_scan_stc->set_behavior_see_obstacle(
        boost::bind(&Wandrian::wandrian_see_obstacle, this, _1, _2));
    full_scan_stc->cover();
  } else if (robot->get_plan_name() == "mo") {
    MstcOnlinePtr mstc_online = MstcOnlinePtr(new MstcOnline());
    mstc_online->initialize(starting_point, robot->get_tool_size(),
        robot->get_communicator());
    mstc_online->set_behavior_go_to(
        boost::bind(&Wandrian::wandrian_go_to, this, _1, _2));
    mstc_online->set_behavior_see_obstacle(
        boost::bind(&Wandrian::wandrian_see_obstacle, this, _1, _2));
    mstc_online->cover();
  } else if (robot->get_plan_name() == "bo") {
    BoustrophedonOnlinePtr boustrophedon_online = BoustrophedonOnlinePtr(
        new BoustrophedonOnline());
    boustrophedon_online->initialize(starting_point, robot->get_tool_size());
    boustrophedon_online->set_behavior_go_to(
        boost::bind(&Wandrian::wandrian_go_to, this, _1, _2));
    boustrophedon_online->set_behavior_see_obstacle(
        boost::bind(&Wandrian::wandrian_see_obstacle, this, _1, _2));
    boustrophedon_online->cover();
  } else if (robot->get_plan_name() == "b") {
    BoustrophedonPtr boustrophedon = BoustrophedonPtr(new Boustrophedon());
    boustrophedon->initialize(starting_point, robot->get_tool_size(),
        find_map_path());
    boustrophedon->set_behavior_go_to(
        boost::bind(&Wandrian::wandrian_go_to, this, _1, _2));
    boustrophedon->cover();
  } else if (robot->get_plan_name() == "rw") {
    RandomWalkPtr random_walk = RandomWalkPtr(new RandomWalk());
    random_walk->set_behavior_rotate_randomly(
        boost::bind(&Wandrian::wandrian_rotate_randomly, this));
    random_walk->set_behavior_go_straight(
        boost::bind(&Wandrian::wandrian_go_straight, this));
    random_walk->cover();
  } else {
    std::list<PointPtr> path = map->get_path();
    for (std::list<PointPtr>::iterator p = path.begin(); p != path.end(); p++) {
      wandrian_go_to((*p));
    }
  }
  robot->stop();
}

bool Wandrian::wandrian_go_to(PointPtr position, bool flexibility) {
  double deviation_linear_position = robot->get_deviation_linear_position();
  double deviation_angular_position = robot->get_deviation_angular_position();
  PointPtr last_position = path.back();
  VectorPtr direction = (position - last_position) / (position % last_position);
  PointPtr actual_position = actual_path.back() + (position - last_position)
      + direction * deviation_linear_position * deviation_linear_count
      + (deviation_angular_position > 0 ? +direction : -direction)
          * std::abs(deviation_angular_position) * deviation_angular_count;
  std::cout << " (" << actual_position->x << "," << actual_position->y << ")";
  bool forward;
  forward = rotate_to(actual_position, flexibility);
  // Assume there is no obstacles, robot go straight
  go(forward);
  double epsilon_direction = robot->get_epsilon_motional_direction();
  double epsilon_position = robot->get_epsilon_position();
  if (epsilon_direction <= 0)
    epsilon_direction = EPSILON_MOTIONAL_DIRECTION;
  if (epsilon_position <= 0)
    epsilon_position = EPSILON_POSITION;
  while (true) {
    // TODO: Detect moving obstacle
    // Check current_position + k * current_direction == new_position
    VectorPtr direction = (actual_position - robot->get_current_position())
        / (actual_position % robot->get_current_position());
    if (forward ?
        (!(std::abs(direction->x - robot->get_current_direction()->x)
            < epsilon_direction
            && std::abs(direction->y - robot->get_current_direction()->y)
                < epsilon_direction)) :
        (!(std::abs(direction->x + robot->get_current_direction()->x)
            < epsilon_direction
            && std::abs(direction->y + robot->get_current_direction()->y)
                < epsilon_direction))) { // Wrong direction
      forward = rotate_to(actual_position, flexibility);
      go(forward);
    }
    if (std::abs(actual_position->x - robot->get_current_position()->x)
        < epsilon_position
        && std::abs(actual_position->y - robot->get_current_position()->y)
            < epsilon_position) { // Reached the new position
      break;
    }
  }
  path.insert(path.end(), position);
  actual_path.insert(actual_path.end(), actual_position);
  std::cout << " [" << robot->get_current_position()->x << ","
      << robot->get_current_position()->y << "]\n";
  return true;
}

bool Wandrian::wandrian_see_obstacle(VectorPtr direction, double distance) {
  RectanglePtr boundary;
  std::list<RectanglePtr> obstacles;
  PointPtr new_position = path.back() + direction * distance;
  if (robot->get_map_name() != "") { // Offline map
    boundary = map->get_boundary();
    obstacles = map->get_obstacles();
  } else {
    boundary = robot->get_map_boundary();
  }
  if (boundary && boundary->get_width() > 0 && boundary->get_height() > 0)
    if (new_position->x
        >= boundary->get_center()->x + boundary->get_width() / 2 - EPSILON
        || new_position->x
            <= boundary->get_center()->x - boundary->get_width() / 2 + EPSILON
        || new_position->y
            >= boundary->get_center()->y + boundary->get_height() / 2 - EPSILON
        || new_position->y
            <= boundary->get_center()->y - boundary->get_height() / 2
                + EPSILON) {
      return true;
    }
  if (obstacles.size() > 0) { // Offline
    for (std::list<RectanglePtr>::iterator o = obstacles.begin();
        o != obstacles.end(); o++) {
      CellPtr obstacle = boost::static_pointer_cast<Cell>(*o);
      if (new_position->x
          >= obstacle->get_center()->x - obstacle->get_size() / 2 - EPSILON
          && new_position->x
              <= obstacle->get_center()->x + obstacle->get_size() / 2 + EPSILON
          && new_position->y
              >= obstacle->get_center()->y - obstacle->get_size() / 2 - EPSILON
          && new_position->y
              <= obstacle->get_center()->y + obstacle->get_size() / 2
                  + EPSILON) {
        return true;
      }
    }
    return false;
  } else { // Online
    double angle = direction ^ robot->get_current_direction();
    return robot->see_obstacle(angle, distance);
  }
}

void Wandrian::wandrian_rotate_randomly() {
  this->robot->set_bumper_state_automatic(true);
  double angle = (double) rand() / RAND_MAX * 2 * M_PI;
  std::cout << "rotate randomly: " << angle << "\n";
  rotate_to(VectorPtr(new Vector(angle)), STRICTLY);
}

void Wandrian::wandrian_go_straight() {
  // this->robot->set_bumper_state_automatic(false);
  // std::cout << "go straight \n";
  // this->robot->set_linear_velocity(this->robot->get_linear_velocity());
  // while (true){
  //   if (this->robot->get_bumper_state() == kobuki_msgs::BumperEvent::PRESSED) {
  //     this->robot->set_linear_velocity(-this->robot->get_linear_velocity());
  //     int count = 1;
  //     int old_count = 0;
  //     double time_counter = 0;
  //     clock_t this_time = clock();
  //     clock_t last_time = this_time;
  //     while (true) {
  //       this_time = clock();
  //       time_counter += (double) (this_time - last_time);
  //       last_time = this_time;
  //       if (time_counter > (double) (TIME * CLOCKS_PER_SEC)) {
  //         time_counter -= (double) (TIME * CLOCKS_PER_SEC);
  //         break;
  //       }
  //     }
  //     break;
  //   }
  // }
  // this->robot->stop();
  // this->robot->set_bumper_state(kobuki_msgs::BumperEvent::RELEASED);
  robot->set_bumper_state_automatic(false);
  std::cout << "go straight \n";
  robot->set_linear_velocity(robot->get_linear_velocity());
  while (true)
    if (robot->get_bumper_state() == kobuki_msgs::BumperEvent::PRESSED)
      break;
  robot->stop();
  robot->set_bumper_state(kobuki_msgs::BumperEvent::RELEASED);
}

bool Wandrian::rotate_to(PointPtr position, bool flexibility) {
  VectorPtr direction = (position - robot->get_current_position())
      / (position % robot->get_current_position());
  return rotate_to(direction, flexibility);
}

bool Wandrian::rotate_to(VectorPtr direction, bool flexibility) {
  double angle = direction ^ robot->get_current_direction();
  double epsilon = robot->get_epsilon_rotational_direction();
  if (epsilon <= 0)
    epsilon = EPSILON_ROTATIONAL_DIRECTION;
  bool will_move_forward =
      (flexibility != FLEXIBLY) ? true : std::abs(angle) < M_PI_2;
  if (angle > epsilon) {
    step_count = 1;
    robot->stop();
    rotate(will_move_forward ? COUNTERCLOCKWISE : CLOCKWISE);
  } else if (angle < -epsilon) {
    step_count = 1;
    robot->stop();
    rotate(will_move_forward ? CLOCKWISE : COUNTERCLOCKWISE);
  } else {
    if (step_count > robot->get_threshold_linear_step_count()) {
      deviation_linear_count++;
    }
    if (step_count < robot->get_threshold_angular_step_count()) {
      step_count++;
      deviation_angular_count++;
    }
    return true;
  }
  while (true) {
    if (will_move_forward ?
        (std::abs(direction->x - robot->get_current_direction()->x) < epsilon
            && std::abs(direction->y - robot->get_current_direction()->y)
                < epsilon) :
        (std::abs(direction->x + robot->get_current_direction()->x) < epsilon
            && std::abs(direction->y + robot->get_current_direction()->y)
                < epsilon)) {
      robot->stop();
      break;
    }
  }
  return will_move_forward;
}

void Wandrian::go(bool forward) {
  double linear_velocity = robot->get_linear_velocity();
  robot->set_linear_velocity(forward ? linear_velocity : -linear_velocity);
}

void Wandrian::rotate(bool rotation_is_clockwise) {
  robot->set_angular_velocity(
      rotation_is_clockwise ?
          -robot->get_negative_angular_velocity() :
          robot->get_positive_angular_velocity());
}

std::string Wandrian::find_map_path() {
  return ros::package::getPath("wandrian") + "/worlds/" + robot->get_map_name()
      + ".map";
}

}
