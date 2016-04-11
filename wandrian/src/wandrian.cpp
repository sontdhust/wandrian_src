/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: cslab
 */

#include <ros/package.h>
#include "../include/plans/stc/full_spiral_stc.hpp"
#include "../include/plans/mstc/full_mstc_online.hpp"
#include "../include/plans/mstc/mstc_online.hpp"
#include "../include/plans/boustrophedon_online/boustrophedon_online.hpp"
#include "../include/plans/boustrophedon/boustrophedon.hpp"
#include "../include/wandrian.hpp"

#define CLOCKWISE true
#define COUNTERCLOCKWISE false

// FIXME: Choose relevant values
#define EPSILON_ROTATIONAL_DIRECTION 0.06
#define EPSILON_MOTIONAL_DIRECTION 0.24
#define EPSILON_POSITION 0.06

using namespace wandrian::plans::stc;
using namespace wandrian::plans::mstc;
using namespace wandrian::plans::boustrophedon_online;
using namespace wandrian::plans::boustrophedon;

namespace wandrian {

Wandrian::Wandrian() {
}

Wandrian::~Wandrian() {
}

bool Wandrian::initialize() {
  robot = RobotPtr(new Robot());
  return robot->initialize();
}

void Wandrian::spin() {
  robot->set_behavior_run(boost::bind(&Wandrian::wandrian_run, this));
  robot->spin();
}

void Wandrian::wandrian_run() {
  if (robot->get_plan_name() == "spiral_stc") {
    plan = SpiralStcPtr(new SpiralStc());
    SpiralStcPtr spiral_stc = boost::static_pointer_cast<SpiralStc>(plan);
    spiral_stc->initialize(
        PointPtr(
            new Point(robot->get_starting_point_x(),
                robot->get_starting_point_y())), robot->get_tool_size());
    spiral_stc->set_behavior_go_to(
        boost::bind(&Wandrian::spiral_stc_go_to, this, _1, _2));
    spiral_stc->set_behavior_see_obstacle(
        boost::bind(&Wandrian::spiral_stc_see_obstacle, this, _1, _2));
    spiral_stc->cover();
  } else if (robot->get_plan_name() == "full_spiral_stc") {
    plan = FullSpiralStcPtr(new FullSpiralStc());
    FullSpiralStcPtr full_spiral_stc =
        boost::static_pointer_cast<FullSpiralStc>(plan);
    full_spiral_stc->initialize(
        PointPtr(
            new Point(robot->get_starting_point_x(),
                robot->get_starting_point_y())), robot->get_tool_size());
    full_spiral_stc->set_behavior_go_to(
        boost::bind(&Wandrian::full_spiral_stc_go_to, this, _1, _2));
    full_spiral_stc->set_behavior_see_obstacle(
        boost::bind(&Wandrian::full_spiral_stc_see_obstacle, this, _1, _2));
    full_spiral_stc->cover();
  } else if (robot->get_plan_name() == "mstc_online") {
    plan = MstcOnlinePtr(new MstcOnline());
    MstcOnlinePtr mstc_online = boost::static_pointer_cast<MstcOnline>(plan);
    mstc_online->initialize(
        PointPtr(
            new Point(robot->get_starting_point_x(),
                robot->get_starting_point_y())), robot->get_tool_size(),
        boost::static_pointer_cast<MstcCommunicator>(
            robot->get_communicator()));
    mstc_online->set_behavior_go_to(
        boost::bind(&Wandrian::mstc_online_go_to, this, _1, _2));
    mstc_online->set_behavior_see_obstacle(
        boost::bind(&Wandrian::mstc_online_see_obstacle, this, _1, _2));
    mstc_online->cover();
//  } else if (robot->get_plan_name() == "full_mstc_online") {
//    plan = FullMstcOnlinePtr(new FullMstcOnline());
//    FullMstcOnlinePtr full_mstc_online = boost::static_pointer_cast<FullMstcOnline>(plan);
//    full_mstc_online->initialize(
//        PointPtr(
//            new Point(robot->get_starting_point_x(),
//                robot->get_starting_point_y())), robot->get_tool_size(),
//        robot->get_communicator());
//    full_mstc_online->set_behavior_go_to(
//        boost::bind(&Wandrian::full_mstc_online_go_to, this, _1, _2));
//    full_mstc_online->set_behavior_see_obstacle(
//        boost::bind(&Wandrian::full_mstc_online_see_obstacle, this, _1, _2));
//    full_mstc_online->cover();
  } else if (robot->get_plan_name() == "boustrophedon_online") {
    plan = BoustrophedonOnlinePtr(new BoustrophedonOnline());
    BoustrophedonOnlinePtr boustrophedon_online = boost::static_pointer_cast<
        BoustrophedonOnline>(plan);
    boustrophedon_online->initialize(
        PointPtr(
            new Point(robot->get_starting_point_x(),
                robot->get_starting_point_y())), robot->get_tool_size());
    boustrophedon_online->set_behavior_go_to(
        boost::bind(&Wandrian::boustrophedon_online_go_to, this, _1, _2));
    boustrophedon_online->set_behavior_see_obstacle(
        boost::bind(&Wandrian::boustrophedon_online_see_obstacle, this, _1,
            _2));
    boustrophedon_online->cover();
  } else if (robot->get_plan_name() == "boustrophedon") {
    BoustrophedonPtr boustrophedon = BoustrophedonPtr(new Boustrophedon());
    boustrophedon->initialize(
        PointPtr(
            new Point(robot->get_starting_point_x(),
                robot->get_starting_point_y())), robot->get_tool_size(),
        find_map_path());
    boustrophedon->set_behavior_go_to(
        boost::bind(&Wandrian::boustrophedon_go_to, this, _1, _2));
    boustrophedon->cover();
  }
  robot->stop();
}

bool Wandrian::spiral_stc_go_to(PointPtr position, bool flexibility) {
  return go_to(position, flexibility);
}

bool Wandrian::spiral_stc_see_obstacle(VectorPtr direction, double distance) {
  RectanglePtr boundary;
  std::list<RectanglePtr> obstacles;
  PointPtr last_position = plan->get_path().back();
  PointPtr new_position = last_position + direction * distance;
  if (robot->get_map_name() != "") { // Offline map
    MapPtr map = MapPtr(new Map(find_map_path()));
    map->build();
    boundary = map->get_boundary();
    obstacles = map->get_obstacles();
  } else {
    boundary = robot->get_map_boundary();
  }
  if (boundary)
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
  if (obstacles.size() > 0)
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
  double angle = direction ^ robot->get_current_direction();
  if (std::abs(angle) <= 3 * M_PI_4)
    return
        (std::abs(angle) <= M_PI_4) ?
            see_obstacle(IN_FRONT, distance) :
            ((angle > M_PI_4) ?
                see_obstacle(AT_LEFT_SIDE, distance) :
                see_obstacle(AT_RIGHT_SIDE, distance));
  else {
    rotate_to(direction, STRICTLY);
    return see_obstacle(IN_FRONT, distance);
  }
}

bool Wandrian::full_spiral_stc_go_to(PointPtr position, bool flexibility) {
  return spiral_stc_go_to(position, flexibility);
}

bool Wandrian::full_spiral_stc_see_obstacle(VectorPtr direction,
    double distance) {
  return spiral_stc_see_obstacle(direction, distance);
}

bool Wandrian::mstc_online_go_to(PointPtr position, bool flexibility) {
  return spiral_stc_go_to(position, flexibility);
}

bool Wandrian::mstc_online_see_obstacle(VectorPtr direction, double distance) {
  return spiral_stc_see_obstacle(direction, distance);
}

bool Wandrian::full_mstc_online_go_to(PointPtr position, bool flexibility) {
  return spiral_stc_go_to(position, flexibility);
}

bool Wandrian::full_mstc_online_see_obstacle(VectorPtr direction,
    double distance) {
  return spiral_stc_see_obstacle(direction, distance);
}

bool Wandrian::boustrophedon_online_go_to(PointPtr position, bool flexibility) {
  return spiral_stc_go_to(position, flexibility);
}

bool Wandrian::boustrophedon_online_see_obstacle(VectorPtr direction,
    double distance) {
  return spiral_stc_see_obstacle(direction, distance);
}

bool Wandrian::boustrophedon_go_to(PointPtr position, bool flexibility) {
  return spiral_stc_go_to(position, flexibility);
}

bool Wandrian::go_to(PointPtr new_position, bool flexibility) {
  bool forward;
  forward = rotate_to(new_position, flexibility);
  // Assume there is no obstacle, robot go straight
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
    VectorPtr direction = (new_position - robot->get_current_position())
        / (new_position % robot->get_current_position());
    if (forward ?
        (!(std::abs(direction->x - robot->get_current_direction()->x)
            < epsilon_direction
            && std::abs(direction->y - robot->get_current_direction()->y)
                < epsilon_direction)) :
        (!(std::abs(direction->x + robot->get_current_direction()->x)
            < epsilon_direction
            && std::abs(direction->y + robot->get_current_direction()->y)
                < epsilon_direction))) { // Wrong direction
      forward = rotate_to(new_position, flexibility);
      go(forward);
    }
    if (std::abs(new_position->x - robot->get_current_position()->x)
        < epsilon_position
        && std::abs(new_position->y - robot->get_current_position()->y)
            < epsilon_position) { // Reached the new position
      break;
    }
  }
  return true;
}

bool Wandrian::see_obstacle(Orientation orientation, double distance) {
  robot->set_laser_range(distance);
  return robot->get_obstacles()[orientation];
}

bool Wandrian::rotate_to(PointPtr new_position, bool flexibility) {
  VectorPtr new_direction = (new_position - robot->get_current_position())
      / (new_position % robot->get_current_position());
  return rotate_to(new_direction, flexibility);
}

bool Wandrian::rotate_to(VectorPtr new_direction, bool flexibility) {
  double angle = new_direction ^ robot->get_current_direction();
  double epsilon = robot->get_epsilon_rotational_direction();
  if (epsilon <= 0)
    epsilon = EPSILON_ROTATIONAL_DIRECTION;
  bool will_move_forward =
      (flexibility != FLEXIBLY) ? true : std::abs(angle) < M_PI_2;
  if (angle > epsilon) {
    robot->stop();
    rotate(will_move_forward ? COUNTERCLOCKWISE : CLOCKWISE);
  } else if (angle < -epsilon) {
    robot->stop();
    rotate(will_move_forward ? CLOCKWISE : COUNTERCLOCKWISE);
  } else
    return true;
  while (true) {
    if (will_move_forward ?
        (std::abs(new_direction->x - robot->get_current_direction()->x)
            < epsilon
            && std::abs(new_direction->y - robot->get_current_direction()->y)
                < epsilon) :
        (std::abs(new_direction->x + robot->get_current_direction()->x)
            < epsilon
            && std::abs(new_direction->y + robot->get_current_direction()->y)
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
  double angular_velocity = robot->get_angular_velocity();
  robot->set_angular_velocity(
      rotation_is_clockwise ? -angular_velocity : angular_velocity);
}

void Wandrian::dodge() {
  while (robot->get_obstacle_movement() == COMING) {
    robot->stop();
  }
}

std::string Wandrian::find_map_path() {
  return ros::package::getPath("wandrian") + "/worlds/" + robot->get_map_name()
      + ".map";
}

}
