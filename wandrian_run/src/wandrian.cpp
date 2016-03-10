/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#include "../include/wandrian.hpp"
#include "../include/plans/spiral_stc/spiral_stc.hpp"
#include "../include/plans/spiral_stc/full_spiral_stc.hpp"
#include "../include/plans/mstc_online/mstc_online.hpp"

#define CLOCKWISE true
#define COUNTERCLOCKWISE false

// TODO: Choose relevant epsilon values
#define EPS_ORI_TO_ROTATE 0.06
#define EPS_ORI_TO_MOVE 4 * EPS_ORI_TO_ROTATE
#define EPS_POS 0.06

using namespace wandrian::plans::spiral_stc;
using namespace wandrian::plans::mstc_online;

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
    SpiralStcPtr spiral_stc = SpiralStcPtr(new SpiralStc());
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
    FullSpiralStcPtr full_spiral_stc = FullSpiralStcPtr(new FullSpiralStc());
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
    MstcOnlinePtr mstc_online = MstcOnlinePtr(new MstcOnline());
    mstc_online->initialize(
        PointPtr(
            new Point(robot->get_starting_point_x(),
                robot->get_starting_point_y())), robot->get_tool_size(),
        robot->get_communicator());
    mstc_online->set_behavior_go_to(
        boost::bind(&Wandrian::mstc_online_go_to, this, _1, _2));
    mstc_online->set_behavior_see_obstacle(
        boost::bind(&Wandrian::mstc_online_see_obstacle, this, _1, _2));
    mstc_online->cover();
  }
}

bool Wandrian::spiral_stc_go_to(PointPtr position, bool flexibility) {
  return go_to(position, flexibility);
}

bool Wandrian::spiral_stc_see_obstacle(VectorPtr direction, double distance) {
  // TODO: Correctly check whether obstacle is near or not
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

bool Wandrian::go_to(PointPtr new_position, bool flexibility) {
  bool forward;
  forward = rotate_to(new_position, flexibility);
  // Assume there is no obstacle, robot go straight
  go(forward);
  while (true) {
    // TODO: Moving obstacle detected, dodging
    if (robot->get_obstacle_movement() != STOPPING) {
      dodge();
    }
    // Check current_position + k * current_direction == new_position
    VectorPtr direction = (new_position - robot->get_current_position())
        / (new_position % robot->get_current_position());
    if (forward ?
        (!(std::abs(direction->x - robot->get_current_direction()->x)
            < EPS_ORI_TO_MOVE
            && std::abs(direction->y - robot->get_current_direction()->y)
                < EPS_ORI_TO_MOVE)) :
        (!(std::abs(direction->x + robot->get_current_direction()->x)
            < EPS_ORI_TO_MOVE
            && std::abs(direction->y + robot->get_current_direction()->y)
                < EPS_ORI_TO_MOVE))) { // Wrong direction
      robot->stop();
      forward = rotate_to(new_position, flexibility);
      go(forward);
    }
    if (std::abs(new_position->x - robot->get_current_position()->x) < EPS_POS
        && std::abs(new_position->y - robot->get_current_position()->y)
            < EPS_POS) { // Reached the new position
      robot->stop();
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
  bool will_move_forward =
      (flexibility != FLEXIBLY) ? true : std::abs(angle) < M_PI_2;
  if (angle > EPS_ORI_TO_ROTATE)
    rotate(will_move_forward ? COUNTERCLOCKWISE : CLOCKWISE);
  else if (angle < -EPS_ORI_TO_ROTATE)
    rotate(will_move_forward ? CLOCKWISE : COUNTERCLOCKWISE);
  while (true) {
    if (will_move_forward ?
        (std::abs(new_direction->x - robot->get_current_direction()->x)
            < EPS_ORI_TO_ROTATE
            && std::abs(new_direction->y - robot->get_current_direction()->y)
                < EPS_ORI_TO_ROTATE) :
        (std::abs(new_direction->x + robot->get_current_direction()->x)
            < EPS_ORI_TO_ROTATE
            && std::abs(new_direction->y + robot->get_current_direction()->y)
                < EPS_ORI_TO_ROTATE)) {
      robot->stop();
      break;
    }
  }
  return will_move_forward;
}

void Wandrian::go(bool forward) {
  double linear_velocity_step = robot->get_linear_velocity_step();
  robot->set_linear_velocity(
      forward ? linear_velocity_step : -linear_velocity_step);
}

void Wandrian::rotate(bool rotation_is_clockwise) {
  double angular_velocity_step = robot->get_angular_velocity_step();
  robot->set_angular_velocity(
      rotation_is_clockwise ? -angular_velocity_step : angular_velocity_step);
}

void Wandrian::dodge() {
  while (robot->get_obstacle_movement() == COMING) {
    robot->stop();
  }
}

}
