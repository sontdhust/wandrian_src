/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#include "../include/wandrian.hpp"
#include "../include/plans/spiral_stc/spiral_stc.hpp"

#define CLOCKWISE true
#define COUNTERCLOCKWISE false

// TODO: Choose relevant epsilon values
#define EPS_ORI_TO_ROTATE 0.06
#define EPS_ORI_TO_MOVE 4 * EPS_ORI_TO_ROTATE
#define EPS_POS 0.06

namespace wandrian {

void Wandrian::run() {
  if (plan == "spiral_stc") {
    spiral_stc = SpiralStcPtr(new SpiralStc());
    spiral_stc->initialize(
        PointPtr(new Point(starting_point_x, starting_point_y)), robot_size);
    spiral_stc->set_behavior_go_to(
        boost::bind(&Wandrian::spiral_stc_go_to, this, _1, _2));
    spiral_stc->set_behavior_see_obstacle(
        boost::bind(&Wandrian::spiral_stc_see_obstacle, this, _1, _2));
    return spiral_stc->cover();
  }
}

bool Wandrian::go_to(PointPtr new_position, bool flexibly) {
  bool forward;
  forward = rotate(new_position, flexibly);
  move(forward);
  while (true) {
    // Check current_position + k * current_orientation == new_position
    Vector direction_vector = (*new_position - *current_position)
        / (*new_position % *current_position);
    if (forward ?
        (!(std::abs(direction_vector.x - current_orientation->x)
            < EPS_ORI_TO_MOVE
            && std::abs(direction_vector.y - current_orientation->y)
                < EPS_ORI_TO_MOVE)) :
        (!(std::abs(direction_vector.x + current_orientation->x)
            < EPS_ORI_TO_MOVE
            && std::abs(direction_vector.y + current_orientation->y)
                < EPS_ORI_TO_MOVE))) {
      stop();
      forward = rotate(new_position, flexibly);
      move(forward);
    }

    if (distance_to_obstacle[IN_FRONT] > robot_size / 2) {
      if (std::abs(new_position->x - current_position->x) < EPS_POS
          && std::abs(new_position->y - current_position->y) < EPS_POS) {
        stop();
        break;
      }
    } else {
      // Obstacle
      stop();
      return false;
    }
  }
  return true;
}

bool Wandrian::rotate(PointPtr new_position, bool flexibly) {
  VectorPtr new_orientation = VectorPtr(
      new Vector(
          (*new_position - *current_position)
              / (*new_position % *current_position)));
  double angle = *new_orientation ^ *current_orientation;

  bool will_move_forward = !flexibly ? true : std::abs(angle) < M_PI_2;
  if (angle > EPS_ORI_TO_ROTATE)
    rotate(will_move_forward ? COUNTERCLOCKWISE : CLOCKWISE);
  else if (angle < -EPS_ORI_TO_ROTATE)
    rotate(will_move_forward ? CLOCKWISE : COUNTERCLOCKWISE);
  while (true) {
    if (will_move_forward ?
        (std::abs(new_orientation->x - current_orientation->x)
            < EPS_ORI_TO_ROTATE
            && std::abs(new_orientation->y - current_orientation->y)
                < EPS_ORI_TO_ROTATE) :
        (std::abs(new_orientation->x + current_orientation->x)
            < EPS_ORI_TO_ROTATE
            && std::abs(new_orientation->y + current_orientation->y)
                < EPS_ORI_TO_ROTATE)) {
      stop();
      break;
    }
  }
  return will_move_forward;
}

void Wandrian::rotate(bool clockwise) {
  if (clockwise)
    velocity->angular.z = -angular_vel_step;
  else
    velocity->angular.z = angular_vel_step;
}

void Wandrian::move(bool forward) {
  if (forward)
    velocity->linear.x = linear_vel_step;
  else
    velocity->linear.x = -linear_vel_step;
}

bool Wandrian::spiral_stc_go_to(PointPtr position, bool flexibly) {
  return go_to(position, flexibly);
}

bool Wandrian::spiral_stc_see_obstacle(VectorPtr orientation, double step) {
  // TODO: Correctly check whether obstacle is near or not
  double angle = *orientation ^ *current_orientation;
  std::cout << "      ang: " << angle << "; ori: " << orientation->x << ","
      << orientation->y << "\n";
  return
      (std::abs(angle) <= M_PI_4) ?
          distance_to_obstacle[IN_FRONT] <= step * robot_size :
          (angle > M_PI_4 ?
              distance_to_obstacle[AT_LEFT_SIDE] <= step * robot_size :
              distance_to_obstacle[AT_RIGHT_SIDE] <= step * robot_size);
}

}
