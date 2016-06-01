/*
 * vertices.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/environment/boustrophedon/vertices.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

Vertices::Vertices(PointPtr positions, PointPtr near_point1,
    PointPtr near_point2, bool is_of_obstacle) {
  this->position = positions;
  set_near_point(near_point1, near_point2);
  set_is_of_obstacles(is_of_obstacle);
  set_type_vertices();
}

void Vertices::set_near_point(PointPtr near1, PointPtr near2) {
  if (std::abs(near1->y - near2->y) < EPSILON) {
    if ((near1->x - near2->x) > EPSILON) {
      set_below_point(near1);
      set_upper_point(near2);
    } else {
      set_below_point(near2);
      set_upper_point(near1);
    }
  } else {
    if ((near1->y - near2->y) > EPSILON) {
      set_below_point(near2);
      set_upper_point(near1);
    } else {
      if (near1->y < near2->y) {
        set_below_point(near1);
        set_upper_point(near2);
      }
    }
  }
}

void Vertices::print_info_list_vertices(
    std::list<boost::shared_ptr<Vertices> > list_vertices) {
  std::list<boost::shared_ptr<Vertices> >::iterator u;
  int i;
  std::cout << "\033[1;34mList vertices \033[0m\033[1;31m\033[0m: " << "\n";
  for (u = list_vertices.begin(), i = 1; u != list_vertices.end(); ++u) {
    std::cout << "Vertices " << i++ << " : (" << (*u)->get_position()->x
        << " , " << (*u)->get_position()->y << " )\n";
    std::cout << "Upper " << " : (" << (*u)->get_upper_point()->x << " , "
        << (*u)->get_upper_point()->y << " )\n";
    std::cout << "Below " << " : (" << (*u)->get_below_point()->x << " , "
        << (*u)->get_below_point()->y << " )\n";
    std::cout << "Type Vertices :" << (*u)->type_vertice << "\n";
    if ((*u)->is_of_obstacles) {
      std::cout << "Vertices is obstacles!\n";
      if ((*u)->is_obstacles_upper) {
        std::cout << " Vertices is upper obstacles!\n";
      }
      if ((*u)->is_obstacles_below) {
        std::cout << " Vertices is below obstacles!\n";
      }
    }
    std::cout << "\n";
  }
}

void Vertices::set_is_of_obstacles(bool is_of_obstacles) {
  this->is_of_obstacles = is_of_obstacles;
}

void Vertices::set_type_vertices() {

  if ((std::abs(position->x - below_point->x) < EPSILON)
      || (std::abs(position->x - upper_point->x) < EPSILON)) {
    if ((position->x - below_point->x > EPSILON)
        || (position->x - upper_point->x > EPSILON)) {
      type_vertice = 5;
    } else {
      type_vertice = 6;
    }
  } else {
    if ((position->x - below_point->x) > EPSILON) {
      if ((position->x - upper_point->x) > EPSILON) {
        type_vertice = 1;
      } else {
        type_vertice = 2;
      }
    } else {
      if ((position->x - upper_point->x) > EPSILON) {
        type_vertice = 3;
      } else {
        type_vertice = 4;
      }
    }
  }
}

PointPtr Vertices::get_below_point() {
  return below_point;
}

void Vertices::set_below_point(PointPtr below_point) {
  this->below_point = below_point;
}

PointPtr Vertices::get_upper_point() {
  return upper_point;
}

void Vertices::set_upper_point(PointPtr upper_point) {
  this->upper_point = upper_point;
}

int Vertices::number_intersect(PolygonPtr obstacles, double size_y) {
  std::list<PointPtr> list_point;
  std::list<PointPtr>::iterator u;
  PointPtr temp_p1, temp_p;
  SegmentPtr temp1, temp2;
  int i;
  temp1 = SegmentPtr(
      new Segment(position->x, position->y, position->x, position->y + size_y));
  list_point = obstacles->get_points();
  u = --list_point.end();
  temp_p1 = PointPtr(new Point((*u)->x, (*u)->y));
  for (u = list_point.begin(), i = 0; u != list_point.end(); ++u) {
    temp2 = SegmentPtr(new Segment(temp_p1->x, temp_p1->y, (*u)->x, (*u)->y));
    temp_p = temp1 % temp2;
    if (temp_p) {
      if (temp_p != position) {
        i++;
      }
    }
    temp_p1 = PointPtr(new Point((*u)->x, (*u)->y));
  }
  return i;
}

void Vertices::set_is_obstacles_upper(PolygonPtr obstacles, double size_y) {
  int i;
  i = number_intersect(obstacles, size_y);
  if (i % 2 == 0) {
    this->is_obstacles_upper = false;
  } else {
    this->is_obstacles_upper = true;
  }
}
;

void Vertices::set_is_obstacles_below(PolygonPtr obstacles, double size_y) {
  int i;
  i = number_intersect(obstacles, -size_y);
  if (i % 2 == 0) {
    this->is_obstacles_below = false;
  } else {
    this->is_obstacles_below = true;
  }
}

SegmentPtr Vertices::get_segment_contain_nearest_intersect_point(
    std::list<SegmentPtr> list_segment, PointPtr point_cut, double size_cut) {
  std::list<SegmentPtr>::iterator u;
  PointPtr nearest_intersect_point, temp_point;
  SegmentPtr segment_cut, segment_find, segment_current;
  segment_cut = SegmentPtr(
      new Segment(point_cut->x, point_cut->y, point_cut->x,
          point_cut->y + size_cut));
  for (u = list_segment.begin(); u != list_segment.end(); ++u) {
    segment_current = SegmentPtr(new Segment((*u)->p1, (*u)->p2));
    temp_point = segment_cut % segment_current;
    if (!temp_point)
      continue;
    if (temp_point == point_cut)
      continue;
    if (!nearest_intersect_point) {
      nearest_intersect_point = temp_point;
      segment_find = segment_current;
      continue;
    }
    if ((std::abs(point_cut->y - nearest_intersect_point->y)
        - std::abs(point_cut->y - temp_point->y)) > EPSILON) {
      nearest_intersect_point = temp_point;
      segment_find = segment_current;
    }
  }
  return segment_find;
}

PointPtr Vertices::get_point_x_litte(SegmentPtr segment) {
  if ((segment->p1->x - segment->p2->x) > EPSILON) {
    return segment->p2;
  }
  return segment->p1;
}

PointPtr Vertices::get_point_x_large(SegmentPtr segment) {
  if ((segment->p1->x - segment->p2->x) > EPSILON) {
    return segment->p1;
  }
  return segment->p2;
}

void Vertices::update_list_vertices(
    std::list<boost::shared_ptr<Vertices> > list_vertices, PointPtr position,
    PointPtr value_befor, PointPtr value_update) {
  std::list<boost::shared_ptr<Vertices> >::iterator u;
  for (u = list_vertices.begin(); u != list_vertices.end(); ++u) {
    if ((*u)->get_position() == position) {
      if ((*u)->get_below_point() == value_befor) {
        (*u)->set_below_point(value_update);
      } else {
        (*u)->set_upper_point(value_update);
      }
    }
  }
}

void Vertices::update_list_segment(std::list<SegmentPtr> list_segment,
    SegmentPtr segment, PointPtr update_point) {
  std::list<SegmentPtr>::iterator u;
  for (u = list_segment.begin(); u != list_segment.end(); ++u) {
    if (((*u)->p1 == segment->p1) && ((*u)->p2 == segment->p2)) {
      (*u)->p1 = update_point;
      break;
    }
  }
}

bool Vertices::compare_vertices(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if ((vertices1->get_position()->x == vertices2->get_position()->x)
      || (vertices1->get_position()->y == vertices2->get_position()->y))
    return true;
  return false;
}

bool Vertices::compare_positions_x(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_position()->x < vertices2->get_position()->x)
    return true;
  return false;
}

bool Vertices::compare_positions_y(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_position()->y < vertices2->get_position()->y)
    return true;
  return false;
}

PointPtr Vertices::get_position() {
  return position;
}

void Vertices::set_positions(PointPtr positions) {
  this->position = positions;
}

}
}
}
