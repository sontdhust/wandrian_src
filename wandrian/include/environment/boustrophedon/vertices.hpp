/*
 * vertices.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_

#include "../../common/rectangle.hpp"
#include "../../common/segment.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace environment {
namespace boustrophedon {

class Vertices {

public:
  Vertices(PointPtr ,PointPtr ,PointPtr , bool);
  bool left_compared_center();
  bool upon_compared_center();
  void set_is_of_obstacles(bool);
  static bool compare_vertices(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);
  static bool compare_positions_x(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);
  static bool compare_positions_y(boost::shared_ptr<Vertices>,
      boost::shared_ptr<Vertices>);

  void set_positions(PointPtr);
  void set_upper_point(PointPtr );
  void set_below_point(PointPtr );

  PointPtr get_position();
  PointPtr get_below_point();
  PointPtr get_upper_point();

  int type_vertice;
  bool is_of_obstacles;
  bool is_obstacles_upper;
  bool is_obstacles_below;
  void set_is_obstacles_upper(PolygonPtr , double);
  void set_is_obstacles_below(PolygonPtr , double);
  static void update_list_vertices(std::list< boost::shared_ptr<Vertices> >,
		  	  	  	  	   PointPtr, PointPtr, PointPtr);
  static void update_list_segment(std::list<SegmentPtr>,SegmentPtr, PointPtr);
  static void print_info_list_vertices(std::list< boost::shared_ptr<Vertices> >);
  static SegmentPtr get_segment_contain_nearest_intersect_point
  (std::list<SegmentPtr>,PointPtr,double);
  static PointPtr get_point_x_litte(SegmentPtr);
  static PointPtr get_point_x_large(SegmentPtr);

private:
  void set_near_point(PointPtr, PointPtr);
  void set_type_vertices();
  int number_intersect(PolygonPtr , double);
  PointPtr position;
  PointPtr upper_point;
  PointPtr below_point;
};

typedef boost::shared_ptr<Vertices const> VerticesConstPtr;
typedef boost::shared_ptr<Vertices> VerticesPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_BOUSTROPHEDON_VERTICES_HPP_ */
