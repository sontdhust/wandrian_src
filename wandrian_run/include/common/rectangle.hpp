/*
 * rectangle.hpp
 *
 *  Created on: Mar 18, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_RUN_INCLUDE_COMMON_RECTANGLE_HPP_
#define WANDRIAN_RUN_INCLUDE_COMMON_RECTANGLE_HPP_

#include "polygon.hpp"

namespace wandrian {
namespace common {

class Rectangle: public Polygon {

public:
  Rectangle(PointPtr, double, double);
  ~Rectangle();

  PointPtr get_center() const;
  double get_width() const;
  double get_height() const;

protected:
  PointPtr center;

private:
  double width;
  double height;
};

typedef boost::shared_ptr<Rectangle> RectanglePtr;
typedef boost::shared_ptr<Rectangle const> RectangleConstPtr;

}
}

#endif /* WANDRIAN_RUN_INCLUDE_COMMON_RECTANGLE_HPP_ */
