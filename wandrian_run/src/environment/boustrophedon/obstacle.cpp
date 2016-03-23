#include "../../../include/environment/boustrophedon/obstacle.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

Obstacle::Obstacle(PointPtr center, double sizex, double sizey) :
    center(center), sizex(sizex), sizey(sizey) {
  points.insert(points.end(),
      PointPtr(new Point(center->x - sizex / 2, center->y + sizey / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + sizex / 2, center->y + sizey / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x + sizex / 2, center->y - sizey / 2)));
  points.insert(points.end(),
      PointPtr(new Point(center->x - sizex / 2, center->y - sizey / 2)));
  build();
}

PointPtr Obstacle::get_center() {
  return center;
}

double Obstacle::get_sizex() {
  return sizex;
}
double Obstacle::get_sizey() {
  return sizey;
}

}
}
}

