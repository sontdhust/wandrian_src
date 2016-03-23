#include "../../../include/environment/boustrophedon/vertices.hpp"

namespace wandrian {
namespace environment {
namespace boustrophedon {

Vertices::Vertices(PointPtr positions, ObstaclePtr polygon) {
  this->positions = positions;
  this->polygon = polygon;
}

PointPtr Vertices::get_positions() {
  return positions;
}

void Vertices::set_positions(PointPtr positions) {
  this->positions = positions;
}

void Vertices::set_polygon(ObstaclePtr polygon) {
  this->polygon = polygon;
}

ObstaclePtr Vertices::get_polygon() {
  return polygon;
}

bool Vertices::left_compared_center() {
  if (this->positions->x < this->polygon->get_center()->x)
    return true;
  return false;
}
bool Vertices::upon_compared_center() {
  if (this->positions->y > this->polygon->get_center()->y)
    return true;
  return false;
}

bool Vertices::compare_vertices(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if ((vertices1->get_positions()->x == vertices2->get_positions()->x)
      || (vertices1->get_positions()->y == vertices2->get_positions()->y))
    return true;
  return false;
}

bool Vertices::compare_positionsx(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_positions()->x < vertices2->get_positions()->x)
    return true;
  return false;
}

bool Vertices::compare_positionsy(boost::shared_ptr<Vertices> vertices1,
    boost::shared_ptr<Vertices> vertices2) {
  if (vertices1->get_positions()->y < vertices2->get_positions()->y)
    return true;
  return false;
}

}
}
}

