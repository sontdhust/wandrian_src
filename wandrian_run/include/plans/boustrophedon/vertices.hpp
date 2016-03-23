
#ifndef WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_VERTICES_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_VERTICES_HPP_

#include "obstacle.hpp"

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace boustrophedon{

class Vertices {

public:
	Vertices(PointPtr, ObstaclePtr);

	PointPtr get_positions();
	void set_positions(PointPtr);

	ObstaclePtr get_polygon();
	void set_polygon(ObstaclePtr);

	bool left_compared_center();
	bool upon_compared_center();

	static bool compare_vertices(boost::shared_ptr<Vertices> , boost::shared_ptr<Vertices> );
	static bool compare_positionsx(boost::shared_ptr<Vertices> , boost::shared_ptr<Vertices> );
	static bool compare_positionsy(boost::shared_ptr<Vertices> , boost::shared_ptr<Vertices> );
private:
  PointPtr positions;
  ObstaclePtr polygon;
};

typedef boost::shared_ptr<Vertices const> VerticesConstPtr;
typedef boost::shared_ptr<Vertices> VerticesPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_BOUSTROPHEDON_VERTICES_HPP_ */




