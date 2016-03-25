/*
 * a_star.hpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#ifndef WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_ONLINE_A_STAR_HPP_
#define WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_ONLINE_A_STAR_HPP_

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <math.h>

namespace wandrian {
namespace plans {
namespace boustrophedon_online {

struct location {
  double x, y; // lat, long
};

typedef float cost;

// euclidean distance heuristic
template<class Graph, class CostType, class LocMap>
class distance_heuristic: public boost::astar_heuristic<Graph, CostType> {

public:
  typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(LocMap l, Vertex goal) :
      m_location(l), m_goal(goal) {
  }
  CostType operator()(Vertex u) {
    CostType dx = m_location[m_goal].x - m_location[u].x;
    CostType dy = m_location[m_goal].y - m_location[u].y;
    return std::sqrt(dx * dx + dy * dy);
  }

private:
  LocMap m_location;
  Vertex m_goal;
};

struct found_goal {
};
// exception for termination

// visitor that terminates when we find the goal
template<class Vertex>
class astar_goal_visitor: public boost::default_astar_visitor {

public:
  astar_goal_visitor(Vertex goal) :
      m_goal(goal) {
  }
  template<class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if (u == m_goal)
      throw found_goal();
  }

private:
  Vertex m_goal;
};

}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_ONLINE_A_STAR_HPP_ */
