/*
 * boustrophedon_online.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: anhnt
 */

#ifndef WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_ONLINE_HPP_
#define WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_ONLINE_HPP_

#include "../../common/vector.hpp"
#include "../../environment/cell.hpp"
#include "../base_plan.hpp"
#include "a_star.hpp"

#define MAX 100

using namespace wandrian::common;
using namespace wandrian::environment;

namespace wandrian {
namespace plans {
namespace boustrophedon_online {

class BoustrophedonOnline: public BasePlan {

public:
  BoustrophedonOnline();
  ~BoustrophedonOnline();
  void initialize(PointPtr, double);
  void cover();

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  bool go_to(PointPtr, bool = STRICTLY);
  bool go_to_bpcell(PointPtr, bool);

private:
  CellPtr starting_cell;
  double tool_size; // = 'cell size'
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
  std::set<CellPtr, CellComp> old_cells;
  std::set<CellPtr, CellComp> bplist;
  int check_rotate;
  bool straight;
  int number_cell;
  int number_neighbor_cell;
  int check_insert;

  // A* search;
  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
      boost::no_property, boost::property<boost::edge_weight_t, cost> > mygraph_t;
  typedef boost::property_map<mygraph_t, boost::edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vertex;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef mygraph_t::vertex_iterator vertex_iterator;
  typedef std::pair<int, int> edge;
  location locations[1000];
  mygraph_t g;
  WeightMap weightmap;
  vertex start;
  vertex goal;
  std::list<vertex> backtrack_path;

  bool see_obstacle(VectorPtr, double);
  bool go_with(VectorPtr, double);
  bool go_with_bpcell(PointPtr, VectorPtr, double);
  void go_straight(CellPtr, CellPtr, VectorPtr);
  void turn_left(CellPtr, CellPtr, VectorPtr);
  void turn_right(CellPtr, CellPtr, VectorPtr);
  void bpmove(CellPtr);
  void scan(CellPtr);
  void find_bpcell(CellPtr);
  bool find_into_bplist(CellPtr);
  void refine_bplist();
  int check_vertex(CellPtr);
  double check_distance(CellPtr, CellPtr);
  void insert_edge(CellPtr, CellPtr, int);
  void insert_cell_to_graph(CellPtr, CellPtr, int);
  State state_of(CellPtr);
};

typedef boost::shared_ptr<BoustrophedonOnline> BoustrophedonOnlinePtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_ONLINE_HPP_ */
