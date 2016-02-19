/*
 * online_boustrophedon.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: anhnt
 */

#ifndef WANDRIAN_RUN_INCLUDE_PLANS_ONLINE_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_HPP_
#define WANDRIAN_RUN_INCLUDE_PLANS_ONLINE_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_HPP_

#include "../../common/environment.hpp"
#include "../base_plan.hpp"
#include "cell.hpp"
#include "a_star.hpp"
#define MAX 100

using namespace wandrian::common;

namespace wandrian {
namespace plans {
namespace online_boustrophedon {


class OnlineBoustrophedon: public BasePlan {

public:
  OnlineBoustrophedon();
  ~OnlineBoustrophedon();
  void initialize(PointPtr, double);
  void cover();

  /* __attribute__((will_be_removed)) */
  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  bool go_to(PointPtr, bool);
  bool go_to_bpcell(PointPtr, bool);

private:

  CellPtr starting_cell;
  double robot_size; // = 'cell size' 
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
  std::set<CellPtr, CellComp> old_cells;
  std::set<CellPtr, CellComp> bplist;
  
  bool see_obstacle(VectorPtr, double);
  bool go_with(VectorPtr, double);
  bool go_with_bpcell(PointPtr, VectorPtr, double);
  void turn_left(CellPtr , CellPtr, VectorPtr);
  void turn_right(CellPtr , CellPtr, VectorPtr);
  void go_straight(CellPtr , CellPtr, VectorPtr);
  void bpmove(CellPtr);
  void find_bpcell(CellPtr);
  void online_boustrophedon(CellPtr);
  bool check(CellPtr);
  int check_rotate;
  bool straight;
  int number_cell;
  int number_neighbor_cell;
  int check_insert;
  // A* search; 
  typedef adjacency_list<listS, vecS, undirectedS, no_property,
    property<edge_weight_t, cost> > mygraph_t;
  typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vertex;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef mygraph_t::vertex_iterator vertex_iterator;
  typedef std::pair<int, int> edge;

  location locations[1000];
  mygraph_t g;
  WeightMap  weightmap;
  vertex start;
  vertex goal;

  list<vertex> backtrack_path;
  int check_vertex(CellPtr);
  void insert_edge(CellPtr, CellPtr, int);
  void insert_cell_to_graph(CellPtr, CellPtr, int);
};

typedef boost::shared_ptr<OnlineBoustrophedon> OnlineBoustrophedonPtr;

}
}
}

#endif /* WANDRIAN_RUN_INCLUDE_PLANS_ONLINE_BOUSTROPHEDON_ONLINE_BOUSTROPHEDON_HPP_ */