/*
 * mstc_online.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: manhnh
 */

#ifndef WANDRIAN_INCLUDE_PLANS_MSTC_MSTC_ONLINE_HPP_
#define WANDRIAN_INCLUDE_PLANS_MSTC_MSTC_ONLINE_HPP_

#include "../boustrophedon_online/a_star.hpp"
#include "../../common/vector.hpp"
//#include "../../environment/mstc/base_communicator.hpp"
#include "../../environment/mstc/mstc_communicator.hpp"
#include "../../environment/mstc/identifiable_cell.hpp"
#include "../base_plan.hpp"

using namespace wandrian::common;
using namespace wandrian::environment;
using namespace wandrian::environment::mstc;
using namespace wandrian::plans::boustrophedon_online;

namespace wandrian {
namespace plans {
namespace mstc {

class MstcOnline: public BasePlan {

public:
  MstcOnline();
  ~MstcOnline();
  virtual void initialize(PointPtr, double, MstcCommunicatorPtr);
  virtual void cover();

  void set_behavior_see_obstacle(boost::function<bool(VectorPtr, double)>);

protected:
  double tool_size; // = 'cell size' / 2
  MstcCommunicatorPtr communicator;

  bool go_to(PointPtr, bool);
  bool see_obstacle(VectorPtr, double);
  virtual State state_of(CellPtr);
  virtual void scan(CellPtr);

private:
  IdentifiableCellPtr starting_cell;
  boost::function<bool(VectorPtr, double)> behavior_see_obstacle;
  bool go_with(VectorPtr, double);

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
  int number_cell;
  int number_neighbor_cell;
  int check_insert;

  void insert_edge(CellPtr, CellPtr, int);
  void insert_cell_to_graph(CellPtr, CellPtr, int);
  int check_vertex(CellPtr);
  void bpmove(CellPtr);
  bool go_with_bpcell(PointPtr, VectorPtr, double);
  bool go_to_bpcell(PointPtr, bool);
  void find_bpcell(CellPtr);
  std::set<CellPtr, CellComp> old_cells_for_backtrack;
};

typedef boost::shared_ptr<MstcOnline> MstcOnlinePtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_PLANS_MSTC_MSTC_ONLINE_HPP_ */
