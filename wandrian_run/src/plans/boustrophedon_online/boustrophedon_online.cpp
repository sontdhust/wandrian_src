/*
 * boustrophedon_online.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: anhnt
 */

#include "../../../include/plans/boustrophedon_online/boustrophedon_online.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon_online {

BoustrophedonOnline::BoustrophedonOnline() :
    robot_size(0), check_rotate(0), straight(false), number_cell(0), number_neighbor_cell(
        0), check_insert(0), start(vertex(0)), goal(vertex(0)) {
}

BoustrophedonOnline::~BoustrophedonOnline() {
}

void BoustrophedonOnline::initialize(PointPtr starting_point,
    double robot_size) {
  this->robot_size = robot_size;
  check_rotate = -1;
  straight = true;
  number_cell = 0;
  number_neighbor_cell = 0;
  check_insert = 0;
  // Initialize starting_cell
  starting_cell = CellPtr(
      new Cell(PointPtr(new Point(starting_point->x, starting_point->y)),
          robot_size));
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - robot_size)),
              robot_size)));
  path.insert(path.end(), starting_point);
  old_cells.insert(old_cells.end(), starting_cell);
}

void BoustrophedonOnline::cover() {
  old_cells.insert(starting_cell);
  boustrophedon_online(starting_cell);
}

void BoustrophedonOnline::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool BoustrophedonOnline::go_to(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  path.insert(path.end(), position);

  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}
bool BoustrophedonOnline::go_to_bpcell(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}

bool BoustrophedonOnline::see_obstacle(VectorPtr orientation, double distance) {
  if (behavior_see_obstacle)
    return behavior_see_obstacle(orientation, distance);
  return false;
}

bool BoustrophedonOnline::go_with(VectorPtr orientation, double distance) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = PointPtr(
      new Point(last_position + orientation * distance));
  return go_to(new_position, STRICTLY);
}
bool BoustrophedonOnline::go_with_bpcell(PointPtr last_position,
    VectorPtr orientation, double distance) {
  PointPtr new_position = PointPtr(
      new Point(last_position + orientation * distance));
  return go_to_bpcell(new_position, STRICTLY);
}
void BoustrophedonOnline::go_straight(CellPtr neighbor, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor->set_parent(current);
  // current->neighbors.insert(current->neighbors.end(), neighbor);
  old_cells.insert(neighbor);
  go_with(orientation, robot_size);
  bplist.erase(neighbor);
  boustrophedon_online(neighbor);
}
void BoustrophedonOnline::turn_left(CellPtr neighbor_left, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor_left->set_parent(current);
  // current->neighbors.insert(current->neighbors.end(), neighbor_left);
  old_cells.insert(neighbor_left);
  go_with(orientation->rotate_counterclockwise_left(), robot_size);
  check_rotate = -1;
  bplist.erase(neighbor_left);
  boustrophedon_online(neighbor_left);
}
void BoustrophedonOnline::turn_right(CellPtr neighbor_right, CellPtr current,
    VectorPtr orientation) {
  std::cout << "\n";
  neighbor_right->set_parent(current);
  // current->neighbors.insert(current->neighbors.end(), neighbor_right);
  old_cells.insert(neighbor_right);
  go_with(orientation->rotate_counterclockwise_right(), robot_size);
  check_rotate = 1;
  bplist.erase(neighbor_right);
  boustrophedon_online(neighbor_right);
}
void BoustrophedonOnline::find_bpcell(CellPtr current) {

  cout << current->get_center()->x << ", " << current->get_center()->y << endl;
  start = check_vertex(current);
  int time = 100;
  for (std::set<CellPtr>::iterator i = bplist.begin(); i != bplist.end(); i++) {
    CellPtr tmp = CellPtr(*i);
    CellPtr neighbor_W = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x + robot_size,
                    tmp->get_center()->y)), robot_size));
    if (state_of(neighbor_W) == OLD)
      goal = check_vertex(neighbor_W);
    CellPtr neighbor_E = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x - robot_size,
                    tmp->get_center()->y)), robot_size));
    if (state_of(neighbor_E) == OLD)
      goal = check_vertex(neighbor_E);

    vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
    vector<cost> d(num_vertices(g));
    try {
      // call astar named parameter interface
      astar_search(g, start,
          distance_heuristic<mygraph_t, cost, location*>(locations, goal),
          predecessor_map(&p[0]).distance_map(&d[0]).visitor(
              astar_goal_visitor<vertex>(goal)));

    } catch (found_goal &fg) { // found a path to the goal
      list<vertex> shortest_path;
      for (vertex v = goal;; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v)
          break;
      }
      if (d[goal] < time) {
        backtrack_path = shortest_path;
        starting_cell = tmp;
        time = d[goal];
      }

    }

  }

  cout << endl << "Total travel time: " << time << endl;
  std::cout << "\033[1;32mNew Starting Cell:\033[0m" << " "
      << starting_cell->get_center()->x << ", "
      << starting_cell->get_center()->y << endl;
  list<vertex>::iterator spi = backtrack_path.begin();

  for (++spi; spi != backtrack_path.end(); ++spi)
    cout << "->" << locations[*spi].x << ", " << locations[*spi].y << endl;

}
void BoustrophedonOnline::bpmove(CellPtr current) {
  list<vertex>::iterator spi = backtrack_path.begin();

  for (++spi; spi != backtrack_path.end(); ++spi) {
    CellPtr next_cell = CellPtr(
        new Cell(PointPtr(new Point(locations[*spi].x, locations[*spi].y)),
            robot_size));
    VectorPtr orientation = VectorPtr(
        new Vector(
            (next_cell->get_center() - current->get_center()) / robot_size));
    go_with_bpcell(current->get_center(), orientation, robot_size);
    current = next_cell;
  }
  VectorPtr orientation = VectorPtr(
      new Vector(
          (starting_cell->get_center() - current->get_center()) / robot_size));
  go_with_bpcell(current->get_center(), orientation, robot_size);
  path.insert(path.end(), starting_cell->get_center());
  old_cells.insert(old_cells.end(), starting_cell);
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - robot_size)),
              robot_size)));
  bplist.erase(starting_cell);
  boustrophedon_online(starting_cell);
}

int BoustrophedonOnline::check_vertex(CellPtr current) {
  for (int i = 0; i <= number_neighbor_cell; i++) {
    if (locations[i].y == current->get_center()->y
        && locations[i].x == current->get_center()->x)
      return i;
  }
  return -1;
}
void BoustrophedonOnline::insert_edge(CellPtr current, CellPtr neighbor,
    int insert) {
  edge_descriptor e;
  bool inserted;
  if (insert == 2) {
    boost::tie(e, inserted) = add_edge(
        edge(number_cell, check_vertex(neighbor)).first,
        edge(number_cell, check_vertex(neighbor)).second, g);
    cout << "Check insert: "
        << boost::edge(number_cell, check_vertex(neighbor), g).first << endl;
  } else {
    boost::tie(e, inserted) = add_edge(
        edge(number_cell, number_neighbor_cell).first,
        edge(number_cell, number_neighbor_cell).second, g);
    cout << "Check insert: "
        << boost::edge(number_cell, number_neighbor_cell, g).first << endl;
  }
  weightmap[e] = 1;
  check_insert = check_insert + 1;
  if (insert == 1) {
    locations[number_neighbor_cell].x = neighbor->get_center()->x;
    locations[number_neighbor_cell].y = neighbor->get_center()->y;
  } else if (insert == 2) {
    locations[number_cell].x = current->get_center()->x;
    locations[number_cell].y = current->get_center()->y;
  } else if (insert == 3) {
    locations[number_cell].x = current->get_center()->x;
    locations[number_cell].y = current->get_center()->y;
    locations[number_neighbor_cell].x = neighbor->get_center()->x;
    locations[number_neighbor_cell].y = neighbor->get_center()->y;
  }
}

void BoustrophedonOnline::insert_cell_to_graph(CellPtr current,
    CellPtr neighbor, int vertex_current) {

  int vertex_neighbor = check_vertex(neighbor);
  if (state_of(neighbor) == OLD) {
    if (vertex_current == -1) {
      if (vertex_neighbor == -1) {
        number_neighbor_cell = number_neighbor_cell + 1;
        insert_edge(current, neighbor, 3);
      } else if (vertex_neighbor != -1) {
        insert_edge(current, neighbor, 2);
      }
    } else if (vertex_current != -1) {
      if (vertex_neighbor == -1) {
        number_neighbor_cell = number_neighbor_cell + 1;
        insert_edge(current, neighbor, 1);
      }
    }
  }
}
// void OnlineBoustrophedon::move_bpcell(CellPtr current){
//   CellPtr neighbor_N = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(
//                     starting_cell->get_center()->x, starting_cell->get_center()->y +robot_size)),
//             robot_size));
//   CellPtr neighbor_W = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(
//                     starting_cell->get_center()->x + robot_size, starting_cell->get_center()->y)),
//             robot_size));
//   CellPtr neighbor_E = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(
//                     starting_cell->get_center()->x -robot_size, starting_cell->get_center()->y)),
//             robot_size));
//   CellPtr neighbor_S = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(
//                     starting_cell->get_center()->x, starting_cell->get_center()->y - robot_size)),
//             robot_size));
//   if(check(neighbor_N)==OLD){
//     std::cout << "\033[1;32mNew neighbor N:\033[0m" << " " << neighbor_N->get_center()->x << ", " << neighbor_N->get_center()->y;     
//     bpmove(current, neighbor_N);
//   }else if(check(neighbor_W)==OLD){
//     std::cout << "\033[1;32mNew neighbor W:\033[0m" << " " << neighbor_W->get_center()->x << ", " << neighbor_W->get_center()->y;
//     bpmove(current, neighbor_W);
//   }else if(check(neighbor_E)==OLD){
//     std::cout << "\033[1;32mNew neighbor E:\033[0m" << " " << neighbor_E->get_center()->x << ", " << neighbor_E->get_center()->y;
//     bpmove(current, neighbor_E);
//   }else if(check(neighbor_S)==OLD){
//     std::cout << "\033[1;32mNew neighbor S:\033[0m" << " " << neighbor_S->get_center()->x << ", " << neighbor_S->get_center()->y;
//     bpmove(current, neighbor_S);
//   }
// }

void BoustrophedonOnline::boustrophedon_online(CellPtr current) {
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  std::cout << "Backtrack list: " << bplist.size() << "\n";
  VectorPtr orientation = VectorPtr(
      new Vector(
          (current->get_parent()->get_center() - current->get_center())
              / robot_size));

  // quay vector 180 do
  orientation = orientation->rotate_counterclockwise_180();
  if (current->get_center() == starting_cell->get_center()
      && see_obstacle(orientation, robot_size / 2)) {
    orientation = orientation->rotate_counterclockwise_180();
  }
  // Scan for the first new neighbor of current cell in counterclockwise order
  CellPtr neighbor = CellPtr(
      new Cell(
          PointPtr(new Point(current->get_center() + orientation * robot_size)),
          robot_size));
  // if(check(neighbor)!=OLD && see_obstacle(orientation,2)==false){
  //   bplist.insert(neighbor);
  // }
  CellPtr neighbor_left = CellPtr(
      new Cell(
          PointPtr(
              new Point(
                  current->get_center()
                      + orientation->rotate_counterclockwise_left()
                          * robot_size)), robot_size));
  if (state_of(neighbor_left) != OLD
      && see_obstacle(orientation->rotate_counterclockwise_left(), robot_size/2)
          == false) {
    bplist.insert(neighbor_left);
  }

  CellPtr neighbor_right = CellPtr(
      new Cell(
          PointPtr(
              new Point(
                  current->get_center()
                      + orientation->rotate_counterclockwise_right()
                          * robot_size)), robot_size));
  if (state_of(neighbor_right) != OLD
      && see_obstacle(orientation->rotate_counterclockwise_right(), robot_size/2)
          == false) {
    bplist.insert(neighbor_right);
  }
  CellPtr neighbor_bottom =
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(
                      *(current->get_center()
                          + orientation->rotate_counterclockwise_180()
                              * robot_size))), robot_size));
  if (state_of(neighbor_bottom) != OLD
      && see_obstacle(orientation->rotate_counterclockwise_180(), robot_size/2) == false) {
    bplist.insert(neighbor_bottom);
  }

  //mahantan
  // current->neighbors.insert(current->neighbors.end(), neighbor);
  // current->neighbors.insert(current->neighbors.end(), neighbor_left);
  // current->neighbors.insert(current->neighbors.end(), neighbor_right);
  // current->neighbors.insert(current->neighbors.end(), neighbor_bottom);
  // std::cout << "  \033[1;33mneighbor\033[0m: " << neighbor->get_center()->x
  //     << "," << neighbor->get_center()->y;
  //A* search
  int vertex_current = check_vertex(current);
  if (vertex_current != -1) {
    if (check_insert == 0) {
      number_neighbor_cell = number_cell;
    } else {
      // number_neighbor_cell = number_neighbor_cell;
    }
    number_cell = vertex_current;

  } else {
    if (check_insert != 0) {
      number_cell = number_neighbor_cell + 1;
      number_neighbor_cell = number_neighbor_cell + 1;
    }

  }

  check_insert = 0;
  insert_cell_to_graph(current, neighbor_bottom, vertex_current);
  insert_cell_to_graph(current, neighbor_left, vertex_current);
  insert_cell_to_graph(current, neighbor_right, vertex_current);
  insert_cell_to_graph(current, neighbor, vertex_current);

  if (straight == false) {
    if (check_rotate == 1) {
      if (see_obstacle(orientation->rotate_counterclockwise_right(), robot_size/2) == false
          && state_of(neighbor_right) != OLD) {
        straight = true;
        turn_right(neighbor_right, current, orientation);
      } else if (see_obstacle(orientation->rotate_counterclockwise_left(), robot_size/2)
          == false && state_of(neighbor_left) != OLD) {
        straight = true;
        turn_left(neighbor_left, current, orientation);
      } else if (see_obstacle(orientation, robot_size/2) == false
          && state_of(neighbor) != OLD) {
        go_straight(neighbor, current, orientation);
      }
    } else {
      if (see_obstacle(orientation->rotate_counterclockwise_left(), robot_size/2) == false
          && state_of(neighbor_left) != OLD) {
        straight = true;
        turn_left(neighbor_left, current, orientation);
      } else if (see_obstacle(orientation->rotate_counterclockwise_right(), robot_size/2)
          == false && state_of(neighbor_right) != OLD) {
        straight = true;
        turn_right(neighbor_right, current, orientation);
      } else if (see_obstacle(orientation, robot_size/2) == false
          && state_of(neighbor) != OLD) {
        go_straight(neighbor, current, orientation);
      }
    }
  } else {
    if (see_obstacle(orientation, robot_size/2) || state_of(neighbor) == OLD) { // TODO: Check obstacle here
      std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
      // Go to next sub-cell

      if (check_rotate == 1) {
        if (see_obstacle(orientation->rotate_counterclockwise_left(), robot_size/2)
            == false && state_of(neighbor_left) != OLD) {
          straight = false;
          turn_left(neighbor_left, current, orientation);
        } else if (see_obstacle(orientation->rotate_counterclockwise_right(), robot_size/2)
            == false && state_of(neighbor_right) != OLD) {
          straight = false;
          turn_right(neighbor_right, current, orientation);
        }

      } else if (check_rotate == -1) {
        if (see_obstacle(orientation->rotate_counterclockwise_right(), robot_size/2)
            == false && state_of(neighbor_right) != OLD) {
          straight = false;
          turn_right(neighbor_right, current, orientation);
        } else if (see_obstacle(orientation->rotate_counterclockwise_left(), robot_size/2)
            == false && state_of(neighbor_left) != OLD) {
          straight = false;
          turn_left(neighbor_left, current, orientation);
        }
      }
    } else if (see_obstacle(orientation, robot_size/2) == false
        && state_of(neighbor) != OLD) { // New free neighbor
      straight = true;
      go_straight(neighbor, current, orientation);
    }
  }
  old_cells.insert(old_cells.end(), current);
  std::cout << "\n\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  std::cout << "Backtrack list: " << bplist.size() << "\n";

  if (bplist.size() > 0) {
    find_bpcell(current);
    bpmove(current);
    // exit(0);
  }

}

State BoustrophedonOnline::state_of(CellPtr cell_to_check) {
  return (old_cells.find(cell_to_check) != old_cells.end()) ? OLD : NEW;
}

}
}
}
