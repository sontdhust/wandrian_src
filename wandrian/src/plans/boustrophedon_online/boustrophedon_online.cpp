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
    tool_size(0), check_rotate(0), straight(false), number_cell(0), number_neighbor_cell(
        0), check_insert(0), start(vertex(0)), goal(vertex(0)) {
}

BoustrophedonOnline::~BoustrophedonOnline() {
}

void BoustrophedonOnline::initialize(PointPtr starting_point,
    double tool_size) {
  this->tool_size = tool_size;
  check_rotate = -1;
  straight = true;
  number_cell = 0;
  number_neighbor_cell = 0;
  check_insert = 0;
  // Initialize starting_cell
  starting_cell = CellPtr(
      new Cell(PointPtr(new Point(starting_point->x, starting_point->y)),
          tool_size));
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - tool_size)),
              tool_size)));
  path.insert(path.end(), starting_point);
  old_cells.insert(old_cells.end(), starting_cell);
}

void BoustrophedonOnline::cover() {
  old_cells.insert(starting_cell);
  scan(starting_cell);
}

void BoustrophedonOnline::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool BoustrophedonOnline::go_to(PointPtr position, bool flexibility) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  return BasePlan::go_to(position, flexibility);
}

bool BoustrophedonOnline::see_obstacle(VectorPtr direction, double distance) {
  if (behavior_see_obstacle)
    return behavior_see_obstacle(direction, distance);
  return false;
}

bool BoustrophedonOnline::go_with(VectorPtr direction, double distance) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = PointPtr(
      new Point(last_position + direction * distance));
  return go_to(new_position);
}

bool BoustrophedonOnline::go_with_bpcell(PointPtr last_position,
    VectorPtr direction, double distance) {
  PointPtr new_position = PointPtr(
      new Point(last_position + direction * distance));
  return go_to(new_position);
}

void BoustrophedonOnline::go_straight(CellPtr neighbor, CellPtr current,
    VectorPtr direction) {
  std::cout << "\n";
  neighbor->set_parent(current);
  old_cells.insert(neighbor);
  go_with(direction, tool_size);
  bplist.erase(neighbor);
  scan(neighbor);
}

void BoustrophedonOnline::turn_left(CellPtr neighbor_left, CellPtr current,
    VectorPtr direction) {
  std::cout << "\n";
  neighbor_left->set_parent(current);
  old_cells.insert(neighbor_left);
  go_with(+direction, tool_size);
  check_rotate = -1;
  bplist.erase(neighbor_left);
  scan(neighbor_left);
}

void BoustrophedonOnline::turn_right(CellPtr neighbor_right, CellPtr current,
    VectorPtr direction) {
  std::cout << "\n";
  neighbor_right->set_parent(current);
  old_cells.insert(neighbor_right);
  go_with(-direction, tool_size);
  check_rotate = 1;
  bplist.erase(neighbor_right);
  scan(neighbor_right);
}

void BoustrophedonOnline::bpmove(CellPtr current) {
  std::list<vertex>::iterator spi = backtrack_path.begin();
  for (++spi; spi != backtrack_path.end(); ++spi) {
    CellPtr next_cell = CellPtr(
        new Cell(PointPtr(new Point(locations[*spi].x, locations[*spi].y)),
            tool_size));
    VectorPtr direction = VectorPtr(
        new Vector(
            (next_cell->get_center() - current->get_center()) / tool_size));
    go_with_bpcell(current->get_center(), direction, tool_size);
    current = next_cell;
  }
  VectorPtr direction = VectorPtr(
      new Vector(
          (starting_cell->get_center() - current->get_center()) / tool_size));
  go_with_bpcell(current->get_center(), direction, tool_size);
  path.insert(path.end(), starting_cell->get_center());
  old_cells.insert(old_cells.end(), starting_cell);
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - tool_size)),
              tool_size)));
  bplist.erase(starting_cell);
  check_rotate = -1;
  straight = 1;
  scan(starting_cell);
}

void BoustrophedonOnline::scan(CellPtr current) {

  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  std::cout << "Backtrack list: " << bplist.size() << "\n";
  std::cout << "Straight: " << straight << "\n";
  VectorPtr direction = VectorPtr(
      new Vector(
          (current->get_parent()->get_center() - current->get_center())
              / tool_size));

  // Quay vector 180 do
  direction = +(+direction);

  CellPtr neighbor = CellPtr(
      new Cell(
          PointPtr(new Point(current->get_center() + direction * tool_size)),
          tool_size));
  if (current->get_center() == starting_cell->get_center()
      && (see_obstacle(direction, tool_size / 2) || state_of(neighbor) == OLD)) {
    direction = +(+direction);
  }

  // Scan for the first new neighbor of current cell in counterclockwise order
  neighbor = CellPtr(
      new Cell(
          PointPtr(new Point(current->get_center() + direction * tool_size)),
          tool_size));
  if (state_of(neighbor) != OLD
      && see_obstacle(direction, tool_size / 2) == false) {
    bplist.insert(neighbor);
  }

  CellPtr neighbor_left = CellPtr(
      new Cell(
          PointPtr(new Point(current->get_center() + +direction * tool_size)),
          tool_size));
  if (state_of(neighbor_left) != OLD
      && see_obstacle(+direction, tool_size / 2) == false) {
    bplist.insert(neighbor_left);
  }

  CellPtr neighbor_right = CellPtr(
      new Cell(
          PointPtr(new Point(current->get_center() + -direction * tool_size)),
          tool_size));
  if (state_of(neighbor_right) != OLD
      && see_obstacle(-direction, tool_size / 2) == false) {
    bplist.insert(neighbor_right);
  }
  CellPtr neighbor_bottom = CellPtr(
      new Cell(
          PointPtr(
              new Point(*(current->get_center() + +(+direction) * tool_size))),
          tool_size));
  if (state_of(neighbor_bottom) != OLD
      && see_obstacle(+(+direction), tool_size / 2) == false) {
    bplist.insert(neighbor_bottom);
  }

  // A* search
  int vertex_current = check_vertex(current);
  if (vertex_current != -1) {
    if (check_insert == 0) {
      number_neighbor_cell = number_cell;
    } else {
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
      if (see_obstacle(-direction, tool_size / 2) == false
          && state_of(neighbor_right) != OLD) {
        straight = true;
        turn_right(neighbor_right, current, direction);
      } else if (see_obstacle(+direction, tool_size / 2) == false
          && state_of(neighbor_left) != OLD) {
        straight = true;
        turn_left(neighbor_left, current, direction);
      } else if (see_obstacle(direction, tool_size / 2) == false
          && state_of(neighbor) != OLD) {
        go_straight(neighbor, current, direction);
      }
    } else {
      if (see_obstacle(+direction, tool_size / 2) == false
          && state_of(neighbor_left) != OLD) {
        straight = true;
        turn_left(neighbor_left, current, direction);
      } else if (see_obstacle(-direction, tool_size / 2) == false
          && state_of(neighbor_right) != OLD) {
        straight = true;
        turn_right(neighbor_right, current, direction);
      } else if (see_obstacle(direction, tool_size / 2) == false
          && state_of(neighbor) != OLD) {
        go_straight(neighbor, current, direction);
      }
    }
  } else {
    if (see_obstacle(direction, tool_size / 2) || state_of(neighbor) == OLD) {
      std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
      // Go to next sub-cell
      if (check_rotate == 1) {
        if (see_obstacle(+direction, tool_size / 2) == false
            && state_of(neighbor_left) != OLD) {
          straight = false;
          turn_left(neighbor_left, current, direction);
        } else if (see_obstacle(-direction, tool_size / 2) == false
            && state_of(neighbor_right) != OLD) {
          straight = false;
          turn_right(neighbor_right, current, direction);
        }
      } else if (check_rotate == -1) {
        if (see_obstacle(-direction, tool_size / 2) == false
            && state_of(neighbor_right) != OLD) {
          straight = false;
          turn_right(neighbor_right, current, direction);
        } else if (see_obstacle(+direction, tool_size / 2) == false
            && state_of(neighbor_left) != OLD) {
          straight = false;
          turn_left(neighbor_left, current, direction);
        }
      }
    } else if (see_obstacle(direction, tool_size / 2) == false
        && state_of(neighbor) != OLD) { // New free neighbor
      straight = true;
      go_straight(neighbor, current, direction);
    }
  }
  old_cells.insert(old_cells.end(), current);
  std::cout << "\n\033[1;34mcurrent-\033[0m\033[1;31mEND\033[0m: "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  if (bplist.size() > 0) {
    refine_bplist();
    std::cout << "Backtrack list: " << bplist.size() << "\n";
    find_bpcell(current);
    bpmove(current);
  }

}

void BoustrophedonOnline::find_bpcell(CellPtr current) {
  std::cout << current->get_center()->x << ", " << current->get_center()->y
      << std::endl;
  start = check_vertex(current);
  int time = 100;
  for (std::set<CellPtr>::iterator i = bplist.begin(); i != bplist.end(); i++) {
    CellPtr tmp = CellPtr(*i);
    CellPtr neighbor_E = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x - tool_size,
                    tmp->get_center()->y)), tool_size));
    if (state_of(neighbor_E) == OLD) {
      goal = check_vertex(neighbor_E);
    }

    CellPtr neighbor_W = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x + tool_size,
                    tmp->get_center()->y)), tool_size));
    if (state_of(neighbor_W) == OLD
        && check_distance(current, neighbor_W)
            < check_distance(current, neighbor_E)) {
      goal = check_vertex(neighbor_W);
    }

    CellPtr neighbor_N = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x,
                    tmp->get_center()->y + tool_size)), tool_size));
    if (state_of(neighbor_N) == OLD) {
      if (state_of(neighbor_E) == OLD
          && check_distance(current, neighbor_N)
              < check_distance(current, neighbor_E)) {
        goal = check_vertex(neighbor_N);
      }
      if (state_of(neighbor_W) == OLD
          && check_distance(current, neighbor_N)
              < check_distance(current, neighbor_W)) {
        goal = check_vertex(neighbor_N);
      }
    }

    CellPtr neighbor_S = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x,
                    tmp->get_center()->y - tool_size)), tool_size));
    if (state_of(neighbor_S) == OLD) {
      goal = check_vertex(neighbor_S);
    }
    std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
    std::vector<cost> d(num_vertices(g));
    try {
      // Call astar named parameter interface
      boost::astar_search(g, start,
          distance_heuristic<mygraph_t, cost, location*>(locations, goal),
          boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(
              astar_goal_visitor<vertex>(goal)));
    } catch (found_goal &fg) { // Found a path to the goal
      std::list<vertex> shortest_path;
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

  std::cout << std::endl << "Total travel time: " << time << std::endl;
  std::cout << "\033[1;32mNew Starting Cell:\033[0m" << " "
      << starting_cell->get_center()->x << ", "
      << starting_cell->get_center()->y << std::endl;
  std::list<vertex>::iterator spi = backtrack_path.begin();
  for (++spi; spi != backtrack_path.end(); ++spi)
    std::cout << "->" << locations[*spi].x << ", " << locations[*spi].y
        << std::endl;
}

int BoustrophedonOnline::check_vertex(CellPtr current) {
  for (int i = 0; i <= number_neighbor_cell; i++) {
    if (std::abs(locations[i].x - current->get_center()->x) < EPSILON
        && std::abs(locations[i].y - current->get_center()->y) < EPSILON)
      return i;
  }
  return -1;
}
double BoustrophedonOnline::check_distance(CellPtr begin, CellPtr end) {
  return std::abs(end->get_center()->x - begin->get_center()->x)
      + std::abs(end->get_center()->y - begin->get_center()->y);
}
bool BoustrophedonOnline::find_into_bplist(CellPtr cell_to_check) {
  return (bplist.find(cell_to_check) != bplist.end()) ? true : false;
}
void BoustrophedonOnline::refine_bplist() {
  std::set<CellPtr, CellComp> tmp_list;
  for (std::set<CellPtr>::iterator i = bplist.begin(); i != bplist.end(); i++) {
    CellPtr tmp = CellPtr(*i);
    CellPtr neighbor_N = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x,
                    tmp->get_center()->y + tool_size)), tool_size));

    CellPtr neighbor_S = CellPtr(
        new Cell(
            PointPtr(
                new Point(tmp->get_center()->x,
                    tmp->get_center()->y - tool_size)), tool_size));

    if (find_into_bplist(neighbor_N) && find_into_bplist(neighbor_S))
      tmp_list.insert(tmp);
  }
  for (std::set<CellPtr>::iterator i = tmp_list.begin(); i != tmp_list.end();
      i++) {
    bplist.erase(CellPtr(*i));
  }
}
void BoustrophedonOnline::insert_edge(CellPtr current, CellPtr neighbor,
    int insert) {
  edge_descriptor e;
  bool inserted;
  if (insert == 2) {
    boost::tie(e, inserted) = add_edge(
        edge(number_cell, check_vertex(neighbor)).first,
        edge(number_cell, check_vertex(neighbor)).second, g);
    std::cout << "Check insert: "
        << boost::edge(number_cell, check_vertex(neighbor), g).first
        << std::endl;
  } else {
    boost::tie(e, inserted) = add_edge(
        edge(number_cell, number_neighbor_cell).first,
        edge(number_cell, number_neighbor_cell).second, g);
    std::cout << "Check insert: "
        << boost::edge(number_cell, number_neighbor_cell, g).first << std::endl;
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

State BoustrophedonOnline::state_of(CellPtr cell_to_check) {
  return (old_cells.find(cell_to_check) != old_cells.end()) ? OLD : NEW;
}

}
}
}
