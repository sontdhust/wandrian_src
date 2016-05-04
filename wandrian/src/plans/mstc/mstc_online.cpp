/*
 * mstc_online.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: manhnh
 */

#include <algorithm>
#include "../../../include/plans/mstc/mstc_online.hpp"

namespace wandrian {
namespace plans {
namespace mstc {

MstcOnline::MstcOnline() :
    tool_size(0), number_cell(0), number_neighbor_cell(0), check_insert(0), start(
        vertex(0)), goal(vertex(0)) {
}

MstcOnline::~MstcOnline() {
}

void MstcOnline::initialize(PointPtr starting_point, double tool_size,
    MstcCommunicatorPtr communicator) {
  number_cell = 0;
  number_neighbor_cell = 0;
  check_insert = 0;

  communicator->set_tool_size(tool_size);
  this->tool_size = tool_size;
  this->communicator = communicator;
  // Initialize starting_cell
  starting_cell = IdentifiableCellPtr(
      new IdentifiableCell(
          PointPtr(
              new Point(starting_point->x - tool_size / 2,
                  starting_point->y + tool_size / 2)), 2 * tool_size,
          communicator->get_robot_name()));
  starting_cell->set_parent(
      IdentifiableCellPtr(
          new IdentifiableCell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - 2 * tool_size)),
              2 * tool_size, communicator->get_robot_name())));
  path.insert(path.end(), starting_point);
  old_cells_for_backtrack.insert(old_cells_for_backtrack.end(), starting_cell);
}

void MstcOnline::cover() {
  communicator->get_old_cells_message_from_server();
  communicator->read_message_then_update_old_cells();
  communicator->insert_old_cell(starting_cell);
  std::string message = communicator->create_old_cells_message();
  communicator->write_old_cells_message(message);
  communicator->send_save_message_to_server(communicator->create_old_cells_message_to_send_to_server(message));
  //  communicator->set_current_cell(starting_cell);
  // FIXME
  std::string status = communicator->create_status_message(starting_cell);
  communicator->write_status_message(status);
  communicator->send_save_message_to_server(communicator->create_status_message_to_send_to_server(status));
  old_cells_for_backtrack.insert(starting_cell);
  scan(starting_cell);
}

void MstcOnline::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool MstcOnline::go_to(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y;
  path.insert(path.end(), position);
  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}

bool MstcOnline::see_obstacle(VectorPtr direction, double distance) {
  bool get_obstacle;
  if (behavior_see_obstacle)
    get_obstacle = behavior_see_obstacle(direction, distance);
  else
    get_obstacle = false;
  if (get_obstacle)
    std::cout << " \033[1;46m(OBSTACLE)\033[0m\n";
  return get_obstacle;
}

State MstcOnline::state_of(CellPtr cell) {
  State state =
      (communicator->find_old_cell(
          boost::static_pointer_cast<IdentifiableCell>(cell))) ? OLD : NEW;
  if (state == OLD)
    std::cout << " \033[1;45m(OLD)\033[0m\n";
  return state;
}

void MstcOnline::scan(CellPtr current) {
  std::string status;
  communicator->set_current_cell(current);
//  communicator->read_obstacle_message();
  // FIXME
  status = communicator->create_status_message(
      boost::static_pointer_cast<IdentifiableCell>(current));
  communicator->write_status_message(status);
  communicator->send_save_message_to_server(communicator->create_status_message_to_send_to_server(status));
  communicator->get_old_cells_message_from_server();
  communicator->read_message_then_update_old_cells();
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;32mBEGIN:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  VectorPtr direction = (current->get_parent()->get_center()
      - current->get_center()) / 2 / tool_size;
  VectorPtr initial_direction = direction++;
  // While current cell has a new obstacle-free neighboring cell

  // Start A star
  VectorPtr tmp_direction = direction;
  tmp_direction = +(+direction);
  CellPtr neighbor = CellPtr(
      new Cell(
          PointPtr(
              new Point(current->get_center() + tmp_direction * tool_size)),
          tool_size));

  CellPtr neighbor_left = CellPtr(
      new Cell(
          PointPtr(new Point(current->get_center() + +direction * tool_size)),
          tool_size));

  CellPtr neighbor_right = CellPtr(
      new Cell(
          PointPtr(new Point(current->get_center() + -direction * tool_size)),
          tool_size));

  CellPtr neighbor_bottom = CellPtr(
      new Cell(
          PointPtr(
              new Point(*(current->get_center() + +(+direction) * tool_size))),
          tool_size));

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
  // End A star

  bool is_starting_cell = current == starting_cell;
  do {
    // Scan for new neighbor of current cell in counterclockwise order
    IdentifiableCellPtr neighbor = IdentifiableCellPtr(
        new IdentifiableCell(current->get_center() + direction * 2 * tool_size,
            2 * tool_size, communicator->get_robot_name()));
    std::cout << "  \033[1;33mneighbor:\033[0m " << neighbor->get_center()->x
        << "," << neighbor->get_center()->y;
    communicator->get_old_cells_message_from_server();
    communicator->read_message_then_update_old_cells();
    if (state_of(neighbor) == OLD) { // Check neighbor with current old cells
      // Go to next sub-cell
      if (communicator->ask_other_robot_still_alive(
          communicator->find_robot_name(neighbor))) {
        // Still alive
        communicator->set_current_cell(current);
        go_with(++direction, tool_size);
        continue;
      } else {
        // Dead
        if (state_of(neighbor) == OLD) { // Check again neighbor with new old cells
          communicator->set_current_cell(current);
          go_with(++direction, tool_size);
          continue;
        } else {
          std::cout << "\n";
          neighbor->set_parent(current);
          communicator->get_old_cells_message_from_server();
          communicator->read_message_then_update_old_cells();
          communicator->insert_old_cell(neighbor);
          std::string message = communicator->create_old_cells_message();
          communicator->write_old_cells_message(message);
          communicator->send_save_message_to_server(communicator->create_old_cells_message_to_send_to_server(message));
          communicator->set_current_cell(current);
          old_cells_for_backtrack.insert(old_cells_for_backtrack.end(),
              neighbor);
          go_with(direction++, tool_size);
          scan(neighbor);
          continue;
        }
      }
    }
    if (see_obstacle(direction, tool_size / 2)) { // Obstacle
      // Go to next sub-cell

      communicator->read_obstacle_message();
      communicator->obstacle_cells.insert(neighbor);
      std::string message = communicator->create_message_from_obstacle_cells();
      communicator->write_obstacle_message(message);

      communicator->set_current_cell(current);
      go_with(++direction, tool_size);
    } else { // New free neighbor
      std::cout << "\n";
      // Construct a spanning-tree edge
      neighbor->set_parent(current);
      communicator->get_old_cells_message_from_server();
      communicator->read_message_then_update_old_cells();
      communicator->insert_old_cell(neighbor);
      std::string message = communicator->create_old_cells_message();
      communicator->write_old_cells_message(message);
      communicator->send_save_message_to_server(communicator->create_old_cells_message_to_send_to_server(message));
      communicator->set_current_cell(current);
      old_cells_for_backtrack.insert(old_cells_for_backtrack.end(), neighbor);
      go_with(direction++, tool_size);
      scan(neighbor);
    }
  } while (direction % initial_direction
      != (is_starting_cell ? AT_RIGHT_SIDE : IN_BACK));
  // Back to sub-cell of parent
  if (!is_starting_cell) {
    communicator->set_current_cell(current);
    go_with(direction, tool_size);
  }
  std::cout << "\033[1;34mcurrent-\033[0m\033[1;31mEND:\033[0m "
      << current->get_center()->x << "," << current->get_center()->y << "\n";
  // Start backtrack
  // if (bplist.size() > 0) {
  //     refine_bplist();
  //     std::cout << "Backtrack list: " << bplist.size() << "\n";
  //     find_bpcell(current);
  //     bpmove(current);
  //   }
  // End backtrack
}

// void MstcOnline::find_bpcell(CellPtr current) {
//   std::cout << current->get_center()->x << ", " << current->get_center()->y
//       << std::endl;
//   start = check_vertex(current);
//   int time = 100;
//   for (std::set<CellPtr>::iterator i = bplist.begin(); i != bplist.end(); i++) {
//     CellPtr tmp = CellPtr(*i);
//     CellPtr neighbor_E = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(tmp->get_center()->x - tool_size,
//                     tmp->get_center()->y)), tool_size));
//     if (state_of(neighbor_E) == OLD) {
//       goal = check_vertex(neighbor_E);
//     }

//     CellPtr neighbor_W = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(tmp->get_center()->x + tool_size,
//                     tmp->get_center()->y)), tool_size));
//     if (state_of(neighbor_W) == OLD
//         && check_distance(current, neighbor_W)
//             < check_distance(current, neighbor_E)) {
//       goal = check_vertex(neighbor_W);
//     }

//     CellPtr neighbor_N = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(tmp->get_center()->x,
//                     tmp->get_center()->y + tool_size)), tool_size));
//     if (state_of(neighbor_N) == OLD) {
//       if (state_of(neighbor_E) == OLD
//           && check_distance(current, neighbor_N)
//               < check_distance(current, neighbor_E)) {
//         goal = check_vertex(neighbor_N);
//       }
//       if (state_of(neighbor_W) == OLD
//           && check_distance(current, neighbor_N)
//               < check_distance(current, neighbor_W)) {
//         goal = check_vertex(neighbor_N);
//       }
//     }

//     CellPtr neighbor_S = CellPtr(
//         new Cell(
//             PointPtr(
//                 new Point(tmp->get_center()->x,
//                     tmp->get_center()->y - tool_size)), tool_size));
//     if (state_of(neighbor_S) == OLD) {
//       goal = check_vertex(neighbor_S);
//     }
//     std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
//     std::vector<cost> d(num_vertices(g));
//     try {
//       // Call astar named parameter interface
//       boost::astar_search(g, start,
//           distance_heuristic<mygraph_t, cost, location*>(locations, goal),
//           boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(
//               astar_goal_visitor<vertex>(goal)));
//     } catch (found_goal &fg) { // Found a path to the goal
//       std::list<vertex> shortest_path;
//       for (vertex v = goal;; v = p[v]) {
//         shortest_path.push_front(v);
//         if (p[v] == v)
//           break;
//       }
//       if (d[goal] < time) {
//         backtrack_path = shortest_path;
//         starting_cell = tmp;
//         time = d[goal];
//       }
//     }
//   }

//   std::cout << std::endl << "Total travel time: " << time << std::endl;
//   std::cout << "\033[1;32mNew Starting Cell:\033[0m" << " "
//       << starting_cell->get_center()->x << ", "
//       << starting_cell->get_center()->y << std::endl;
//   std::list<vertex>::iterator spi = backtrack_path.begin();
//   for (++spi; spi != backtrack_path.end(); ++spi)
//     std::cout << "->" << locations[*spi].x << ", " << locations[*spi].y
//         << std::endl;
// }

bool MstcOnline::go_with(VectorPtr direction, double distance) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = last_position + direction * distance;

//  CellPtr new_cell = IdentifiableCellPtr(
//        new IdentifiableCell(
//            PointPtr(
//                new Point(new_position->x - tool_size / 2,
//                    new_position->y + tool_size / 2)), 2 * tool_size,
//            communicator->get_robot_name()));
//  communicator->set_current_cell(new_cell);

  bool succeed = go_to(new_position, STRICTLY);
  std::cout << "\n";
  return succeed;
}

void MstcOnline::insert_edge(CellPtr current, CellPtr neighbor, int insert) {
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

void MstcOnline::insert_cell_to_graph(CellPtr current, CellPtr neighbor,
    int vertex_current) {
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

int MstcOnline::check_vertex(CellPtr current) {
  for (int i = 0; i <= number_neighbor_cell; i++) {
    if (std::abs(locations[i].x - current->get_center()->x) < EPSILON
        && std::abs(locations[i].y - current->get_center()->y) < EPSILON)
      return i;
  }
  return -1;
}

void MstcOnline::bpmove(CellPtr current) {
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
  old_cells_for_backtrack.insert(old_cells_for_backtrack.end(), starting_cell);
  starting_cell->set_parent(
      CellPtr(
          new Cell(
              PointPtr(
                  new Point(starting_cell->get_center()->x,
                      starting_cell->get_center()->y - tool_size)),
              tool_size)));
//  bplist.erase(starting_cell);
//  check_rotate = -1;
//  straight = 1;
  scan(starting_cell);
}

bool MstcOnline::go_with_bpcell(PointPtr last_position, VectorPtr direction,
    double distance) {
  PointPtr new_position = PointPtr(
      new Point(last_position + direction * distance));
  return go_to_bpcell(new_position, STRICTLY);
}

bool MstcOnline::go_to_bpcell(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}

}
}
}
