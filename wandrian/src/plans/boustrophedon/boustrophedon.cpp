/*
 * boustrophedon.cpp
 *
 *  Created on: Mar 25, 2016
 *      Author: cslab
 */

#include "../../../include/plans/boustrophedon/boustrophedon.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon {

Boustrophedon::Boustrophedon() :
    robot_size(0) {
}

Boustrophedon::~Boustrophedon() {
}

// double environment_size

void Boustrophedon::initialize(PointPtr starting_point, double robot_size,
    std::string namefile) {
  this->robot_size = robot_size;
  this->map = ExtendedMapPtr(new ExtendedMap(namefile));
  this->map->build();
  path.insert(path.end(), starting_point);
}

ExtendedMapPtr Boustrophedon::get_map() {
  return map;
}

void Boustrophedon::cover() {
  // old_cells.insert(starting_cell);
  boustrophedon_cd();
}

bool Boustrophedon::go_to(PointPtr position, bool flexibility) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  return BasePlan::go_to(position, flexibility);
}

bool Boustrophedon::go_into(SpacePtr space) {
  double x = (space->get_center()->x * 2 - space->get_size_x() + robot_size)
      / 2;
  double y = (space->get_center()->y * 2 - space->get_size_y() + robot_size)
      / 2;

  PointPtr starting_point = PointPtr(new Point(x, y));
  go_to(starting_point, STRICTLY);
  std::cout << "\033[1;34mLast_position-\033[0m\033[1;31m\033[0m: "
      << starting_point->x << "," << starting_point->y << "\n";
  std::cout << "\033[1;34mSize_Space-\033[0m\033[1;31m\033[0m: "
      << space->get_size_x() << "," << space->get_size_y() << "\n";

  PointPtr last_position;
  PointPtr new_position;
  double flag;
  flag = robot_size;
  for (int i = 0; i < int(space->get_size_x() / robot_size + EPSILON); ++i) {
    if (i != 0) {
      last_position = path.back();
      new_position = PointPtr(
          new Point(last_position->x + robot_size, last_position->y));
      go_to(new_position, STRICTLY);
    }
    std::cout << "\033[1;34mNumber_line-\033[0m\033[1;31m\033[0m: " << i
        << "\n";
    for (int j = 0; j < int(space->get_size_y() / robot_size - 1 + EPSILON);
        ++j) {
      last_position = path.back();
      new_position = PointPtr(
          new Point(last_position->x, last_position->y + flag));
      go_to(new_position, STRICTLY);
    }
    flag = -flag;
  }
  return true;
}

void Boustrophedon::dfs(SpacePtr space) {
  std::list<SpacePtr>::iterator inspectLC;
  space->status_visited = true;
  double x, y;
  std::cout << "Visit Space" << space->get_size_x() << std::endl;
  go_into(space);

  for (inspectLC = space->children.begin(); inspectLC != space->children.end();
      ++inspectLC) {
    if ((*inspectLC)->status_visited == false) {
      go_to((*inspectLC)->point_backtrack, STRICTLY);
      go_to(
          PointPtr(
              new Point((*inspectLC)->point_backtrack->x + robot_size,
                  (*inspectLC)->point_backtrack->y)), STRICTLY);
      dfs(*inspectLC);
      go_to(
          PointPtr(
              new Point((*inspectLC)->point_backtrack->x + robot_size,
                  (*inspectLC)->point_backtrack->y)), STRICTLY);
      go_to((*inspectLC)->point_backtrack, STRICTLY);
    }
  }
  if (space->point_backtrack) {
    go_to(
        PointPtr(
            new Point(
                space->get_center()->x + space->get_size_x() / 2
                    - robot_size / 2, space->point_backtrack->y)), STRICTLY);
  }
}

std::list<SpacePtr> Boustrophedon::create_list_space(RectanglePtr environment,
    std::list<VerticesPtr> list_vertices) {
  std::list<VerticesPtr> listvertices_temp;
  std::list<VerticesPtr>::iterator inspectLV;
  std::list<VerticesPtr>::iterator inspectLVT;
  std::list<SpacePtr> list_space;
  std::list<SpacePtr>::iterator inspectLS;
  std::list<SpacePtr>::iterator inspectLS_temp;
  PointPtr center_temp;
  int i = 1, j = 1;
  double size_x = 0, size_y = 0;
  VerticesPtr vertices_previous;
  std::list<PointPtr> list_point;
  std::list<PointPtr>::iterator inspectLP;

  // Create list space!
  for (inspectLV = list_vertices.begin(), j = 1, i = 1;
      j <= list_vertices.size(); ++inspectLV) {
    std::cout << "\n" << "V" << j++ << "(" << (*inspectLV)->get_position()->x
        << ", " << (*inspectLV)->get_position()->y << " )" << std::endl;

    listvertices_temp.sort(Vertices::compare_positions_y);
    if (listvertices_temp.empty() && (inspectLV == list_vertices.begin())) {
      std::cout << "Empty1" << std::endl;
      listvertices_temp.push_back(*inspectLV);
      ++inspectLV;
      ++j;
      listvertices_temp.push_back(*inspectLV);
      continue;
    }

    std::cout << "List temp current " << std::endl;
    for (inspectLVT = listvertices_temp.begin();
        inspectLVT != listvertices_temp.end(); ++inspectLVT) {
      std::cout << "V" << "(" << (*inspectLVT)->get_position()->x << ", "
          << (*inspectLVT)->get_position()->y << " )" << std::endl;
    }

    // 1. Create one space
    //   + Center
    //   + Size
    // 2. Push + pop: Temp

    if ((*inspectLV)->left_compared_center()
        || ((*inspectLV)->get_polygon() == environment)) {

      for (inspectLVT = listvertices_temp.begin();
          inspectLVT != listvertices_temp.end(); ++inspectLVT) {
        if (inspectLVT == listvertices_temp.begin()) {
          vertices_previous = *inspectLVT;
          continue;
        }
        if ((*inspectLVT)->get_position()->y
            < (*inspectLV)->get_position()->y) {
          vertices_previous = *inspectLVT;
          continue;
        }
        break;
      }
      std::cout << "Into1" << std::endl;
      // Create space
      size_x = (*inspectLV)->get_position()->x
          - (*inspectLVT)->get_position()->x;
      size_y = (*inspectLVT)->get_position()->y
          - vertices_previous->get_position()->y;

      center_temp = PointPtr(
          new Point((*inspectLVT)->get_position()->x + size_x / 2,
              (*inspectLVT)->get_position()->y - size_y / 2));
      std::cout << "Create Space:" << size_x << "," << size_y << std::endl;
      std::cout << "Center Space:" << center_temp->x << "," << center_temp->y
          << std::endl;

      // Remove : two vetices space left
      listvertices_temp.remove(vertices_previous);
      listvertices_temp.remove(*inspectLVT);

      // Push: two vertices space right
      if ((*inspectLV)->get_position()->y
          != environment->get_center()->y + environment->get_height() / 2) {
        listvertices_temp.push_back(
            VerticesPtr(
                new Vertices(
                    PointPtr(
                        new Point(center_temp->x + size_x / 2,
                            center_temp->y + size_y / 2)),
                    RectanglePtr(new Rectangle(center_temp, size_x, size_y)))));
        listvertices_temp.push_back(*inspectLV);
      }
      ++inspectLV;
      ++j;
      if ((*inspectLV)->get_position()->y
          != environment->get_center()->y - environment->get_height() / 2) {
        listvertices_temp.push_back(
            VerticesPtr(
                new Vertices(
                    PointPtr(
                        new Point(center_temp->x + size_x / 2,
                            center_temp->y - size_y / 2)),
                    RectanglePtr(new Rectangle(center_temp, size_x, size_y)))));
        listvertices_temp.push_back(*inspectLV);
      }
    } else {
      if ((fabs(
          (*inspectLV)->get_position()->x - environment->get_center()->x
              - environment->get_width() / 2) < EPSILON)) {
        if ((*inspectLV)->upon_compared_center()) {
          continue;
        }
      }
      if (((*inspectLV)->get_position()->y
          == environment->get_center()->y - environment->get_height() / 2)
          || ((*inspectLV)->get_position()->y
              == environment->get_center()->y + environment->get_height() / 2)) {
        listvertices_temp.push_back(*inspectLV);
        continue;
      }
      inspectLVT = --listvertices_temp.end();
      //      if ((*inspectLV)->get_position()->y == (*inspectLVT)->get_position()->y) {
      //        listvertices_temp.push_back(*inspectLV);
      //        ++inspectLV;
      //        ++j;
      //        continue;
      //      }
      for (inspectLVT = listvertices_temp.begin();
          inspectLVT != listvertices_temp.end(); ++inspectLVT) {
        std::cout << "V current " << (*inspectLV)->get_position()->y
            << std::endl;
        std::cout << "V temp " << (*inspectLVT)->get_position()->y << std::endl;
        if ((*inspectLV)->get_position()->y
            == (*inspectLVT)->get_position()->y) {
          std::cout << (*inspectLV)->get_position()->y << std::endl;
          break;
        }
        vertices_previous = *inspectLVT;
      }
      if ((*inspectLV)->upon_compared_center()) {
        vertices_previous = *inspectLVT;
        ++inspectLVT;
      }
      size_x = (*inspectLV)->get_position()->x
          - vertices_previous->get_position()->x;
      size_y = (*inspectLVT)->get_position()->y
          - vertices_previous->get_position()->y;
      center_temp = PointPtr(
          new Point(vertices_previous->get_position()->x + size_x / 2,
              vertices_previous->get_position()->y + size_y / 2));

      std::cout << "Create Space:" << size_x << "," << size_y << std::endl;
      std::cout << "Center Space:" << center_temp->x << "," << center_temp->y
          << std::endl;

      // Remove : two vetices space left
      listvertices_temp.remove(vertices_previous);
      listvertices_temp.remove(*inspectLVT);
      if ((*inspectLV)->upon_compared_center()) {
        listvertices_temp.push_back(
            VerticesPtr(
                new Vertices(
                    PointPtr(
                        new Point(center_temp->x + size_x / 2,
                            center_temp->y + size_y / 2)),
                    RectanglePtr(new Rectangle(center_temp, size_x, size_y)))));
        std::cout << center_temp->x + size_x / 2 << center_temp->y + size_y / 2
            << std::endl;
      } else {
        listvertices_temp.push_back(
            VerticesPtr(
                new Vertices(
                    PointPtr(
                        new Point(center_temp->x + size_x / 2,
                            center_temp->y - size_y / 2)),
                    RectanglePtr(new Rectangle(center_temp, size_x, size_y)))));
      }
      if ((*inspectLV)->get_position()->x
          == environment->get_center()->x + environment->get_width() / 2) {
        ++inspectLV;
        ++j;
      }
    }
    if ((size_x != 0) && (size_y != 0)) {
      list_space.push_back(SpacePtr(new Space(center_temp, size_x, size_y)));
    }
  }

  std::cout << "Starting add parent!" << "\n";
  list_space.sort(Space::compare_positions_x);
  std::cout << list_space.size() << "\n";
  for (inspectLS = --list_space.end(), i = 1; inspectLS != list_space.end();
      --inspectLS) {
    std::cout << "Space " << (*inspectLS)->get_center()->x << ","
        << (*inspectLS)->get_center()->y << std::endl;
    for (inspectLS_temp = list_space.begin(), i = 1;
        inspectLS_temp != list_space.end(); ++inspectLS_temp) {
      if (Space::is_parent(*inspectLS_temp, *inspectLS)) {
        std::cout << " Parent (" << (*inspectLS_temp)->get_center()->x << " ,"
            << (*inspectLS_temp)->get_center()->y << ")" << std::endl;
        std::cout << "Children (" << (*inspectLS)->get_center()->x << " ,"
            << (*inspectLS)->get_center()->y << ")" << std::endl;
        (*inspectLS_temp)->children.push_back(*inspectLS);
        (*inspectLS)->set_parent(*inspectLS_temp);
        (*inspectLS)->set_point_backtrack(*inspectLS_temp, *inspectLS,
            robot_size);
        break;
      }
      std::cout << "Find \n";
    }
  }
  return list_space;
}

std::list<VerticesPtr> Boustrophedon::create_list_vertices(
    RectanglePtr environment, std::list<RectanglePtr> listobstacle) {
  std::list<PointPtr> list_point;
  std::list<PointPtr>::iterator inspectLP;
  std::list<VerticesPtr> list_vertices;
  std::list<VerticesPtr>::iterator inspectLV;
  std::list<RectanglePtr>::iterator inspectLO;
  int i = 1, j = 1;
  std::cout << "Environment : 0(" << environment->get_center()->x << " ,"
      << environment->get_center()->y << ")" << " Size:" << "("
      << environment->get_width() << " ," << environment->get_height() << ")"
      << std::endl;

  // Create list vertices
  for (inspectLO = listobstacle.begin(), i = 1; inspectLO != listobstacle.end();
      ++inspectLO) {
    std::cout << "Obstacle " << i++ << ":O(" << (*inspectLO)->get_center()->x
        << " , " << (*inspectLO)->get_center()->y << ")" << std::endl;
    list_point = (*inspectLO)->get_points();

    for (inspectLP = list_point.begin(), j = 1; inspectLP != list_point.end();
        ++inspectLP) {
      std::cout << "V" << j++ << "(" << (*inspectLP)->x << ", "
          << (*inspectLP)->y << " )" << std::endl;
      list_vertices.push_back(
          VerticesPtr(new Vertices(*inspectLP, *inspectLO)));
    }
    list_point.clear();
  }

  // Add vertices enviroment
  list_point = environment->get_points();
  for (inspectLP = list_point.begin(); inspectLP != list_point.end();
      ++inspectLP) {
    std::cout << "O" << (*inspectLP)->x << (*inspectLP)->y << std::endl;
    for (inspectLV = list_vertices.begin(), j = 0;
        inspectLV != list_vertices.end(); ++inspectLV) {
      if (((*inspectLV)->get_position()->x == (*inspectLP)->x)
          && ((*inspectLV)->get_position()->y == (*inspectLP)->y)) {
        j = 1;
        list_vertices.remove(*inspectLV);
        break;
      }
    }
    if (j != 1) {
      list_vertices.push_back(
          VerticesPtr(new Vertices(*inspectLP, environment)));
    }
  }
  list_vertices.sort(Vertices::compare_positions_x);
  std::cout << " " << std::endl;
  return list_vertices;
}

void Boustrophedon::boustrophedon_cd() {
  std::list<VerticesPtr> list_vertices;
  std::list<SpacePtr> list_space;
  std::list<SpacePtr>::iterator inspectLS;
  std::list<SpacePtr>::iterator inspectLS_child;
  std::list<VerticesPtr>::iterator u;
  int i, j;
  PointPtr point_temp;

  // Create vertices
  list_vertices = create_list_vertices(map->get_boundary(),
      map->get_obstacles());
  for (u = list_vertices.begin(), i = 1; u != list_vertices.end(); ++u) {
    std::cout << "V " << i++ << " ( " << (*u)->get_position()->x << ", "
        << (*u)->get_position()->y << " )\n";
  }
  list_space = create_list_space(map->get_boundary(), list_vertices);

  // Create list space
  std::cout << " " << std::endl;
  for (inspectLS = list_space.begin(), i = 1; inspectLS != list_space.end();
      ++inspectLS) {
    std::cout << "O" << i++ << "(" << (*inspectLS)->get_center()->x << " ,"
        << (*inspectLS)->get_center()->y << " ) ";
    std::cout << "Size(" << (*inspectLS)->get_size_x() << " ,"
        << (*inspectLS)->get_size_y() << " )" << std::endl;
    for (inspectLS_child = (*inspectLS)->children.begin(), j = 1;
        inspectLS_child != (*inspectLS)->children.end(); ++inspectLS_child) {
      if (inspectLS_child == (*inspectLS)->children.begin()) {
        std::cout << "Children:" << std::endl;
      }
      std::cout << "O" << j++ << " (" << (*inspectLS_child)->get_center()->x
          << " ," << (*inspectLS_child)->get_center()->y << " ) ";
      std::cout << "Size(" << (*inspectLS_child)->get_size_x() << " ,"
          << (*inspectLS_child)->get_size_y() << " )" << std::endl;
    }
    std::cout << " " << std::endl;
  }

  for (inspectLS = list_space.begin(), i = 1; inspectLS != list_space.end();
      ++inspectLS) {
    std::cout << "Space:" << ": " << std::endl;
    if ((*inspectLS)->status_visited == false) {
      dfs(*inspectLS);
    }
  }
}

}
}
}
