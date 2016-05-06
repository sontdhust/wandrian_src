/*
 * mstc_communicator.cpp
 *
 *  Created on: Mar 30, 2016
 *      Author: manhnh
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <algorithm>
#include <sstream>

#include "../../../include/environment/mstc/mstc_communicator.hpp"

#define foreach BOOST_FOREACH

namespace wandrian {
namespace environment {
namespace mstc {

MstcCommunicator::MstcCommunicator() {
  is_backtracking = false;
  set_tool_size(0.5);
  countRecvData = 0;
  countSendData = 0;
  countTotalRecvData = 0;
  countTotalSendData = 0;
  sockfd = 0;
}

MstcCommunicator::~MstcCommunicator() {
}

void MstcCommunicator::write_old_cells_message_to_rosbag(std::string message) {
  // Write old cells
  ROS_INFO("[Writing]My old cells: %s", message.data());
  rosbag::Bag bag;
  bag.open("message.bag", rosbag::bagmode::Write);
  std_msgs::String str;
  str.data = message.data();
  bag.write("publisher_communication", ros::Time::now(), str);
  bag.close();
}

void MstcCommunicator::write_status_message_to_rosbag(std::string status) {
  // Write status
  rosbag::Bag status_bag;
  ROS_INFO("[Writing status]Status: %s", status.data());
  status_bag.open("status.bag", rosbag::bagmode::Write);
  std_msgs::String str_status;
  str_status.data = status.data();
  status_bag.write("status_publisher", ros::Time::now(), str_status);
  status_bag.close();
}

std::string MstcCommunicator::create_old_cells_message() {
  std::string messages;
  for (std::list<IdentifiableCellPtr>::iterator item = old_cells.begin();
      item != old_cells.end(); ++item) {
    std::stringstream messsage;
    messsage << (*item)->get_center()->x << "," << (*item)->get_center()->y
        << "," << (*item)->get_robot_name() << ";";
    messages.append(messsage.str());
  }
  return messages;
}

std::string MstcCommunicator::create_status_message(
    IdentifiableCellPtr last_cell) {
  bool check_added = false;
  std::string my_status;
  std::string all_robots_new_status = "";
  std::stringstream status;
  std::string status_from_ros_bag = read_status_message();
  status << this->get_robot_name() << "," << last_cell->get_center()->x << ","
      << last_cell->get_center()->y << "," << ros::Time::now() << ","
      << "[ALIVE];";
  my_status.append(status.str());
  if (status_from_ros_bag == "") {
    all_robots_new_status = my_status;
  } else {
    boost::char_separator<char> split_status(";");
    boost::tokenizer<boost::char_separator<char> > tokens(status_from_ros_bag,
        split_status);
    foreach (const std::string& status, tokens) {
      if (status.find(get_robot_name()) != std::string::npos) {
        // Found
        all_robots_new_status.append(my_status);
        check_added = true;
      } else {
        // Not found
        all_robots_new_status.append(status);
        all_robots_new_status.append(";");
      }
    }
    if (!check_added) {
      all_robots_new_status.append(my_status);
    }
  }
  return all_robots_new_status;
}

void MstcCommunicator::read_message_from_rosbag_then_update_old_cells() {
  update_old_cells_from_message(read_old_cells_message_from_rosbag());
}

bool MstcCommunicator::ask_other_robot_still_alive(
    std::string robot_name_want_ask) {
  bool result = true;
  std::string cell_string;
  int i;
  std::string status_string; // When robot is dead, change robot's status to [DEAD] and store to this variable
  if (robot_name_want_ask == this->get_robot_name()) {
    result = true;
  } else {
    boost::char_separator<char> split_status(";");
    boost::char_separator<char> split_information(",");
    boost::tokenizer<boost::char_separator<char> > tokens(read_status_message(),
        split_status);
    foreach (const std::string& status, tokens) {
      if (status.find(robot_name_want_ask) != std::string::npos) {
        // Found
        boost::tokenizer<boost::char_separator<char> > tokens(status,
            split_information);
        i = 1;
        foreach (const std::string& information, tokens) {
          if (i == 1) {
            status_string.append(information);
            status_string.append(",");
          } else if (i == 2) {
            status_string.append(information);
            status_string.append(",");
            cell_string.append(information);
            cell_string.append(",");
          } else if (i == 3) {
            status_string.append(information);
            status_string.append(",");
            cell_string.append(information);
            cell_string.append(",");
          } else if (i == 4) {
            status_string.append(information);

            int old_time = atoi(information.c_str());
            int current;
            std::stringstream ss;
            ss << ros::Time::now();
            current = atoi(ss.str().c_str());

            std::cout << "DHBKHNHEDSPIK56 <<" << current - old_time << ">>";
            if (current - old_time > 10) {
              // Robot was dead
              result = false;
              status_string.append("[DEAD];");
//              cell_string.append(get_robot_name());
              cell_string.append("DEAD_ROBOT");
              cell_string.append(";");

              // Update all status
              boost::char_separator<char> split(";");
              boost::tokenizer<boost::char_separator<char> > tokens(
                  read_status_message(), split);
              foreach (const std::string& dead_robot_status, tokens) {
                if (dead_robot_status.find(robot_name_want_ask)
                    != std::string::npos) {
                  // Found
                  continue;
                } else {
                  // Not found
                  status_string.append(dead_robot_status);
                  status_string.append(";");
                }
              }
              clear_robots_dead_old_cells(robot_name_want_ask, cell_string,
                  status_string);
            }
          } else if (i == 5) {
            if (information == "[DEAD];") {
              status_string.append(information);
              // Robot was dead
              result = false;
//              cell_string.append(get_robot_name());
              cell_string.append("DEAD_ROBOT");
              cell_string.append(";");

              // Update all status
              boost::char_separator<char> split(";");
              boost::tokenizer<boost::char_separator<char> > tokens(
                  read_status_message(), split);
              foreach (const std::string& dead_robot_status, tokens) {
                if (dead_robot_status.find(robot_name_want_ask)
                    != std::string::npos) {
                  // Found
                  continue;
                } else {
                  // Not found
                  status_string.append(dead_robot_status);
                  status_string.append(";");
                }
              }
              clear_robots_dead_old_cells(robot_name_want_ask, cell_string,
                  status_string);
            }
          }
          i++;
        }
      } else {
        // Not found
      }
    }
  }
  return result;
}

std::string MstcCommunicator::find_robot_name(
    IdentifiableCellPtr cell_to_find) {
  std::string robot_name = "NOT FOUND";
  int i;
  double x;
  double y;
  boost::char_separator<char> split_old_cells(";");
  boost::char_separator<char> split_point(",");
  boost::tokenizer<boost::char_separator<char> > tokens(
      create_old_cells_message(), split_old_cells);
  foreach (const std::string& cell, tokens) {
    if (robot_name != "NOT FOUND")
      break;
    boost::tokenizer<boost::char_separator<char> > tokens(cell, split_point);
    i = 1;
    foreach (const std::string& coordinates, tokens) {
      if (i == 1) {
        x = atof(coordinates.c_str());
      } else if (i == 2) {
        y = atof(coordinates.c_str());
      } else if (i == 3) {
        if ((x == cell_to_find->get_center()->x)
            && (y == cell_to_find->get_center()->y))
          robot_name = coordinates.c_str();
        break;
      }
      i++;
    }
  }
  return robot_name;
}

bool MstcCommunicator::find_old_cell(IdentifiableCellPtr cell) {
  bool value = false;
  for (std::list<IdentifiableCellPtr>::iterator item = old_cells.begin();
      item != old_cells.end(); ++item) {
    if (((*item)->get_center()->x == cell->get_center()->x)
        && ((*item)->get_center()->y == cell->get_center()->y)) {
      value = true;
      break;
    }
  }
  return value;
}

void MstcCommunicator::insert_old_cell(IdentifiableCellPtr cell) {
  old_cells.push_back(
      IdentifiableCellPtr(
          new IdentifiableCell(cell->get_center(), cell->get_size(),
              this->get_robot_name())));
}

std::string MstcCommunicator::read_old_cells_message_from_rosbag() {
  rosbag::Bag bag;
  std::string msg;
  try {
    bag.open("message.bag", rosbag::bagmode::Read);
  } catch (rosbag::BagIOException &e) {
    write_old_cells_message_to_rosbag("");
    bag.open("message.bag", rosbag::bagmode::Read);
  }
  std::vector<std::string> topics;
  topics.push_back(std::string("publisher_communication"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL) {
      msg.append(s->data.c_str());
    }
  }
  bag.close();
  ROS_INFO("[Reading]Ros bag old cells: %s", msg.data());
  return msg;
}

void MstcCommunicator::update_old_cells_from_message(std::string msg) {
  old_cells.clear();
  double x = 0.0;
  double y = 0.0;
  std::string robot_name;
  int i;
  boost::char_separator<char> split_old_cells(";");
  boost::char_separator<char> split_point(",");
  boost::tokenizer<boost::char_separator<char> > tokens(msg, split_old_cells);
  foreach (const std::string& cell, tokens) {
    boost::tokenizer<boost::char_separator<char> > tokens(cell, split_point);
    i = 1;
    foreach (const std::string& coordinates, tokens) {
      if (i == 1) {
        x = atof(coordinates.c_str());
      } else if (i == 2) {
        y = atof(coordinates.c_str());
      } else if (i == 3) {
        robot_name = coordinates.c_str();
      }
      i++;
    }
    IdentifiableCellPtr old_cell = IdentifiableCellPtr(
        new IdentifiableCell(PointPtr(new Point(x, y)),
            2 * this->get_tool_size(), robot_name));
    old_cells.push_back(old_cell);
  }
  ROS_INFO("[Reading]My old cells: %s", create_old_cells_message().data());
}

std::string MstcCommunicator::read_status_message() {
  rosbag::Bag status_bag;
  std::string msg;
  try {
    status_bag.open("status.bag", rosbag::bagmode::Read);
  } catch (rosbag::BagIOException &e) {
    write_status_message_to_rosbag("");
    status_bag.open("status.bag", rosbag::bagmode::Read);
  }
  std::vector<std::string> topics;
  topics.push_back(std::string("status_publisher"));

  rosbag::View view(status_bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL) {
      msg.append(s->data.c_str());
    }
  }
  status_bag.close();
  return msg;
}

void MstcCommunicator::clear_robots_dead_old_cells(std::string dead_robot_name,
    std::string last_cell, std::string last_status) {

  old_cells.clear();
  double x = 0.0;
  double y = 0.0;
  std::string robot_name;
  int i;
  boost::char_separator<char> split_point(",");

  // Read old cells data from ros bag
  std::string old_old_cells = read_old_cells_message_from_rosbag();
  std::string new_old_cells;
  // Clear dead robot's old cells
  boost::char_separator<char> split_old_cell(";");
  boost::tokenizer<boost::char_separator<char> > tokens(old_old_cells,
      split_old_cell);
  foreach (const std::string& cell, tokens) {
    if (cell.find(dead_robot_name) != std::string::npos) {
      // Found
      continue;
    } else {
      // Not found
      new_old_cells.append(cell);
      new_old_cells.append(";");
      // Update local old cells
      boost::tokenizer<boost::char_separator<char> > tokens(cell, split_point);
      i = 1;
      foreach (const std::string& coordinates, tokens) {
        if (i == 1) {
          x = atof(coordinates.c_str());
        } else if (i == 2) {
          y = atof(coordinates.c_str());
        } else if (i == 3) {
          robot_name = coordinates.c_str();
        }
        i++;
      }
      IdentifiableCellPtr old_cell = IdentifiableCellPtr(
          new IdentifiableCell(PointPtr(new Point(x, y)),
              2 * this->get_tool_size(), robot_name));
      old_cells.push_back(old_cell);
    }
  }
  new_old_cells.append(last_cell);

  boost::tokenizer<boost::char_separator<char> > token_of_last_cell(last_cell,
      split_point);
  i = 1;
  foreach (const std::string& coordinates, token_of_last_cell) {
    if (i == 1) {
      x = atof(coordinates.c_str());
    } else if (i == 2) {
      y = atof(coordinates.c_str());
    } else if (i == 3) {
      robot_name = coordinates.c_str();
    }
    i++;
  }
  IdentifiableCellPtr old_cell = IdentifiableCellPtr(
      new IdentifiableCell(PointPtr(new Point(x, y)), 2 * this->get_tool_size(),
          robot_name));
  old_cells.push_back(old_cell);

// Write new old cells to ros bag and update status all robots
  write_old_cells_message_to_rosbag(new_old_cells);
  write_status_message_to_rosbag(last_status);
}

bool MstcCommunicator::get_is_backtracking() const {
  return is_backtracking;
}

void MstcCommunicator::set_is_backtracking(bool isBacktracking) {
  is_backtracking = isBacktracking;
}

// Start handle obstacle

void MstcCommunicator::write_obstacle_message_to_rosbag(std::string message) {
  ROS_INFO("[Writing]My obstacle cells: %s", message.data());
  rosbag::Bag obstacle_bag;
  obstacle_bag.open("obstacle.bag", rosbag::bagmode::Write);
  std_msgs::String str_obstacle;
  str_obstacle.data = message.data();
  obstacle_bag.write("obstacle_publisher", ros::Time::now(), str_obstacle);
  obstacle_bag.close();
}

void MstcCommunicator::read_obstacle_message_from_rosbag() {
  ROS_INFO("I am Here");
  rosbag::Bag obstacle_bag;
  std::string msg;
  try {
    obstacle_bag.open("obstacle.bag", rosbag::bagmode::Read);
  } catch (rosbag::BagIOException &e) {
    write_obstacle_message_to_rosbag("");
    obstacle_bag.open("obstacle.bag", rosbag::bagmode::Read);
  }
  std::vector<std::string> topics;
  topics.push_back(std::string("obstacle_publisher"));

  rosbag::View view(obstacle_bag, rosbag::TopicQuery(topics));
  foreach(rosbag::MessageInstance const m, view) {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL) {
      msg.append(s->data.c_str());
    }
  }

  obstacle_bag.close();
  ROS_INFO("[Reading]Ros bag obstacle cells: %s", msg.data());
  update_obstacle_cells_from_message(msg);
}

std::string MstcCommunicator::create_message_from_obstacle_cells() {
  std::string msg;
  std::set<IdentifiableCellPtr, CellComp> temp_obstacle_cells =
      this->obstacle_cells;
  // for (int i = 0; i <= temp_old_cells.size(); i++) {
  while (temp_obstacle_cells.size() != 0) {
    IdentifiableCellPtr temp_cell = *temp_obstacle_cells.begin();
    std::stringstream tmp;
    tmp << temp_cell->get_center()->x << "," << temp_cell->get_center()->y
        << ";";
    msg.append(tmp.str());
    temp_obstacle_cells.erase(temp_cell);
  }
  return msg;
}

void MstcCommunicator::update_obstacle_cells_from_message(std::string msg) {
  double x = 0.0;
  double y = 0.0;
  IdentifiableCellPtr temp_cell;
  int i;
  boost::char_separator<char> split_obstacle_cells(";");
  boost::char_separator<char> split_point(",");
  boost::tokenizer<boost::char_separator<char> > tokens(msg,
      split_obstacle_cells);
  BOOST_FOREACH (const std::string& cell, tokens) {
    boost::tokenizer<boost::char_separator<char> > tokens(cell, split_point);
    i = 1;
    BOOST_FOREACH (const std::string& coordinates, tokens) {
      if (i == 1) {
        x = atof(coordinates.c_str());
      } else if (i == 2) {
        y = atof(coordinates.c_str());
      }
      i++;
    }
    // temp_cell = IdentifiableCellPtr(new Cell(PointPtr(new Point(x, y)), 2 * get_tool_size()));
    temp_cell = IdentifiableCellPtr(
        new IdentifiableCell(PointPtr(new Point(x, y)),
            2 * this->get_tool_size(), robot_name));
    this->obstacle_cells.insert(temp_cell);
  }
  ROS_INFO("[Reading]My obstacle cells: %s",
      create_message_from_obstacle_cells().data());
}

// End handle obstacle

int MstcCommunicator::connect_server(std::string ip_server) {
  char *sendMes = (char *) malloc(MAX_SIZE);
  char *recvMes = (char *) malloc(MAX_SIZE);
  strcpy(sendMes, "");
  strcpy(recvMes, "");
  int countTotalRecvData = 0, countTotalSendData = 0, countRecvData = 0,
      countSendData = 0;
  printf("Creat a socket, please wait...\n");
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    printf("Socket retrieve failed!\n");
    return 1;
  } else
    printf("Socket retrieve success!...\n");
  memset(&server, '0', sizeof(server));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(ip_server.c_str());
  server.sin_port = htons(5678);
  if (connect(sockfd, (struct sockaddr *) &server, sizeof(server)) == -1) {
    printf("Connect failed!\n");
    return 1;
  } else
    printf("Connected. Ready to communicate with server!\n");
  return 0;
}

void MstcCommunicator::disconnect_server() {
  shutdown(sockfd, 2);
}

std::string MstcCommunicator::create_status_message_to_send_to_server(
    std::string origin_message) {
  std::string output_message = "[SAVE_STATUS]|" + origin_message;
  return output_message;
}

std::string MstcCommunicator::create_my_new_old_cells_message_to_send_to_server(
    IdentifiableCellPtr new_cell) {
  std::string return_mess;
  std::stringstream messsage;
  messsage << (new_cell)->get_center()->x << "," << (new_cell)->get_center()->y
      << "," << (new_cell)->get_robot_name() << ";";
  return_mess.append(messsage.str());
  return return_mess;
}

std::string MstcCommunicator::create_old_cells_message_to_send_to_server(
    std::string origin_message) {
  std::string output_message = "[SAVE_OLD_CELLS]|" + origin_message;
  return output_message;
}

int MstcCommunicator::send_save_message_to_server(std::string message) {
  char *sendMes = (char *) malloc(MAX_SIZE);
  char *recvMes = (char *) malloc(MAX_SIZE);

  strcpy(sendMes, message.c_str());
  strcpy(recvMes, "");
  countSendData = send(sockfd, sendMes, strlen(sendMes), 0);
  if (countSendData == -1) {
    printf("Send data failed!\n");
    return 1;
  } else {
    countTotalSendData = countTotalSendData + countSendData;
    countRecvData = recv(sockfd, recvMes, 2000, 0);
    if (countRecvData == -1) {
      printf("Receive data failed!\n");
      return 1;
    } else {
      countTotalRecvData = countTotalRecvData + countRecvData;
      printf("Server reply: %s\n", recvMes);
      recvMes = (char *) malloc(MAX_SIZE);
      strcpy(recvMes, "");
    }
  }
  return 0;
}

int MstcCommunicator::get_status_message_from_server() {
  char *sendMes = (char *) malloc(MAX_SIZE);
  char *recvMes = (char *) malloc(MAX_SIZE);
  std::string new_status;
  std::cout << "I am here, old cells";
  strcpy(sendMes, "[GIVE_ME_STATUS]|_");
  strcpy(recvMes, "");
  countSendData = send(sockfd, sendMes, strlen(sendMes), 0);
  if (countSendData == -1) {
    printf("Send data failed!\n");
    return 1;
  } else {
    countTotalSendData = countTotalSendData + countSendData;
    countRecvData = recv(sockfd, recvMes, 2000, 0);
    if (countRecvData == -1) {
      printf("Receive data failed!\n");
      return 1;
    } else {
      printf("Received data!\n");
      countTotalRecvData = countTotalRecvData + countRecvData;
      // printf("Server reply: %s\n", recvMes);
      new_status.assign(recvMes, countRecvData);
      // recvMes = (char *) malloc(MAX_SIZE);
      // strcpy(recvMes, "");
      // boost::char_separator<char> split_str("|");
      // int i = 1;
      // boost::tokenizer<boost::char_separator<char> > tokens(new_status,
      //     split_str);
      // BOOST_FOREACH (const std::string& mess, tokens) {
      //   {
      //     if (i == 1) {
      //       // Do nothing
      //     } else if (i == 2) {
      //       write_status_message(mess);
      //     }
      //     i++;
      //   }
      // }
      if (new_status == "no thing") {
        write_status_message_to_rosbag("");
      } else {
        write_status_message_to_rosbag(new_status);
      }
    }
  }
  return 0;
}

int MstcCommunicator::get_old_cells_message_from_server() {
  char *sendMes = (char *) malloc(MAX_SIZE);
  char *recvMes = (char *) malloc(MAX_SIZE);
  std::string new_old_cells;
  std::cout << "I am here, old cells";
  strcpy(sendMes, "[GIVE_ME_OLD_CELLS]|_");
  strcpy(recvMes, "");
  countSendData = send(sockfd, sendMes, strlen(sendMes), 0);
  if (countSendData == -1) {
    printf("Send data failed!\n");
    return 1;
  } else {
    countTotalSendData = countTotalSendData + countSendData;
    countRecvData = recv(sockfd, recvMes, 2000, 0);
    if (countRecvData == -1) {
      printf("Receive data failed!\n");
      return 1;
    } else {
      printf("Received data!\n");
      countTotalRecvData = countTotalRecvData + countRecvData;
      // printf("Server reply: %s\n", recvMes);
      new_old_cells.assign(recvMes, countRecvData);
      // recvMes = (char *) malloc(MAX_SIZE);
      // strcpy(recvMes, "");

      // boost::char_separator<char> split_str("|");
      // int i = 1;
      // boost::tokenizer<boost::char_separator<char> > tokens(new_old_cells,
      //     split_str);
      // BOOST_FOREACH (const std::string& mess, tokens) {
      //   {
      //     if (i == 1) {
      //       // Do nothing
      //     } else if (i == 2) {
      //       write_old_cells_message(mess);
      //     }
      //     i++;
      //   }
      // }
      if (new_old_cells == "no thing") {
        write_old_cells_message_to_rosbag("");
      } else {
        write_old_cells_message_to_rosbag(new_old_cells);
      }
    }
  }
  return 0;
}

const std::string& MstcCommunicator::get_ip_server() const {
  return ip_server;
}

void MstcCommunicator::set_ip_server(const std::string& ipServer) {
  ip_server = ipServer;
}

}
}
}

