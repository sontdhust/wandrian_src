/*
 * mstc_communicator.hpp
 *
 *  Created on: Mar 30, 2016
 *      Author: manhnh
 */

#ifndef WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_MSTC_COMMUNICATOR_HPP_
#define WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_MSTC_COMMUNICATOR_HPP_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/String.h>
#include <list>
#include "identifiable_cell.hpp"
#include "base_communicator.hpp"

#include <stdio.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <arpa/inet.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define MAX_SIZE 2000

namespace wandrian {
namespace environment {
namespace mstc {

class MstcCommunicator: public BaseCommunicator {

public:
  MstcCommunicator();
  ~MstcCommunicator();

  void write_old_cells_message_to_rosbag(std::string); // Write old cells
  void write_status_message_to_rosbag(std::string);
  std::string create_old_cells_message();
  std::string create_status_message(IdentifiableCellPtr); // robot_name, last x, last y, last time update, status
  void read_message_from_rosbag_then_update_old_cells(); // Read old cells data, update to local old cells

  bool ask_other_robot_still_alive(std::string);
  std::string find_robot_name(IdentifiableCellPtr);
  bool find_old_cell(IdentifiableCellPtr);
  void insert_old_cell(IdentifiableCellPtr);

  std::string read_old_cells_message_from_rosbag(); // Read old cells data, no update to local old cells
  void update_old_cells_from_message(std::string);
  std::string read_status_message(); // Read status data from ros bag
  void clear_robots_dead_old_cells(std::string, std::string, std::string);

  bool get_is_backtracking() const; // Check backtracking in function scan() of mstc_online.cpp
  void set_is_backtracking(bool = false);

  void write_obstacle_message_to_rosbag(std::string);
  void read_obstacle_message_from_rosbag();
  std::string create_message_from_obstacle_cells();
  void update_obstacle_cells_from_message(std::string);

  std::set<IdentifiableCellPtr, CellComp> obstacle_cells;

  int connect_server(std::string);
  void disconnect_server();
  std::string create_status_message_to_send_to_server(std::string);
  std::string create_old_cells_message_to_send_to_server(std::string);
  std::string create_my_new_old_cells_message_to_send_to_server(IdentifiableCellPtr);
  int send_save_message_to_server(std::string);
  int get_status_message_from_server();
  int get_old_cells_message_from_server();

  const std::string& get_ip_server() const;
  void set_ip_server(const std::string&);

private:
  std::list<IdentifiableCellPtr> old_cells; // Old cells with list
  std::set<IdentifiableCellPtr, CellComp> backtrack_cells;
  bool is_backtracking;
  struct sockaddr_in server;
  int sockfd;
  int countTotalRecvData;
  int countTotalSendData;
  int countRecvData;
  int countSendData;
  std::string ip_server;
};

typedef boost::shared_ptr<MstcCommunicator> MstcCommunicatorPtr;

}
}
}

#endif /* WANDRIAN_INCLUDE_ENVIRONMENT_MSTC_MSTC_COMMUNICATOR_HPP_ */
