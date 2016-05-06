#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <ctype.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/select.h>
#include <poll.h>

#include <ros/ros.h>
#include <arpa/inet.h>

#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#define MAX_SIZE 2000
#define TIME_OUT 10000
#define MAX_CONN 1024
int main(int argc, char* argv[]) {
  std::string status = "";
  std::string old_cells = "";
  std::string str_mess = "";
  int check_action;
  int sockfd, acceptfd, fd;
  int choose;
  int countTotalRecvData[MAX_CONN], countTotalSendData[MAX_CONN],
      countRecvData = 0, countSendData = 0;
  struct sockaddr_in server, clients;
  socklen_t socksize = sizeof(struct sockaddr_in);
  char * message;
  // char *message=malloc(MAX_SIZE);
  message = (char*) malloc(MAX_SIZE);
  strcpy(message, "");
  struct pollfd client[MAX_CONN];
  int max_client;
  int i, j;
  int numberClient = 0;
  int activity;
  printf("Creat a socket, please wait...\n");
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    printf("Socket retrieve failed!\n");
    return 1;
  } else
    printf("Socket retrieve success!...\n");
  memset(&server, '0', sizeof(server));
  server.sin_family = AF_INET;
  if (argv[1] == "") {
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
  } else {
    server.sin_addr.s_addr = inet_addr(argv[1]);
  }
  server.sin_port = htons(5678);
  printf("Bind the socket, please wait...\n");
  if (bind(sockfd, (struct sockaddr*) &server, sizeof(server)) == -1) {
    printf("Bind failed...\n");
    return 1;
  } else
    printf("Bind done...\n");
  printf("Listen on the socket, please wait...\n");
  if ((listen(sockfd, 1)) == -1) {
    printf("Failed to listen\n");
    return 1;
  } else
    printf("Success to listen\n");
  client[0].fd = sockfd;
  client[0].events = POLLIN;
  max_client = 0;
  for (i = 1; i < MAX_CONN; i++) {
    client[i].fd = -1;
    countTotalSendData[i] = 0;
    countTotalRecvData[i] = 0;
  }
  printf("Ready to communicate with client.\n");
  for (;;) {
    activity = poll(client, max_client + 1, TIME_OUT);
    if (activity < 0) {
      printf("Poll failed!\n");
      return 1;
    }
    if (client[0].revents & POLLIN) {
      acceptfd = accept(sockfd, (struct sockaddr *) &clients, &socksize);
      //countTotalSendData=0;
      //countTotalRecvData=0;
      if (acceptfd == -1) {
        printf("Accept failed!\n");
        return 1;
      } else {
        numberClient++;
        printf(
            "Accepted request! Having %d client(s) connect to server in this time.\n",
            numberClient);
      }
      for (i = 1; i <= MAX_CONN; i++) {
        if (client[i].fd == -1) {
          client[i].fd = acceptfd;
          break;
        }
      }
      if (i == MAX_CONN) {
        printf("FULL SLOTS.\n");
        return 1;
      }
      client[i].events = POLLIN;
      if (i > max_client)
        max_client = i;
      if (--activity <= 0)
        continue;
    }
    for (j = 1; j <= max_client; j++) {
      if ((fd = client[j].fd) == -1)
        continue;
      if (client[j].revents & (POLLIN | POLLERR)) {
        //while(1){
        // message=malloc(MAX_SIZE);
        message = (char*) malloc(MAX_SIZE);
        strcpy(message, "");
        countRecvData = recv(fd, message, 2000, 0);
        if (strcmp(message, "") != 0){
          printf("Message from client in port %d: %s\n", fd, message);
        }
        if (countRecvData == -1) {
          printf("Receive data failed!\n");
          shutdown(fd, 2);
          shutdown(sockfd, 2);
          return 1;
        } else if ((strcmp(message, "q") == 0) || (strcmp(message, "Q") == 0)) {
          numberClient--;
          printf(
              "Client in port %d disconnected! Having %d client(s) connect to server in this time. \n",
              fd, numberClient);
          printf("Total size of data received from port %d: %d\n", fd,
              countTotalRecvData[j]);
          printf("Total size of data sent to port %d: %d\n", fd,
              countTotalSendData[j]);
          shutdown(fd, 2);
          client[j].fd = -1;
          if (numberClient == 0) {
            do {
              printf("Do you want to keep server working?\n");
              printf("1. Yes\n");
              printf("2. No\n");
              scanf("%d", &choose);
              if (choose != 1 && choose != 2)
                printf("Input failed!\n");
            } while (choose != 1 && choose != 2);
            if (choose == 1) {
              printf("Server is working!\n");
              continue;
            } else if (choose == 2) {
              printf("Server is closed!\n");
              shutdown(sockfd, 2);
              return 0;
            }
          }
        } else {
          countTotalRecvData[j] = countTotalRecvData[j] + countRecvData;
          str_mess = "";
          str_mess.assign(message, countRecvData);
          boost::char_separator<char> split_str("|");
          int i = 1;
          boost::tokenizer<boost::char_separator<char> > tokens(str_mess,
              split_str);
          BOOST_FOREACH (const std::string& tmp_str, tokens) {
            std::cout << "TMP data" << tmp_str << "\n";
            if (i == 1) {
              if (tmp_str == "[SAVE_STATUS]") {
                check_action = 1;
              } else if (tmp_str == "[SAVE_OLD_CELLS]") {
                check_action = 2;
              } else if (tmp_str == "[GIVE_ME_STATUS]") {
                check_action = 3;
              } else if (tmp_str == "[GIVE_ME_OLD_CELLS]") {
                check_action = 4;
              }
            } else if (i == 2) {
              std::cout << "Check action = " << check_action << "\n";
              switch (check_action) {
              case 1:
                std::cout << "Status in server" << status << "\n";
                if (status.find(tmp_str) != std::string::npos) {
                  // Found
                  status = tmp_str;
                } else {
                  // Not found
                  status += tmp_str;
                }
                strcpy(message, "OK");
                break;
              case 2:
                std::cout << "OLD CELLS in server" << old_cells << "\n";
                if (tmp_str.find(old_cells) != std::string::npos) {
                  // Found
                  old_cells = tmp_str;
                } else {
                  // Not found
                  old_cells += tmp_str;
                }
                strcpy(message, "OK");
                break;
              case 3:
                std::cout << "Status in server" << status << "\n";
                if (status == ""){
                  strcpy(message, "no thing");
                }else{
                  strcpy(message, status.c_str());
                }
                break;
              case 4:
                std::cout << "OLD CELLS in server" << old_cells << "\n";
                if (old_cells == ""){
                  strcpy(message, "no thing");
                }else{
                  strcpy(message, old_cells.c_str());
                }
                break;
              }
            }
            i++;
          }
          countSendData = send(fd, message, strlen(message), 0);
          // std::cout << "Sent data";
          // message=malloc(MAX_SIZE);
          message = (char*) malloc(MAX_SIZE);
          strcpy(message, "");
          if (countSendData == -1) {
            printf("Send data failed!\n");
            shutdown(fd, 2);
            shutdown(sockfd, 2);
            return 1;
          } else {
            countTotalSendData[j] = countTotalSendData[j] + countSendData;
          }
        }
        if (--activity <= 0)
          break;
      }
    }
  }
}
