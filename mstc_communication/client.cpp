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
#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

int main(int argc, char* argv[]) {
  struct sockaddr_in server;
  int sockfd;
  int c = 0;

  char *sendMes = (char *) malloc (MAX_SIZE);
  char *recvMes = (char *) malloc (MAX_SIZE);
  strcpy(sendMes,"");
  strcpy(recvMes,"");
  int countTotalRecvData=0, countTotalSendData=0, countRecvData=0, countSendData=0;
  printf("Creat a socket, please wait...\n");
  if((sockfd=socket(AF_INET,SOCK_STREAM,0))==-1){
    printf("Socket retrieve failed!\n");
    return 1;
  }else printf("Socket retrieve success!...\n");
  memset(&server,'0',sizeof(server));
  server.sin_family=AF_INET;
  server.sin_addr.s_addr=inet_addr(argv[1]);
  server.sin_port=htons(5678);
  if(connect(sockfd,(struct sockaddr *)&server, sizeof(server))==-1){
    printf("Connect failed!\n");
    return 1;
  }else printf("Connected. Ready to communicate with server!\n");
  while(1){
    c = 0;
    sendMes = (char *) malloc (MAX_SIZE);
    recvMes = (char *) malloc (MAX_SIZE);
    strcpy(sendMes,"");
    strcpy(recvMes,"");
    
    //scanf("%c",&blank);

    // gets(sendMes);
    
    //scanf("%s",sendMes);
    // printf("Your write: %s\n",sendMes);
    while(!kbhit()){
      // printf("Press a key: ");
    }
    switch(c = getchar()) {
        case 'w':
            // sendMes << "Up" << endl;//key up
            strcpy(sendMes,"Up");
            break;
        case 's':
            // sendMes << "Down" << endl;   // key down
            strcpy(sendMes,"Down");
            break;
        case 'a':
            // sendMes << "Left" << endl;  // key left
            strcpy(sendMes,"Left");
            break;
        case 'd':
            // sendMes << "Right" << endl;  // key right
            strcpy(sendMes,"Right");
            break;
        case ' ':
            strcpy(sendMes,"Space");
            break;
        case 'q':
        case 'Q':
            strcpy(sendMes,"Quit");
            break;
        case 'r':
            strcpy(sendMes,"Disable motors");
            break;
        case 'e':
            strcpy(sendMes,"Enable motors");
            break;
        default:
            // sendMes << "null" << endl;  // not arrow
            strcpy(sendMes,"null");
            break;
        }
    
    countSendData=send(sockfd,sendMes,strlen(sendMes),0);
    if(countSendData==-1){
      printf("Send data failed!\n");
      return 1;
    }else if((strcmp(sendMes,"Quit")==0)){
      sendMes = (char *) malloc (MAX_SIZE);
      strcpy(sendMes,"");
      printf("Disconnected!\n");
      printf("Total size of data sent: %d\n",countTotalSendData);
      printf("Total size of data received: %d\n",countTotalRecvData);
      break;
    }else{
      countTotalSendData=countTotalSendData+countSendData;
      countRecvData=recv(sockfd,recvMes,2000,0);
      if(countRecvData==-1){
	printf("Receive data failed!\n");
	return 1;
      }else{
	countTotalRecvData=countTotalRecvData+countRecvData;
	printf("Server reply: %s\n",recvMes);
	recvMes = (char *) malloc (MAX_SIZE);
	strcpy(recvMes,"");
      }
    }
  }
  shutdown(sockfd,2);
  return 0;
}
