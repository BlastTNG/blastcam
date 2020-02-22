#include<stdio.h>
#include<sys/types.h>  // socket
#include<sys/socket.h> // socket
#include<string.h>     // memset
#include<stdlib.h>     // sizeof
#include<netinet/in.h> // INADDR_ANY

#include <stdio.h>     // clean this up later on to remove unncessary libraries
#include <stdlib.h>
#include <string.h>
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <netdb.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <pthread.h>

#include "telemetry.h"
#include "camera.h"
#include "astrometry.h"
#include "lens_adapter.h"

#define PORT  8000
#define MAXSZ 1024

// define data structure
#pragma pack(push)
#pragma pack(1)
struct udp_data {
    unsigned int curr_time;
  	float ra;
  	float dec;
  	float fr;
	float az;
	float el;
	float ir;
};
struct cmd_data {
	// unsigned int id; // char cmdname[24];
    float focusPos; // new focus position
    // float focusPosRel; // focus position to go to (absolute change)
    float set_focus_inf; // 0 = false, 1 = true (whether or not to set the focus to infinity)
    int aperture_steps; // aperture value to go to (absolute change)
    // float apertureRel; // aperture value to shift by (relative change)
    float set_max_aperture; // 0 = false, 1 = true (whether or not to maximize aperture)
    float blobParams[9]; //  blob-finding parameters
};
#pragma pack(pop)

struct blob_params all_blob_params;

int main() {
    int sockfd;    // to create socket
    int newsockfd; // to accept connection
    // initialize instance of udp data structure to 0
    struct cmd_data all_cmds = {0};

    struct sockaddr_in serverAddress; // server receive on this address
    struct sockaddr_in clientAddress; // server sends to client on this address 
    int n;
    int len; 
    char msg[MAXSZ];
    int clientAddressLength;
    int pid;    

    // create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    // initialize the socket addresses
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(PORT); 
    // bind the socket with the server address and port
    bind(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress));  
    // listen for connection from client
    listen(sockfd, 5);  

    // make socket non-blocking
    struct timeval read_timeout;
    read_timeout.tv_sec = 5;
    read_timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    // initialize the camera
    //init_camera();

    // initialize the lens adapter
    //int fileDescriptor = init_lensAdapter("/dev/ttyLens");

    while (1) {
     // parent process waiting to accept a new connection
     printf("\n***** Server waiting for new client connection: *****\n");

     clientAddressLength = sizeof(clientAddress);
     newsockfd = accept(sockfd, (struct sockaddr*)&clientAddress, &clientAddressLength);
     // make socket non-blocking
     // setsockopt(newsockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));
     printf("Connected to client: %s\n",inet_ntoa(clientAddress.sin_addr)); 

     // child process is created for serving each new clients
     pid = fork();
     if (pid == 0) { // child process rec and send
      // receive from client
      while (1) {
        printf("Waiting for commands.\n");
        n = recvfrom(newsockfd, &all_cmds, sizeof(struct cmd_data), 0, (struct sockaddr *) &serverAddress, &len); 
        // n = recv(newsockfd, msg, MAXSZ, 0);
        if (n == -1) {
            printf("User sent no commands in time. Default parameters kept for camera.\n");
            // send default blob parameters to camera
        } else {
            // if it did receive user commands, execute them and transmit the telemetry back to the user
            printf("User message received!\n"); 
            // printf("focus pos: %f\n", all_cmds.focusPos);
            // printf("set focus to inf bool: %f\n", all_cmds.set_focus_inf);
            // printf("aperture steps: %i\n", all_cmds.aperture_steps);
            // printf("set max aperture bool: %f\n", all_cmds.set_max_aperture);
            // printf("blob params 1: %f\n", all_cmds.blobParams[0]);
            // printf("blob params 2: %f\n", all_cmds.blobParams[1]);
            // printf("blob params 3: %f\n", all_cmds.blobParams[2]);
            // printf("blob params 4: %f\n", all_cmds.blobParams[3]);
            // printf("blob params 5: %f\n", all_cmds.blobParams[4]);
            // printf("blob params 6: %f\n", all_cmds.blobParams[5]);
            // printf("blob params 7: %f\n", all_cmds.blobParams[6]);
            // printf("blob params 8: %f\n", all_cmds.blobParams[7]);
            // printf("blob params 9: %f\n", all_cmds.blobParams[8]);
        }
        sleep(3);
      } // close interior while

     exit(0);
     } else {
       close(newsockfd); // sock is closed BY PARENT
     }
    } // close exterior while

    return 0;
}