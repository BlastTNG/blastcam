/* This script should run on the StarCamera computer on an infinite loop (always waiting
   for commands). It receives commands from the user computer via a Python user script and UDP
   and then communicates with telemetry.c to collect the desired data.).
   
   Server-side implementation of UDP server-client model. */

#include <stdio.h> // clean this up later on to remove unncessary libraries
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

#include "commands.h"
#include "telemetry.h"
#include "camera.h"
#include "astrometry.h"
#include "lens_adapter.h"

// hard-coded, agreed-upon port between user computer and StarCamera computer
#define PORT	         62190     
#define BROADCASTPORT	 41214  
#define MAXLINE           1024 

// define data structure
#pragma pack(push)
#pragma pack(1)
// struct udp_data {
//     unsigned int curr_time;
//   	float ra;
//   	float dec;
//   	float fr;
// 	float az;
// 	float el;
// 	float ir;
//     int imageHeight;
//     int imageWidth;
// };
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

void printCharArr(unsigned char * arr) {
    int loop;
    for (loop = 0; loop < 500; loop++) {
        printf("%d\n", arr[loop]);
    }
}

struct udp_data all_data = {
  .curr_time = 0,
  .ra = 0,
  .dec = 0,
  .fr = 0,
  .az = 0,
  .el = 0,
  .ir = 0,
  //.imageHeight = 0,
  //.imageWidth = 0,
  //.image = {0}
};

// Driver code
int main() {
    int sockfd; 
    int sockfd_camera;
	int buffer[MAXLINE]; 
	struct sockaddr_in servaddr, useraddr, servaddr_cam; 
	// initialize instance of udp data structure to 0
    struct cmd_data all_cmds = {0};
	
	// Create a UDP socket to communicate with listening.py
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
		perror("Socket creation failed."); 
		exit(EXIT_FAILURE); 
	} 
    // set sockfd socket option to waiting ten seconds for user commands before returning
    // in recvfrom (below)
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 100;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    // give the socket broadcast permission
    //int broadcast = 1;
    //if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    //    printf("Unable to set broadcast permissions for the socket %d: %s\n", sockfd, strerror(errno));
    //}

	memset(&servaddr, 0, sizeof(servaddr)); 
	memset(&useraddr, 0, sizeof(useraddr)); 
    memset(&servaddr_cam, 0, sizeof(servaddr_cam)); 

	// Filling server information 
	servaddr.sin_family = AF_INET;           // IPv4 
	servaddr.sin_addr.s_addr = INADDR_ANY;   // receive from any user address
	servaddr.sin_port = htons(PORT); 

    // set up useraddr as to broadcast mode
    struct hostent * target = gethostbyname("10.102.185.86"); // getting the specific address // Aguirre computer IP: 128.91.43.54, Bob computer IP: 10.102.185.86
    memcpy(&useraddr.sin_addr.s_addr, target->h_addr, target->h_length); // setting the specific address
    useraddr.sin_family = AF_INET;        
    useraddr.sin_port = htons(BROADCASTPORT); // to be determined
    // useraddr.sin_addr.s_addr  = INADDR_BROADCAST; // this is equiv to 255.255.255.255, the broadcast address
    // useraddr.sin_addr.s_addr = INADDR_ANY;
	
	// Bind the socket with the server address
	if (bind(sockfd, (const struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) { 
		perror("Bind failed."); 
		exit(EXIT_FAILURE); 
	} 

    // initialize the camera
    init_camera();

    // initialize the lens adapter
    int fileDescriptor = init_lensAdapter("/dev/ttyLens");
	
	int len, n; 
    // create infinite loop so commands.c is always waiting for user commands
	while (1) {
        // make recvfrom non-blocking, so that if no user commands are received after a certain time, the 
        // rest of the code executes anyway
        n = recvfrom(sockfd, &all_cmds, sizeof(struct cmd_data), 0, (struct sockaddr *) &servaddr, &len); 
        if (n == -1) {
            printf("User sent no commands in time. Default parameters kept for camera.\n");
            // send default blob parameters to camera
        } else {
            // if it did receive user commands, execute them and transmit the telemetry back to the user
            printf("User message received!\n"); 

            // printf("focus pos: %f\n", all_cmds.focusPos);
            // printf("set focus to inf bool: %f\n", all_cmds.set_focus_inf);
            // printf("aperture steps: %f\n", all_cmds.aperture_steps);
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

            // but don't execute if this is just the initialization 'command' meant to establish connection between user computer and StarCamera
            if (all_cmds.focusPos == -10000) { 
                printf("Message was initialization connection message, so don't execute these particular 'commands'.\n");
            } else {
                printf("User sent actual commands. Executing...\n");
                // process the focus
                // if the command to set the focus to infinity is true (1), ignore any other commands the user might have put in for focus
                all_camera_params.focus_inf = all_cmds.set_focus_inf;
                printf("%f\n", all_camera_params.prev_focus_pos);
                all_camera_params.prev_focus_pos = all_camera_params.focus_position;
                printf("%f\n", all_camera_params.prev_focus_pos);
                if (all_cmds.focusPos != -1) {
                    printf("Received command to move the focus position, not shifting: %f\n", all_cmds.focusPos);
                    all_camera_params.focus_position = all_cmds.focusPos; // store the new focus position 
                    // all_camera_params.focus_shift = 0; // nothing to shift by, we are going to an absolute position
                } 
                    
                // process the aperture
                // printf("max aperture bool was: %d\n", all_camera_params.max_aperture);
                all_camera_params.max_aperture = all_cmds.set_max_aperture;
                //printf("%i\n", all_cmds.aperture_steps);
                all_camera_params.aperture_steps = all_cmds.aperture_steps;
                // printf("now max aperture bool is: %d\n", all_camera_params.max_aperture);
                //if (all_cmds.aperture_steps != 0) {
                    //printf("Received command to change aperture. Will execute...\n");
                    // all_camera_params.prev_aperture = all_camera_params.aperture;
                    //all_camera_params.aperture_steps = all_cmds.aperture_steps;
                    // all_camera_params.aperture_shift = 0;
                //} 

                // perform changes
                handleFocusAndAperture(fileDescriptor);

                // process the blob parameters
                // printArr(all_cmds.blobParams);
                int new_spike_limit = all_cmds.blobParams[0]; // how agressive is the dynamic hot pixel finder.  Small is more agressive
                int new_dynamic_hot_pixels = all_cmds.blobParams[1]; // 0 == off, 1 == on
                int new_r_smooth = all_cmds.blobParams[2];   // image smooth filter radius [px]
                int new_high_pass_filter = all_cmds.blobParams[3]; // 0 == off, 1 == on
                int new_r_high_pass_filter = all_cmds.blobParams[4]; // image high pass filter radius [px]
                int new_centroid_search_border = all_cmds.blobParams[5]; // distance from image edge from which to start looking for stars [px]
                int new_filter_return_image = all_cmds.blobParams[6]; // 1 == true; 0 = false
                double new_n_sigma = all_cmds.blobParams[7]; // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
                int new_unique_star_spacing = all_cmds.blobParams[8];

                if (new_spike_limit >= 0) {
                    printf("previous spike limit: %d\n", all_blob_params.spike_limit);
                    all_blob_params.spike_limit = new_spike_limit;
                    printf("now spike limit is: %d\n", all_blob_params.spike_limit);
                } 
                
                printf("previous dynamic hot pixels bool: %d\n", all_blob_params.dynamic_hot_pixels);
                all_blob_params.dynamic_hot_pixels = new_dynamic_hot_pixels;
                printf("now dynamic hot pixels bool is: %d\n", all_blob_params.dynamic_hot_pixels);
            
                if (new_r_smooth >= 0) {
                    printf("previous image smooth filter radius: %d\n", all_blob_params.r_smooth);
                    all_blob_params.r_smooth = new_r_smooth;
                    printf("now image smooth filter radius is: %d\n", all_blob_params.r_smooth);
                }

                printf("previous high pass filter bool bool: %d\n", all_blob_params.high_pass_filter);
                all_blob_params.high_pass_filter = new_high_pass_filter;
                printf("now high pass filter bool is: %d\n", all_blob_params.high_pass_filter);
                
                if (new_r_high_pass_filter >= 0) {
                    printf("previous high pass filter radius: %d\n", all_blob_params.r_high_pass_filter);
                    all_blob_params.r_high_pass_filter = new_r_high_pass_filter;
                    printf("now high pass filter radius is: %d\n", all_blob_params.r_high_pass_filter);
                } 
                
                if (new_centroid_search_border >= 0) {
                    printf("previous centroid search border: %d\n", all_blob_params.centroid_search_border);
                    all_blob_params.centroid_search_border = new_centroid_search_border;
                    printf("now centroid search border is: %d\n", all_blob_params.centroid_search_border);
                } 
                
                printf("previous filter return image bool: %d\n", all_blob_params.filter_return_image);
                all_blob_params.filter_return_image = new_filter_return_image;
                printf("now filter return image bool is: %d\n", all_blob_params.filter_return_image);
                
                if (new_n_sigma >= 0) {
                    printf("previous n sigma: %f\n", all_blob_params.n_sigma);
                    all_blob_params.n_sigma = new_n_sigma;
                    printf("now n sigma is: %f\n", all_blob_params.n_sigma);
                } 
                
                if (new_unique_star_spacing >= 0) {
                    printf("previous unique star spacing: %d\n", all_blob_params.unique_star_spacing);
                    all_blob_params.unique_star_spacing = new_unique_star_spacing;
                    printf("now unique star spacing is: %d\n", all_blob_params.unique_star_spacing);
                } 
            }
        }
       // printf("Image height before struct access by camera.c: %i\n", all_data.imageHeight);
        // camera stuff
        doCameraAndAstrometry();
        //printf("Image height after struct access by camera.c: %i\n", all_data.imageHeight);

        // TELEMETRY COMPILATION AND TRANSMISSION
        all_data.ra = get_ra();
        all_data.dec = get_dec();
        all_data.fr = get_fr();
        all_data.az = get_az();                
        all_data.el = get_elev();
        all_data.ir = get_ir();
        // set time field of telemetry data structure to current universal time 
        time_t seconds; 
        seconds = time(NULL); 
        all_data.curr_time = seconds;
	    // give the data back to the user via UDP broadcast
        len  = sizeof(struct sockaddr);
	    sendto(sockfd, &all_data, sizeof(struct udp_data), MSG_NOSIGNAL, (const struct sockaddr *) &useraddr, len);
	    printf("Telemetry sent back to user.\n");
    }
	return 0; // return dummy value (0)
}