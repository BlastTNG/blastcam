#include <stdio.h>
#include <sys/types.h>  // socket
#include <sys/socket.h> // socket
#include <string.h>     // memset
#include <stdlib.h>     // sizeof
#include <netinet/in.h> // INADDR_ANY

#include <stdio.h>      // clean this up later on to remove unncessary libraries
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
#include <signal.h>
#include <ueye.h>

#include "camera.h"
#include "astrometry.h"
#include "lens_adapter.h"
#include "commands.h"

#define PORT  8000

// define data structures
#pragma pack(push)
#pragma pack(1)
// telemetry and camera settings structure
struct tcp_data {
    unsigned int curr_time;
  	float ra;
  	float dec;
  	float fr;
	float az;
	float el;
	float ir;
    float logodds;
    double latitude;
    double longitude;
    struct camera_params cam_settings; 
    struct blob_params current_blob_params;
};
// struct for commands user has sent
struct cmd_data {
    double logodds;
    double latitude;
    double longitude;
    double exposure;
    float focusPos; 
    float set_focus_inf;    // 0 = false, 1 = true (whether or not to set the focus to infinity)
    int aperture_steps;
    float set_max_aperture; // 0 = false, 1 = true (whether or not to maximize aperture)
    float blobParams[9];    //  blob-finding parameters
};
// struct for passing arguments to client thread
struct args {
    uintptr_t client_socket;
    socklen_t len;
    struct sockaddr_in userAddress;
    int fileDescriptor;
    char * user_IP;
};
#pragma pack(pop)

// initialize instances of necessary structures
struct cmd_data all_cmds = {0};
struct tcp_data all_data = {0};

void * camera_raw;

int command_lock = 0;       // if 1, then commanding is in use, if 0, then not
int shutting_down = 0;      // if 0, then camera is not closing, so keep solving astrometry

void *updateAstrometry() {
    // solve astrometry perpetually
    while (!shutting_down) {
        doCameraAndAstrometry();
    }
}

void *client_handler(void *arg) {
    // get the client socket information
    int sock = ((struct args*)arg)->client_socket;
    int length = ((struct args*)arg)->len;
    struct sockaddr_in addr = ((struct args*)arg)->userAddress;
    int fileDesc = ((struct args*)arg)->fileDescriptor;
    char * useraddr = ((struct args*)arg)->user_IP;
    int n;
    // send data to this client infinitely and receive potential commands if user sends them
    while (1) {
        n = recvfrom(sock, &all_cmds, sizeof(struct cmd_data), 0, (struct sockaddr *) &addr, &length); 
        if (n == -1) {
            printf("User %s did not send any commands. Send telemetry and camera settings back anyway.\n", useraddr);
        } else {
            while (command_lock) {
                usleep(100000);
            }
            command_lock = 1;
            printf("User %s sent commands. Executing...\n", useraddr);

            // print statements to evaluate commands when testing (comment out otherwise)
            // printf("Logodds command: %f\n", all_cmds.logodds);
            // printf("Latitude and longitude commands: %f and %f\n", all_cmds.latitude, all_cmds.longitude);
            // printf("Exposure command in commands.c: %f\n", all_cmds.exposure);
            // printf("Focus position command: %f\n", all_cmds.focusPos);
            // printf("Set focus to infinity bool command: %f\n", all_cmds.set_focus_inf);
            // printf("Aperture steps command: %i\n", all_cmds.aperture_steps);
            // printf("Set aperture max bool: %f\n", all_cmds.set_max_aperture);
            // printf("Blob parameters: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", all_cmds.blobParams[0], all_cmds.blobParams[1],
                    // all_cmds.blobParams[2], all_cmds.blobParams[3], all_cmds.blobParams[4], all_cmds.blobParams[5],
                    // all_cmds.blobParams[6], all_cmds.blobParams[7], all_cmds.blobParams[8]);

            // if user changed latitude and longitude (they are not at default location, DRL), then change that in struct
            // of astro params as well
            all_astro_params.logodds = all_cmds.logodds;
            all_astro_params.latitude = all_cmds.latitude;
            all_astro_params.longitude = all_cmds.longitude;
            // printf("all_cmds exposure is: %f and all_camera_params exposure rounded is: %f\n", all_cmds.exposure, ceil(all_camera_params.exposure_time));
            // if user has adjusted the exposure time, set camera exposure to their desired value
            if (ceil(all_cmds.exposure) != ceil(all_camera_params.exposure_time)) {
                printf("User %s wants to change exposure.\n", useraddr);
                // update value in camera params struct
                all_camera_params.exposure_time = all_cmds.exposure;
                all_camera_params.change_exposure_bool = 1;
            }

            // if the command to set the focus to infinity is true (1), ignore any other commands the user might have put in for focus
            all_camera_params.focus_inf = all_cmds.set_focus_inf;
            // if user wants to change the focus, change focus position value in camera params struct
            if (all_cmds.focusPos != -1) {
                all_camera_params.focus_position = all_cmds.focusPos; 
            } 
                
            // process the aperture
            all_camera_params.max_aperture = all_cmds.set_max_aperture;
            all_camera_params.aperture_steps = all_cmds.aperture_steps;

            // perform changes to camera lens settings
            handleFocusAndAperture(fileDesc);

            // process the blob parameters
            int new_spike_limit = all_cmds.blobParams[0];            // how agressive is the dynamic hot pixel finder.  Small is more agressive
            int new_dynamic_hot_pixels = all_cmds.blobParams[1];     // 0 = off, 1 = on
            int new_r_smooth = all_cmds.blobParams[2];               // image smooth filter radius [px]
            int new_high_pass_filter = all_cmds.blobParams[3];       // 0 = off, 1 = on
            int new_r_high_pass_filter = all_cmds.blobParams[4];     // image high pass filter radius [px]
            int new_centroid_search_border = all_cmds.blobParams[5]; // distance from image edge from which to start looking for stars [px]
            int new_filter_return_image = all_cmds.blobParams[6];    // 1 = true; 0 = false
            double new_n_sigma = all_cmds.blobParams[7];             // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
            int new_unique_star_spacing = all_cmds.blobParams[8];

            if (new_spike_limit >= 0) {
                all_blob_params.spike_limit = new_spike_limit;
            } 
            
            all_blob_params.dynamic_hot_pixels = new_dynamic_hot_pixels;
        
            if (new_r_smooth >= 0) {
                all_blob_params.r_smooth = new_r_smooth;
            }

            all_blob_params.high_pass_filter = new_high_pass_filter;
            
            if (new_r_high_pass_filter >= 0) {
                all_blob_params.r_high_pass_filter = new_r_high_pass_filter;
            } 
            
            if (new_centroid_search_border >= 0) {
                all_blob_params.centroid_search_border = new_centroid_search_border;
            } 

            all_blob_params.filter_return_image = new_filter_return_image;

            if (new_n_sigma >= 0) {
                all_blob_params.n_sigma = new_n_sigma;
            } 
            
            if (new_unique_star_spacing >= 0) {
                all_blob_params.unique_star_spacing = new_unique_star_spacing;
            } 
            command_lock = 0; // allow other clients to execute commands
        }
        // compile telemetry and transmit back to use
        all_data.ra = all_astro_params.ra;
        all_data.dec = all_astro_params.dec;
        all_data.fr = all_astro_params.fr;
        all_data.az = all_astro_params.az;  
        all_data.el = all_astro_params.alt;
        all_data.ir = all_astro_params.ir;
        all_data.logodds = all_astro_params.logodds;
        all_data.latitude = all_astro_params.latitude;
        all_data.longitude = all_astro_params.longitude;
        // transfer camera params struct to one in tcp_data struct to send back to user
        memcpy(&all_data.cam_settings, &all_camera_params, sizeof(all_camera_params));
        memcpy(&all_data.current_blob_params, &all_blob_params, sizeof(all_blob_params));

        printf("Size of all_data: %lu\n", sizeof(all_data));

        time_t seconds; 
        seconds = time(NULL); 
        all_data.curr_time = seconds;
        // printf("size of data %lu\n", sizeof(all_data));
        if (send(sock, &all_data, sizeof(struct tcp_data), MSG_NOSIGNAL) <= 0) {
            printf("Client dropped the connection.\n");
            break;
        } 
        // send image bytes back to user for display
        int n_sent_bytes = send(sock, camera_raw, CAMERA_WIDTH*CAMERA_HEIGHT, MSG_NOSIGNAL);
        printf("Telemetry and image bytes sent back to user.\n");      
    }
    // clean up socket when the connection is done
    close(sock);
}

int main() {
    // signal handling
    signal(SIGHUP, clean_up);
    signal(SIGINT, clean_up);
    signal(SIGTERM, clean_up);
    signal(SIGPIPE, SIG_IGN);

    int sockfd;    // to create socket
    int newsockfd; // to accept connection

    struct sockaddr_in serverAddress; // server receive on this address
    struct sockaddr_in clientAddress; // server sends to client on this address 
    int n;
    int len; 
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

    // make socket non-blocking (times out after a certain time)
    struct timeval read_timeout;
    read_timeout.tv_sec = 3;
    read_timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    // initialize the camera
    init_camera();

    // initialize the lens adapter
    int fileDescriptor = init_lensAdapter("/dev/ttyLens");

    pthread_t thread_id;
    pthread_t astro_thread_id;
    // create a separate thread from all the client thread(s) that just solves astrometry 
    if (pthread_create(&astro_thread_id, NULL, updateAstrometry, NULL) < 0) {
        perror("Could not create thread for astrometry function.\n");
    }

    // loop forever, accepting new clients
    while (newsockfd = accept(sockfd, (struct sockaddr*)&clientAddress, &clientAddressLength)) {
        // parent process waiting to accept a new connection
        printf("\n***** Server waiting for new client connection: *****\n");
        // store length of client socket that has connected (if any)
        clientAddressLength = sizeof(clientAddress);
        if (newsockfd == -1) {
            printf("New client did not connect.\n");
        } else {
            // user did connect
            printf("Connected to client: %s\n", inet_ntoa(clientAddress.sin_addr)); 
            struct args *socket_args = (struct args *)malloc(sizeof(struct args));
            socket_args->client_socket = newsockfd;
            socket_args->len = clientAddressLength;
            socket_args->userAddress = clientAddress;
            socket_args->fileDescriptor = fileDescriptor;
            socket_args->user_IP = inet_ntoa(clientAddress.sin_addr);
            // create new thread for this new client
            if (pthread_create(&thread_id, NULL, client_handler, (void*) socket_args) < 0) {
                perror("Could not create thread for client.\n");
            }
        }
    }
}