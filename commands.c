// import necessary libraries
#include <sys/types.h>      // socket
#include <sys/socket.h>     // socket
#include <string.h>         // memset
#include <stdlib.h>         // sizeof
#include <netinet/in.h>     // INADDR_ANY
#include <stdio.h>          
#include <arpa/inet.h>     
#include <netdb.h>         
#include <errno.h>          // interpreting errors
#include <time.h>           // time functions
#include <math.h>           // math functions
#include <pthread.h>        // client handler thread
#include <signal.h>         // signal handling (ctrl+c exception)
#include <ueye.h>           // uEye camera operation

// include our header files
#include "camera.h"
#include "astrometry.h"
#include "lens_adapter.h"
#include "commands.h"

#pragma pack(push)
#pragma pack(1)
// telemetry and camera settings structure
struct tcp_data {
    struct astro_params astrom;
    struct camera_params cam_settings; 
    struct blob_params current_blob_params;
};
// struct for commands user has sent
struct cmd_data {
    double logodds;         // parameter for amount of false positives that make it through astrometry
    double latitude;        // user's latitude (radians)
    double longitude;       // user's longitude (degrees)
    double hm;              // user's height above ellipsoid from GPS (WGS84)
    double exposure;        // user's desired camera exposure (msec)
    float focusPos;         // user's desired focus position (counts)
    float set_focus_inf;    // 0 = false, 1 = true (whether or not to set the focus to infinity)
    int aperture_steps;     // number of shifts (+/-) needed to reach desired aperture
    float set_max_aperture; // 0 = false, 1 = true (whether or not to maximize aperture)
    int make_hp;            // flag to make a new static hp mask (0 = don't make, 20 = re-make)
    int use_hp;             // flag to use current static hp mask (0 = don't use, 1 = use)
    float blobParams[9];    // rest of blob-finding parameters (see definition of blob_params struct for explanations)
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

// constants
#define PORT  8000

// initialize instances of these structures
struct cmd_data all_cmds = {0};
struct tcp_data all_data = {0};
// initialize pointer for storing address to current camera image bytes (accessible by camera.c)
void * camera_raw;
int command_lock = 0;       // if 1, then commanding is in use, if 0, then not
int shutting_down = 0;      // if 0, then camera is not closing, so keep solving astrometry

// helper function for testing reception of user commands
void verifyUserCommands() {
    printf("\n**** USER COMMANDS ****\n");
    printf("Logodds command: %f\n", all_cmds.logodds);
    printf("Latitude and longitude commands: %f and %f\n", all_cmds.latitude, all_cmds.longitude);
    printf("Exposure command in commands.c: %f\n", all_cmds.exposure);
    printf("Focus position command: %f\n", all_cmds.focusPos);
    printf("Set focus to infinity bool command: %f\n", all_cmds.set_focus_inf);
    printf("Aperture steps command: %i\n", all_cmds.aperture_steps);
    printf("Set aperture max bool: %f\n", all_cmds.set_max_aperture);
    printf("Make static hp mask: %i and use static hp mask: %i\n", all_cmds.make_hp, all_cmds.use_hp);
    printf("Blob parameters: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", all_cmds.blobParams[0], all_cmds.blobParams[1],
            all_cmds.blobParams[2], all_cmds.blobParams[3], all_cmds.blobParams[4], all_cmds.blobParams[5],
            all_cmds.blobParams[6], all_cmds.blobParams[7], all_cmds.blobParams[8]);
    printf("***********************\n");
}

// helper function for testing transmission of telemetry
void verifyTelemetryData() {
    printf("\n**** TELEMETRY ****\n");
    printf("Current rawtime: %f\n", all_data.astrom.rawtime);
    printf("RA: %.15f\n", all_data.astrom.ra);
    printf("DEC: %.15f\n", all_data.astrom.dec);
    printf("FR: %.15f\n", all_data.astrom.fr);
    printf("AZ: %.15f\n", all_data.astrom.az);
    printf("ALT: %.15f\n", all_data.astrom.alt);
    printf("IR: %.15f\n", all_data.astrom.ir);
    printf("PS: %f\n", all_data.astrom.ps);
    printf("Logodds: %f\n", all_data.astrom.logodds);
    printf("Latitude: %.15f\n", all_data.astrom.latitude);
    printf("Longitude: %.15f\n", all_data.astrom.longitude);
    printf("Height: %f\n", all_data.astrom.hm);
    printf("***********************\n");
}

// function devoted to taking pictures and solving astrometry while camera is not in a state of
// shutting down
void * updateAstrometry() {
    // solve astrometry perpetually
    while (!shutting_down) {
        doCameraAndAstrometry();
    }
}

// accept newly connected clients and send telemetry/receive their commands
void * client_handler(void * arg) {
    // get the client socket information
    int sock = ((struct args *) arg)->client_socket;
    int length = ((struct args *) arg)->len;
    struct sockaddr_in addr = ((struct args *) arg)->userAddress;
    int fileDesc = ((struct args *) arg)->fileDescriptor;
    char * useraddr = ((struct args *) arg)->user_IP;
    int n;
    // send data to this client perpetually and receive potential commands if user sends them
    while (1) {
        n = recvfrom(sock, &all_cmds, sizeof(struct cmd_data), 0, (struct sockaddr *) &addr, &length); 
        if (n == -1) {
            printf("User %s did not send any commands. Send telemetry and camera settings back anyway.\n", useraddr);
        } else {
            // if another client is sending commands at the same time, wait until they are done
            while (command_lock) {
                usleep(100000);
            }
            // now it's this client's turn to execute commands (lock)
            command_lock = 1;
            printf("User %s sent commands. Executing...\n", useraddr);
            verifyUserCommands();

            // update astro params struct with user commands (cmd struct values)
            all_astro_params.logodds = all_cmds.logodds;
            all_astro_params.latitude = all_cmds.latitude;
            all_astro_params.longitude = all_cmds.longitude;
            all_astro_params.hm = all_cmds.hm;
            // if user has adjusted the exposure time, set camera exposure to their desired value
            if (ceil(all_cmds.exposure) != ceil(all_camera_params.exposure_time)) {
                // update value in camera params struct as well
                all_camera_params.exposure_time = all_cmds.exposure;
                all_camera_params.change_exposure_bool = 1;
            }

            // if the command to set the focus to infinity is true (1), ignore any other commands the user might 
            // have put in for focus accidentally
            all_camera_params.focus_inf = all_cmds.set_focus_inf;
            // if user wants to change the focus, change focus position value in camera params struct
            if (all_cmds.focusPos != -1) {
                all_camera_params.focus_position = all_cmds.focusPos; 
            } 
                
            // update camera params struct with user commands
            all_camera_params.max_aperture = all_cmds.set_max_aperture;
            all_camera_params.aperture_steps = all_cmds.aperture_steps;

            // perform changes to camera lens settings in lens_adapter.c
            handleFocusAndAperture(fileDesc);

            // process the blob parameters
            all_blob_params.make_static_hp_mask = all_cmds.make_hp;               // re-make static hot pixel map with new image (0 = off, 20 = re-make)
            all_blob_params.use_static_hp_mask = all_cmds.use_hp;                 // use the static hot pixel map (0 = off, 1 = on)

            if (all_cmds.blobParams[0] >= 0) {
                all_blob_params.spike_limit = all_cmds.blobParams[0];             // how agressive is the dynamic hot pixel finder.  Smaller is more agressive
            } 
            
            all_blob_params.dynamic_hot_pixels = all_cmds.blobParams[1];          // turn dynamic hot pixel finder 0 = off, 1 = on
        
            if (all_cmds.blobParams[2] >= 0) {
                all_blob_params.r_smooth = all_cmds.blobParams[2];                // image smooth filter radius [px]
            }

            all_blob_params.high_pass_filter = all_cmds.blobParams[3];            // turn high pass filter 0 = off, 1 = on
            
            if (all_cmds.blobParams[4] >= 0) {
                all_blob_params.r_high_pass_filter = all_cmds.blobParams[4];      // image high pass filter radius [px]
            } 
            
            if (all_cmds.blobParams[5] >= 0) {
                all_blob_params.centroid_search_border = all_cmds.blobParams[5];  // distance from image edge from which to start looking for stars [px]
            } 

            all_blob_params.filter_return_image = all_cmds.blobParams[6];         // filter return image 1 = true; 0 = false

            if (all_cmds.blobParams[7] >= 0) {
                all_blob_params.n_sigma = all_cmds.blobParams[7];                 // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
            } 
            
            if (all_cmds.blobParams[8] >= 0) {
                all_blob_params.unique_star_spacing = all_cmds.blobParams[8];     // minimum pixel spacing between unique stars
            } 

            // allow other clients to execute commands (unlock)
            command_lock = 0; 
        }
        // compile telemetry and transmit back to use
        memcpy(&all_data.astrom, &all_astro_params, sizeof(all_astro_params));
        memcpy(&all_data.cam_settings, &all_camera_params, sizeof(all_camera_params));
        memcpy(&all_data.current_blob_params, &all_blob_params, sizeof(all_blob_params));

        printf("Size of all_data: %lu bytes\n", sizeof(all_data));
        verifyTelemetryData();

        // printf("size of data %lu\n", sizeof(all_data));
        if (send(sock, &all_data, sizeof(struct tcp_data), MSG_NOSIGNAL) <= 0) {
            printf("Client dropped the connection.\n");
            break;
        } 
        // send image bytes back to user for image display
        int n_sent_bytes = send(sock, camera_raw, CAMERA_WIDTH*CAMERA_HEIGHT, MSG_NOSIGNAL);
        printf("Telemetry and image bytes sent back to user.\n");      
    }
    // clean up socket when the connection is done
    close(sock);
}

// Driver function for StarCamera operation
int main() {
    // signal handling (e.g. ctrl+c exception)
    signal(SIGHUP, clean_up);
    signal(SIGINT, clean_up);
    signal(SIGTERM, clean_up);
    signal(SIGPIPE, SIG_IGN);

    int sockfd;                       // to create socket
    int newsockfd;                    // to accept connection
    struct sockaddr_in serverAddress; // server receives on this address
    struct sockaddr_in clientAddress; // server sends to client on this address 
    int n;
    int len; 
    int clientAddressLength;
    int pid;                          // new pthread ID

    printf("Size of all_data: %lu bytes\n", sizeof(all_data));
    printf("--------------------------------\n");

    // create socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    // initialize the socket addresses
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(PORT); 
    // bind the socket with the server address and port
    bind(sockfd, (struct sockaddr *) &serverAddress, sizeof(serverAddress));  
    // listen for connections from client (maximum is 5)
    listen(sockfd, 5);  

    // make socket non-blocking (times out after a certain time, 3 sec)
    struct timeval read_timeout;
    read_timeout.tv_sec = 2.5;
    read_timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    // initialize the camera (inputs: 1 = load previous observing data for testing, 0 = take new data)
    init_camera(0);

    // initialize the lens adapter
    int fileDescriptor = init_lensAdapter("/dev/ttyLens");

    pthread_t thread_id;
    pthread_t astro_thread_id;
    // create a thread separate from all the client thread(s) to just solve astrometry 
    if (pthread_create(&astro_thread_id, NULL, updateAstrometry, NULL) < 0) {
        perror("Could not create Astrometry thread.\n");
    }

    // loop forever, accepting new clients
    while (newsockfd = accept(sockfd, (struct sockaddr*)&clientAddress, &clientAddressLength)) {
        // parent process waiting to accept a new connection
        printf("\n******************************* Server waiting for new client connection: *******************************\n");
        // store length of client socket that has connected (if any)
        clientAddressLength = sizeof(clientAddress);
        if (newsockfd == -1) {
            printf("New client did not connect.\n");
        } else {
            // user did connect, so process their information for their respective client thread
            printf("Connected to client: %s\n", inet_ntoa(clientAddress.sin_addr)); 
            struct args * socket_args = (struct args *)malloc(sizeof(struct args));
            socket_args->client_socket = newsockfd;
            socket_args->len = clientAddressLength;
            socket_args->userAddress = clientAddress;
            socket_args->fileDescriptor = fileDescriptor;
            socket_args->user_IP = inet_ntoa(clientAddress.sin_addr);
            // create new thread for this new client
            if (pthread_create(&thread_id, NULL, client_handler, (void *) socket_args) < 0) {
                perror("Could not create thread for new client.\n");
            }
        }
    }
}