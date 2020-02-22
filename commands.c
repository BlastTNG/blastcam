#include <stdio.h>
#include <sys/types.h>  // socket
#include <sys/socket.h> // socket
#include <string.h>     // memset
#include <stdlib.h>     // sizeof
#include <netinet/in.h> // INADDR_ANY

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
#include <signal.h>
#include <ueye.h>

#include "camera.h"
#include "astrometry.h"
#include "lens_adapter.h"
#include "commands.h"

#define PORT  8000
#define MAXSZ 1024

// define data structures
#pragma pack(push)
#pragma pack(1)
// telemetry structure (udp refers to old method of udp broadcast)
struct tcp_data {
    unsigned int curr_time;
  	float ra;
  	float dec;
  	float fr;
	float az;
	float el;
	float ir;
    // struct for camera settings to send back to user
    struct camera_params cam_settings; 
    // information fields for blob parameters
    // int spike_limit; // how agressive is the dynamic hot pixel finder.  Small is more agressive
    // int dynamic_hot_pixels; // 0 == off, 1 == on
    // int r_smooth;   // image smooth filter radius [px]
    // int high_pass_filter; // 0 == off, 1 == on
    // int r_high_pass_filter; // image high pass filter radius [px]
    // int centroid_search_border; // distance from image edge from which to start looking for stars [px]
    // int filter_return_image; // 1 == true; 0 = false
    // float n_sigma; // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
    // int unique_star_spacing;
    struct blob_params current_blob_params;
};
// struct for commands user has sent
struct cmd_data {
    double latitude;
    double longitude;
	// unsigned int id; // char cmdname[24];
    float focusPos; // new focus position
    // float focusPosRel; // focus position to go to (absolute change)
    float set_focus_inf; // 0 = false, 1 = true (whether or not to set the focus to infinity)
    int aperture_steps; // aperture value to go to (absolute change)
    // float apertureRel; // aperture value to shift by (relative change)
    float set_max_aperture; // 0 = false, 1 = true (whether or not to maximize aperture)
    float blobParams[9]; //  blob-finding parameters
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
// struct blob_params all_blob_params;
struct cmd_data all_cmds = {0};
struct tcp_data all_data = {0};

void * camera_raw;

// IMPORTANT QUESTION: WILL THIS UPDATE WITH EACH ASTROMETRY SOLVE?
//char * image_to_send = "latest_saved_image.bmp"; // could add to all_data struct as field to send
int command_lock = 0; // if 1, then commanding is in use, if 0, then not

void *updateAstrometry() {
    // solve astrometry perpetually
    while (1) {
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
    // send back current camera parameters for display on user GUI as well
    //sendto(sock, &all_camera_params, sizeof(struct camera_params), MSG_NOSIGNAL, (const struct sockaddr *) &addr, length);
    printf("Current camera settings sent back to user.\n");
    // send data to this client infinitely and receive potential commands if user sends them
    while (1) {
        n = recvfrom(sock, &all_cmds, sizeof(struct cmd_data), 0, (struct sockaddr *) &addr, &length); 
        if (n == -1) {
            printf("User did not send any commands. Send telemetry back anyway.\n");
        } else {
            while (command_lock) {
                usleep(100000);
            }
            command_lock = 1;
            printf("User sent commands. Executing...\n");
            // if user changed latitude and longitude (they are not at default location, DRL), then change that in struct
            // of astro params as well
            all_astro_params.latitude = all_cmds.latitude;
            printf("%f and %f\n", all_cmds.latitude, all_cmds.longitude);
            all_astro_params.longitude = all_cmds.longitude;
            // process the focus
            printf("%f\n", all_cmds.focusPos);
            // if the command to set the focus to infinity is true (1), ignore any other commands the user might have put in for focus
            all_camera_params.focus_inf = all_cmds.set_focus_inf;
            printf("%f\n", all_cmds.set_focus_inf);
            printf("%i\n", all_camera_params.focus_inf);

            //printf("%f\n", all_camera_params.prev_focus_pos);
            if (all_cmds.focusPos != -1) {
                //printf("Received command to move the focus position, not shifting: %f\n", all_cmds.focusPos);
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
            handleFocusAndAperture(fileDesc);

            // update focus fields
            //all_camera_params.prev_focus_pos = all_camera_params.focus_position;

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
                //printf("previous spike limit: %d\n", all_blob_params.spike_limit);
                all_blob_params.spike_limit = new_spike_limit;
                //printf("now spike limit is: %d\n", all_blob_params.spike_limit);
            } 
            
            //printf("previous dynamic hot pixels bool: %d\n", all_blob_params.dynamic_hot_pixels);
            all_blob_params.dynamic_hot_pixels = new_dynamic_hot_pixels;
            //printf("now dynamic hot pixels bool is: %d\n", all_blob_params.dynamic_hot_pixels);
        
            if (new_r_smooth >= 0) {
                //printf("previous image smooth filter radius: %d\n", all_blob_params.r_smooth);
                all_blob_params.r_smooth = new_r_smooth;
                //printf("now image smooth filter radius is: %d\n", all_blob_params.r_smooth);
            }
            //printf("previous high pass filter bool bool: %d\n", all_blob_params.high_pass_filter);
            all_blob_params.high_pass_filter = new_high_pass_filter;
            //printf("now high pass filter bool is: %d\n", all_blob_params.high_pass_filter);
            
            if (new_r_high_pass_filter >= 0) {
                //printf("previous high pass filter radius: %d\n", all_blob_params.r_high_pass_filter);
                all_blob_params.r_high_pass_filter = new_r_high_pass_filter;
                //printf("now high pass filter radius is: %d\n", all_blob_params.r_high_pass_filter);
            } 
            
            if (new_centroid_search_border >= 0) {
                //printf("previous centroid search border: %d\n", all_blob_params.centroid_search_border);
                all_blob_params.centroid_search_border = new_centroid_search_border;
                //printf("now centroid search border is: %d\n", all_blob_params.centroid_search_border);
            } 
            
            //printf("previous filter return image bool: %d\n", all_blob_params.filter_return_image);
            all_blob_params.filter_return_image = new_filter_return_image;
            //printf("now filter return image bool is: %d\n", all_blob_params.filter_return_image);
            
            if (new_n_sigma >= 0) {
                //printf("previous n sigma: %f\n", all_blob_params.n_sigma);
                all_blob_params.n_sigma = new_n_sigma;
                //printf("now n sigma is: %f\n", all_blob_params.n_sigma);
            } 
            
            if (new_unique_star_spacing >= 0) {
                //printf("previous unique star spacing: %d\n", all_blob_params.unique_star_spacing);
                all_blob_params.unique_star_spacing = new_unique_star_spacing;
                //printf("now unique star spacing is: %d\n", all_blob_params.unique_star_spacing);
            } 
            command_lock = 0; // allow other clients to execute commands
        }
        // update astrometry regardless for most recent data
        //doCameraAndAstrometry();
        // TELEMETRY COMPILATION AND TRANSMISSION
        all_data.ra = all_astro_params.ra;
        all_data.dec = all_astro_params.dec;
        all_data.fr = all_astro_params.fr;
        all_data.az = all_astro_params.az;  
        all_data.el = all_astro_params.alt;
        all_data.ir = all_astro_params.ir;
        // transfer camera params struct to one in tcp_data struct to send back to user
        memcpy(&all_data.cam_settings, &all_camera_params, sizeof(all_camera_params));
        memcpy(&all_data.current_blob_params, &all_blob_params, sizeof(all_blob_params));
        //all_data.cam_settings = all_camera_params;
        // transfer blob params struct to one in tcp_data struct to send back to user
       // all_data.current_blob_params = all_blob_params;
        // all_data.spike_limit = all_blob_params.spike_limit;
        // all_data.dynamic_hot_pixels = all_blob_params.dynamic_hot_pixels;
        // all_data.r_smooth = all_blob_params.r_smooth;
        // all_data.high_pass_filter = all_blob_params.high_pass_filter;
        // all_data.r_high_pass_filter = all_blob_params.r_high_pass_filter;
        // all_data.centroid_search_border = all_blob_params.centroid_search_border;
        // all_data.filter_return_image = all_blob_params.filter_return_image;
        // all_data.n_sigma = all_blob_params.n_sigma;
        // all_data.unique_star_spacing = all_blob_params.unique_star_spacing;
        // set time field of telemetry data structure to current universal time 
        time_t seconds; 
        seconds = time(NULL); 
        all_data.curr_time = seconds;
	    //if (sendto(sock, &all_data, sizeof(struct tcp_data), MSG_NOSIGNAL, (const struct sockaddr *) &addr, length) <= 0) {
        if ((send(sock, &all_data, sizeof(struct tcp_data), MSG_NOSIGNAL) <= 0) && 
            (send(sock, &camera_raw, 1936*1216, MSG_NOSIGNAL) <= 0)) {
            printf("Client dropped the connection.\n");
            break;
        } else {
            printf("Telemetry and image sent back to user.\n");
            // send image as well
            //system("rsync /home/xscblast/Desktop/blastcam/BMPs/latest_saved_image.bmp brooke@128.91.43.54:/home/brooke/Desktop/StarCamera/"); // change to any user in the future - IP address is str concatenation, ask Javier about username fix
            //printf("Latest saved image sent to user as well.\n");
        }
    }
    // clean up socket when the connection is done
    close(sock);
}

int main() {
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

    // make socket non-blocking (times out after a certain time)
    struct timeval read_timeout;
    read_timeout.tv_sec = 3;
    read_timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    // initialize the camera
    init_camera();

    // initialize the lens adapter
    int fileDescriptor = init_lensAdapter("/dev/ttyLens");

    // call astrometry calculation function once to properly initialize structure -- THIS IS A TEMPORARY SOLUTION
    // (without this initial call, astrometry calculations and telemetry sent to user is not correct)
    //doCameraAndAstrometry();

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
        // update astrometry regardless for most recent data
        //doCameraAndAstrometry();
        // store length of client socket that has connected (if any)
        clientAddressLength = sizeof(clientAddress);
        // pointer of client socket
        // uintptr_t the_sock = newsockfd;
        if (newsockfd == -1) {
            // printf("Image height before struct access by camera.c: %i\n", all_data.imageHeight);
            // camera stuff
            //doCameraAndAstrometry();
            //printf("Image height after struct access by camera.c: %i\n", all_data.imageHeight);
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
            if (pthread_create(&thread_id, NULL, client_handler, (void*) socket_args) < 0) {
                perror("Could not create thread for client.\n");
            }
        }
    }
}