#include <sys/types.h>  
#include <sys/socket.h> 
#include <string.h>     
#include <stdlib.h>     
#include <netinet/in.h>     
#include <stdio.h>          
#include <arpa/inet.h>
#include <ifaddrs.h> 
#include <netdb.h>         
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

#pragma pack(push, 1)
/* Telemetry and camera settings structure */
struct telemetry {
    struct astrometry astrom;
    struct camera_params cam_settings; 
    struct blob_params current_blob_params;
};
/* User commands structure */
struct commands {
    double logodds;         // controls Astrometry false positives
    double latitude;        // user's latitude (radians)
    double longitude;       // user's longitude (degrees)
    double height;          // user's height above ellipsoid from GPS (WGS84)
    double exposure;        // user's desired camera exposure (msec)
    double timelimit;       // Astrometry tries to solve until this timeout
    float focus_pos;        // user's desired focus position (counts)
    int focus_mode;         // flag to enter auto-focusing mode
    int start_focus_pos;    // where to start the auto-focusing process
    int end_focus_pos;      // where to end the auto-focusing process
    int focus_step;         // granularity of auto-focusing checker
    int photos_per_focus;   // number of photos per auto-focusing position
    int set_focus_inf;      // whether or not to set the focus to infinity
    int aperture_steps;     // number of shifts (+/-) to reach desired aperture
    int set_max_aperture;   // whether or not to maximize aperture
    int make_hp;            // flag to make new static hp mask (20 = re-make)
    int use_hp;             // flag to use current static hp mask
    float blob_params[9];   // rest of blob-finding parameters
};
/* Struct for passing arguments to client thread(s) from main thread */
struct args {
    uintptr_t user_socket;
    socklen_t user_length;
    struct sockaddr_in user_address;
    char * user_IP;
};
#pragma pack(pop)

struct commands all_cmds = {0};
struct telemetry all_data = {0};
int num_clients = 0;
int telemetry_sent = 0;
// flag for cancelling auto-focus mid-process
int cancelling_auto_focus = 0;
// assume non-verbose output
int verbose = 0;
void * camera_raw = NULL;
// if 1, then commanding is in use; if 0, then not
int command_lock = 0;
// if 0, then camera is not closing, so keep solving astrometry       
int shutting_down = 0;
// return values for terminating the threads
int astro_thread_ret, client_thread_ret;

/* Helper function for testing reception of user commands.
** Input: Nones.
** Output: None (void). Prints the most recent user commands to the terminal.
*/
void verifyUserCommands() {
    printf("\n**** USER COMMANDS ****\n");
    printf("Logodds command: %f\n", all_cmds.logodds);
    printf("Latitude | longitude: %f | %f\n", all_cmds.latitude, 
                                              all_cmds.longitude);
    printf("Exposure command in commands.c: %f\n", all_cmds.exposure);
    printf("Astrometry timeout: %f\n", all_cmds.timelimit);
    printf("Focus mode: %s\n", (all_cmds.focus_mode) ? "Auto-focus" : 
                                                       "Normal focus");
    printf("Start focus: %i, end focus %i, focus step %i\n", 
            all_cmds.start_focus_pos, all_cmds.end_focus_pos, 
            all_cmds.focus_step);
    printf("Photos per focus: %d\n", all_cmds.photos_per_focus);
    printf("Focus position command: %f\n", all_cmds.focus_pos);
    printf("Set focus to infinity bool command: %i\n", all_cmds.set_focus_inf);
    printf("Aperture steps command: %i\n", all_cmds.aperture_steps);
    printf("Set aperture max bool: %i\n", all_cmds.set_max_aperture);
    printf("Make static hp mask: %i, use static hp mask: %i\n", 
            all_cmds.make_hp, all_cmds.use_hp);
    printf("Blob parameters: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
            all_cmds.blob_params[0], all_cmds.blob_params[1], 
            all_cmds.blob_params[2], all_cmds.blob_params[3], 
            all_cmds.blob_params[4], all_cmds.blob_params[5],
            all_cmds.blob_params[6], all_cmds.blob_params[7], 
            all_cmds.blob_params[8]);
    printf("***********************\n\n");
}

/* Helper function for testing transmission of telemetry.
** Input: None.
** Output: None (void). Prints the telemetry sent to the user to the terminal.
*/
void verifyTelemetryData() {
    printf("\n**** TELEMETRY SENT TO USER ****\n");
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

/* Function devoted to taking pictures and solving astrometry while camera is 
** not in a state of shutting down.
** Input: None.
** Output: None (void). 
*/
void * updateAstrometry() {
    // solve astrometry perpetually when the camera is not shutting down
    while (!shutting_down) {
        if (doCameraAndAstrometry() < 1) {
            printf("Did not solve or timeout of Astrometry properly, or did "
                   "not auto-focus properly.\n");
        }
    }

    // when we are shutting down or exiting, close Astrometry engine and solver
    closeAstrometry();

    // terminate and exit thread
    astro_thread_ret = 1;
    pthread_exit(&astro_thread_ret);
}

/* Function for accepting newly connected clients and sending telemetry and 
** receiving their commands.
** Input: An args struct containing the client information.
** Output: None (void).
*/
void * processClient(void * for_client_thread) {
    // get the socket information for this user
    int socket = ((struct args *) for_client_thread)->user_socket;
    int length = ((struct args *) for_client_thread)->user_length;
    struct sockaddr_in address = ((struct args *) 
                                   for_client_thread)->user_address;
    char * ip_addr = ((struct args *) for_client_thread)->user_IP;
    int recv_status;

    // send data to this client perpetually and receive potential commands if 
    // user sends them
    while (1) {
        recv_status = recvfrom(socket, &all_cmds, sizeof(struct commands), 0, 
                              (struct sockaddr *) &address, &length); 
        if (recv_status == -1) {
            printf("User %s did not send any commands. Send telemetry and "
                   "camera settings back anyway.\n", ip_addr);
        } else {
            // if another client sends commands at the same time, wait until 
            // they are done
            while (command_lock) {
                usleep(100000);
            }
            // now it's this client's turn to execute commands (lock)
            command_lock = 1;
            printf("User %s sent commands. Executing...\n", ip_addr);
            if (verbose) {
                verifyUserCommands();
            }

            // some constants for solving Astrometry
            all_astro_params.logodds = all_cmds.logodds;
            all_astro_params.latitude = all_cmds.latitude;
            all_astro_params.longitude = all_cmds.longitude;
            all_astro_params.hm = all_cmds.height;
            all_astro_params.timelimit = all_cmds.timelimit;

            // update blob-finding parameters (see camera.h for documentation)
            all_blob_params.make_static_hp_mask = all_cmds.make_hp;            
            all_blob_params.use_static_hp_mask = all_cmds.use_hp;                 

            if (all_cmds.blob_params[0] >= 0) {
                all_blob_params.spike_limit = all_cmds.blob_params[0];        
            } 
            
            all_blob_params.dynamic_hot_pixels = all_cmds.blob_params[1];    
        
            if (all_cmds.blob_params[2] >= 0) {
                all_blob_params.r_smooth = all_cmds.blob_params[2];      
            }

            all_blob_params.high_pass_filter = all_cmds.blob_params[3];   
            
            if (all_cmds.blob_params[4] >= 0) {
                all_blob_params.r_high_pass_filter = all_cmds.blob_params[4]; 
            } 
            
            if (all_cmds.blob_params[5] >= 0) {
                all_blob_params.centroid_search_border = all_cmds.blob_params[5];
            } 

            all_blob_params.filter_return_image = all_cmds.blob_params[6];  

            if (all_cmds.blob_params[7] >= 0) {
                all_blob_params.n_sigma = all_cmds.blob_params[7];     
            } 
            
            if (all_cmds.blob_params[8] >= 0) {
                all_blob_params.unique_star_spacing = all_cmds.blob_params[8];
            } 

            if (!all_cmds.focus_mode && all_camera_params.focus_mode) {
                printf("\nCancelling auto-focus process!\n");
                cancelling_auto_focus = 1;
            } else {
                // need to reset cancellation flag to 0 if we are not auto-
                // focusing at all, we are remaining in auto-focusing, or we are
                // entering auto-focusing
                cancelling_auto_focus = 0;
            }

            // performing auto-focusing will restrict some of the other cmds, so
            // check that before other lens commands & hardware adjustments
            if (all_cmds.focus_mode) {
                all_camera_params.begin_auto_focus = 1;
            }
            all_camera_params.focus_mode = all_cmds.focus_mode;
            all_camera_params.start_focus_pos = all_cmds.start_focus_pos;
            all_camera_params.end_focus_pos = all_cmds.end_focus_pos;
            all_camera_params.focus_step = all_cmds.focus_step;
            all_camera_params.photos_per_focus = all_cmds.photos_per_focus;

            if (!all_camera_params.focus_mode && !cancelling_auto_focus) {
                // if user adjusted exposure, set exposure to their value
                if (ceil(all_cmds.exposure) != 
                    ceil(all_camera_params.exposure_time)) {
                    // update value in camera params struct as well
                    all_camera_params.exposure_time = all_cmds.exposure;
                    all_camera_params.change_exposure_bool = 1;
                }

                // contradictory commands between setting focus to inf and 
                // adjusting it to a different position are handled in 
                // adjustCameraHardware()
                all_camera_params.focus_inf = all_cmds.set_focus_inf;
                // if user wants to change focus, change focus in camera params
                if (all_cmds.focus_pos != -1) {
                    all_camera_params.focus_position = all_cmds.focus_pos; 
                } 

                // update camera params struct with user commands
                all_camera_params.max_aperture = all_cmds.set_max_aperture;
                all_camera_params.aperture_steps = all_cmds.aperture_steps;

                // if we are taking an image right now, need to wait to execute
                // any lens commands
                while (taking_image) {
                    usleep(100000);
                }

                // perform changes to camera settings in lens_adapter.c (focus, 
                // aperture, and exposure deal with camera hardware)
                if (adjustCameraHardware() < 1) {
                    printf("Error executing at least one user command.\n");
                }
            } else {
                printf("In or entering auto-focusing mode, or cancelling "
                       "current auto-focus process, so ignore lens " 
                       "commands.\n");
            }

            // allow other clients to execute commands (unlock)
            command_lock = 0; 
        }
        telemetry_sent = 0;

        while (!send_data) {
            usleep(100000);
        }

        // compile telemetry and transmit back to use
        memcpy(&all_data.astrom, &all_astro_params, 
                sizeof(all_astro_params));
        memcpy(&all_data.cam_settings, &all_camera_params, 
                sizeof(all_camera_params));
        memcpy(&all_data.current_blob_params, &all_blob_params, 
                sizeof(all_blob_params));

        if (send(socket, &all_data, sizeof(struct telemetry), 
                 MSG_NOSIGNAL) <= 0) {
            printf("Client dropped the connection.\n");
            break;
        } 

        if (send(socket, camera_raw, CAMERA_WIDTH*CAMERA_HEIGHT, 
                 MSG_NOSIGNAL) <= 0) {
            printf("Client dropped the connection.\n");
            break;
        }

        telemetry_sent = 1;
        
        if (verbose) {
            printf("Telemetry and image bytes sent back to user.\n"); 
            verifyTelemetryData();
        }
    }
    // clean up socket when the connection is done
    close(socket);
    num_clients--;

    // in case user exits in middle of two send() calls above, we don't want to
    // wait forever in auto-focusing
    telemetry_sent = 1;

    // free(for_client_thread);
    client_thread_ret = 1;
    pthread_exit(&client_thread_ret);
}

/* Driver function for Star Camera operation.
** Input: Number of command-line arguments passed and an array of those argu-
** ments.
** Output: Flag indicating successful Star Camera operation or an error -> exit.
*/
int main(int argc, char * argv[]) {
    // signal handling (e.g. ctrl+c exception)
    signal(SIGHUP, clean);
    signal(SIGINT, clean);
    signal(SIGTERM, clean);
    signal(SIGPIPE, SIG_IGN);

    char * lens_desc;                // cmd-line serial port from user
    int port;                        // port to establish socket on from user
    int sockfd;                      // to create socket
    int newsockfd;                   // to accept new connection
    struct sockaddr_in serv_addr;    // server receives on this address
    struct sockaddr_in client_addr;  // server sends to client on this address
    struct timeval read_timeout;     // timeout options for server socket 
    int client_addr_len;             // length of client addresses
    pthread_t client_thread_id;      // thread ID for new clients        
    pthread_t astro_thread_id;       // thread ID for Astrometry thread
    struct args * client_args;       // arguments to pass to clients
    int * astro_ptr = NULL;          // ptr for returning from Astrometry thread
    int * client_ptr = NULL;         // ptr for returning from client thread
    int any_clients = 0;             // if a client every connected during run
    int ret;                         // return status of main()

    // parse command-line arguments
    if (argc == 2 && strcmp(argv[1], "--help") == 0) {
        printf("Star Camera command-line program.\n\nSpecify a valid camera "
               "handle from 1 to 254,\nthe known string descriptor for the "
               "serial port,\nand the port to establish a socket on. Or:\n\n"
               "--network: see Star Camera computer's current IP address\n"
               "for connection via the Star Camera application on "
               "another\ncomputer.\n\n--number: current number of cameras "
               "connected to this\ncomputer.\n\n<handle> <serial port> <port> "
               "--info: run program as usual,\noutputting information for the "
               "camera as well.\n\n--valid: valid combinations of the above"
               "input arguments.\n\n<handle> <serial port> <port> -v: for " 
               "verbose output.\n");
        return 1;
    } else if (argc == 2 && strcmp(argv[1], "--network") == 0) {
        printf("Enter the address is the 'inet addr' field below into\nthe " 
               "Star Camera application to connect:\n\n");
        system("/sbin/ifconfig enp3s0");
        printf("Size of data packet that will be sent to user: %lu bytes\n", 
               sizeof(all_data));
        return 1;
    } else if (argc == 2 && strcmp(argv[1], "--number") == 0) {
        int num_cams;

        if (is_GetNumberOfCameras(&num_cams) != IS_SUCCESS) {
            printf("Unable to get number of cameras connected to computer.\n");
            return 0;
        } else {
            printf("Number of cameras connected to Star Camera computer: %d.\n",
                   num_cams);
            return 1;
        }
    } else if (argc == 2 && strcmp(argv[1], "--valid") == 0) {
        printf("The valid <handle> <serial port> <socket port> argument "
               "combinations are:\n");
        // enter these combinations once known
        return 1;
    } else if (argc > 5) {
        printf("Too many arguments supplied.\n./commands --option, or"
               "\n./commands <camera handle> <serial port> <socket port>\n");
        return 0;
    } else if (argc < 4) {
        printf("Missing arguments.\n./commands --option, or"
               "\n./commands <camera handle> <serial port> <socket port>\n");
        return 0;
    } else {
        printf("*** Star Camera command-line interface ***\nCamera handle: "
               "%s\nSerial Port: %s\nSocket Port: %s\n\n", argv[1], argv[2],
                                                           argv[3]);
        camera_handle = atoi(argv[1]);
        if (camera_handle > 254 || camera_handle < 1) {
            printf("Invalid camera handle. Choose one in the range 1-254.\n");
            return 0;
        }

        lens_desc = argv[2];
        port = atoi(argv[3]);
        if (port < 0 || port > 65535) {
            printf("Invalid port number. Ports must be between 0 and 65535.\n");
            return 0;
        }
    }

    // create server socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        fprintf(stderr, "Error creating Star Camera server socket: %s.\n", 
                strerror(errno));
        // the program did not successfully run
        exit(EXIT_FAILURE);
    }

    // initialize the server socket addresses
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(port); 

    // bind the server socket with the server address and port
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        fprintf(stderr, "Error binding Star Camera server socket to Star Camera "
                        "address and port: %s. Try again in a few seconds - "
                        "resources may not be freed if last run just ended.\n", 
                strerror(errno));
        // the program did not successfully run
        exit(EXIT_FAILURE);
    }

    // listen for connections from clients (maximum is 5 waiting)
    if (listen(sockfd, 5) < 0) {
        fprintf(stderr, "Star Camera server error listening for clients: %s.\n", 
                strerror(errno));
        exit(EXIT_FAILURE);
    }

    // make server socket non-blocking (times out after a certain time)
    read_timeout.tv_sec = 3;
    read_timeout.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, 
                   sizeof(read_timeout)) < 0) {
        fprintf(stderr, "Error setting Star Camera server timeout: %s.\n", 
                strerror(errno));
        exit(EXIT_FAILURE);
    }

    // allow address to be reused (in case of shutdown with client still
    // connected)
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &(int){1}, 
                   sizeof(int)) < 0) {
        fprintf(stderr, "Error setting server reuseaddrr option: %s.\n", 
                strerror(errno));
        exit(EXIT_FAILURE);
    }
    
    if (argc == 5 && strcmp(argv[4], "-v") == 0) {
        verbose = 1;
    }

    // initialize the camera with input ID
    if (initCamera() < 0) {
        printf("Could not initialize camera due to above error. Could be that"
               "you specified a handle for a camera already in use.\n");
        // if camera was already initialized, close it before exiting
        if (camera_handle > 0) {
            closeCamera();
        }
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (argc == 5 && strcmp(argv[4], "--info") == 0) {
        CAMINFO cam_info;
        if (is_GetCameraInfo(camera_handle, &cam_info) != IS_SUCCESS) {
            printf("Could not get camera information.\n");
            closeCamera();
            exit(EXIT_FAILURE);
        }

        printf("----------------- Information -----------------\n");
        printf("Serial number: %s\n", cam_info.SerNo);
        printf("Camera manufacturer: %s\n", cam_info.ID);
        printf("Camera version: %s\n", cam_info.Version);
        printf("System date of final quality check: %s\n", cam_info.Date);
        printf("Customizable, persistent handle ID\nstored in the camera: %u\n", 
               cam_info.Select);
        printf("Camera type: %hhu\n", cam_info.Type);
        printf("-----------------------------------------------\n");
    }

    // initialize the lens adapter
    if (initLensAdapter(lens_desc) < 0) {
        printf("Could not initialize lens adapter due to above error.\n");
        closeCamera();
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // create a thread separate from all client thread(s) to solve Astrometry 
    if (pthread_create(&astro_thread_id, NULL, updateAstrometry, NULL) != 0) {
        fprintf(stderr, "Error creating Astrometry thread: %s.\n", 
                strerror(errno));
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // loop forever, accepting new clients
    client_addr_len = sizeof(struct sockaddr_in); 
    while ((!shutting_down) && (newsockfd = accept(sockfd, (struct sockaddr *) 
                                                           &client_addr, 
                                                           &client_addr_len))) {
        // parent process waiting to accept a new connection
        printf("\n*************************** Server waiting for new client, "
               "%d connected already ***************************\n", 
               num_clients);
        // store length of client that has connected (if any)
        client_addr_len = sizeof(client_addr);
        if (newsockfd == -1) {
            printf("New client did not connect.\n");
        } else {
            any_clients = 1;

            // user did connect so process their info for their client thread
            printf("Client %s connected.\n", inet_ntoa(client_addr.sin_addr)); 

            client_args = (struct args *) malloc(sizeof(struct args));
            if (client_args == NULL) {
                fprintf(stderr, "Error creating struct for new client: %s.\n", 
                        strerror(errno));
                continue;
            }

            client_args->user_socket = newsockfd;
            client_args->user_length = client_addr_len;
            client_args->user_address = client_addr;
            client_args->user_IP = inet_ntoa(client_addr.sin_addr);

            // create new thread for this new client
            if (pthread_create(&client_thread_id, NULL, processClient, 
                              (void *) client_args) < 0) {
                fprintf(stderr, "Error creating thread for new client: %s.\n", 
                        strerror(errno));
            }

            num_clients++;
        }
    }
    
    // join threads once the Astrometry thread has closed and terminated
    pthread_join(astro_thread_id, (void **) &(astro_ptr));
    if (any_clients) {
        pthread_join(client_thread_id, (void **) &(client_ptr));
        free(client_args);
    }

    closeCamera();
    shutdown(sockfd, SHUT_RDWR);
    close(sockfd);

    if (*astro_ptr == 1) {
        printf("Successfully exited Astrometry.\n");
        ret = 1;
    } else {
        printf("Did not return successfully from Astrometry thread.\n");
        ret = 0;
    }

    if (any_clients) {
        if (*client_ptr == 1) {
            printf("Successfully exited client thread.\n");
            ret = 1;
        } else {
            printf("Did not return successfully from client thread.\n");
            ret = 0;
        }
    }

    return ret;
}