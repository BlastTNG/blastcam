/* HOW TO USE THIS PROGRAM:

This program is designed so that only two-letter functions need to be input to execute commands.
e.g., comStatus = runCommand("mi\r", fileDescriptor, returnVal);
where 'mi' is the function and the only thing that needs to be changed.

All camera settings are listed in the user manual for the camera.

FUNCTIONS:
	Aperture:
		in: initialize aperture motor; aperture will fully open
		da: print aperture information
		ma: move aperture to absolute position
		mc: move aperture to fully stopped down limit
		mn: move aperture incremental (mn2 moves the aperture by +2, not to +2)
		mo: move aperture to completely open
		pa: print aperture position
	Focus:
		eh: set absolute lens focus position (0...0x3FFF)
		fa: move focus to absolute position
		fd: print focus distance range
		ff: fast focus
		fp: print the raw focus positions
		la: learn the focus range
		mf: move focus incremental (mf200 moves the focus by +200, not to +200, mf-200 increments down 200)
		mi: move focus to the infinity stop
		mz: move focus to the zero stop
		pf: print focus position
		sf: set the focus counter
	Misc:
		bv: print the bootloader version
		de: dump EEPROM
		ds: prints distance stops
		dz: prints the zoom range
		ex: exit to the bootloader
		gs: echo current device and lens statuses
		hv: print the hardware version
		id: print basic lens identification (zoom and f-number)
		is: turn image stabilization off/on
		ll: library loaded check
		lp: lens presence
		ls: query lens for status immediately and print
		lv: print the library version string
		pl: lens power
		rm: set response modes
		se: temporarily set non-volatile (EEMPROM) byte
		sg: set GPIO
		sm: set special modes
		sn: print the device serial number
		sr: set spontaneous responses off/on
		vs: print the short version string
		we: write non-volatile parameters to EEPROM
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <pthread.h>
#include <sys/time.h>
#include <dirent.h>
#include <sched.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <ueye.h>
#include <stdbool.h>

#include "lens_adapter.h"
#include "astrometry.h"

int runCommand(const char * command, int file, char * returnStr);
char * returnVal;
int comStatus;
char * buffer;
int fileDescriptor;
double focus_shift;

int dummy_storage;

// initialize the camera parameters global structure
struct camera_params all_camera_params = {
  .prev_focus_pos = 0, // need to save previous focus value to determine by how much we need to move in mf command below
  .focus_position = 0, 
  .focus_inf = 0,     
  .aperture_steps = 0,
  .max_aperture = 0,   // (bool) default is to maximize aperture
  // these fields are for information for user GUI, not for changing camera settings
  .current_aperture = 0,
  .min_focus_pos = 0,
  .max_focus_pos = 0,
};

// initialize the lens adapter and run commands
int init_lensAdapter (char * path) {
    // open file descriptor
    fileDescriptor = open(path, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
    if (fileDescriptor == -1) {
        printf("Error opening the file descriptor for lens %s; error %d\n", path, errno);
        return -1;
    }

    struct termios options;

    tcgetattr(fileDescriptor, &options);
    cfsetispeed(&options, B115200);               // input speed
    cfsetospeed(&options, B115200);               // output speed

    // standard setting for DSP 1750
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 cha
    options.c_iflag |= IGNBRK;          // ignore break signal
    options.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
    options.c_oflag = 0;                // no remapping, no delays
    options.c_cc[VMIN]  = 1;            // read doesn't block
    options.c_cc[VTIME] = 5;            // tenths of seconds for read timeout (intege
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    options.c_cflag &= ~CSTOPB;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls,
                                                // enable reading
    options.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS; // turns off flow control maybe?
                                 // http://pubs.opengroup.org/onlinepubs/009695399/basedefs/xbd_chap11.html#tag_11_02_04
                                 // ^list of possible c_cflag options (doesnt include crtscts)
                                 // crtscts does not exist in termios.h documentation
    options.c_iflag |= ICRNL;
    // sets a read timeout of 2 seconds so it doesn't block forever
    options.c_lflag &= ~ICANON;
    options.c_cc[VTIME] = 2;
    options.c_cc[VMIN] = 0;
    // options.c_iflag |= IGNCR;

    tcsetattr(fileDescriptor, TCSANOW, &options);     // apply changes

    // flush the buffer (in case of unclean shutdown)
    if (tcflush(fileDescriptor, TCIOFLUSH) < 0) {
        printf("Buffer fails to flush in add lens.\n");
    }

    returnVal = malloc(100);

    // set focus to 80 below infinity (hard-coded value focus position determined by testing with iDS and uEye software)
    comStatus = runCommand("la\r", fileDescriptor, returnVal);
    comStatus = runCommand("mi\r", fileDescriptor, returnVal);
    comStatus = runCommand("mf -80\r", fileDescriptor, returnVal);
    if (comStatus == -1) {
        printf("Failed to move the focus to the desired default position.\n");
    } else {
        printf("Focus moved to desired default position.\n");
    }
    // print focus position
    printf("Focus range at 80 counts below infinity:\n");
    comStatus = runCommand("fp\r", fileDescriptor, returnVal);
    if (comStatus == -1) {
        printf("Failed to print the new focus position.\n");
    } 

    // set aperture parameter to maximum
    all_camera_params.max_aperture = 1;
    // initialize the aperture motor
    comStatus = runCommand("in\r", fileDescriptor, returnVal);
    if (comStatus == -1) {
        printf("Failed to initialize the motor.\n");
    }
    // run the aperture maximization command
    comStatus = runCommand("mo\r", fileDescriptor, returnVal);
    if (comStatus == -1) {
        printf("Setting the aperture to maximum fails.\n");
    }
    // print aperture position
    comStatus = runCommand("pa\r", fileDescriptor, returnVal);
    if (comStatus == -1) {
        printf("Failed to print the new aperture position.\n");
    } 

    free(returnVal);
    return fileDescriptor;
}

void handleFocusAndAperture(int fileDescriptor) {
    // If user set focus infinity command to true (1), execute this command and none of the other focus commands 
    // that would contradict this one
    if (all_camera_params.focus_inf == 1) {
        comStatus = runCommand("mi\r", fileDescriptor, returnVal);
        if (comStatus == -1) {
            printf("Failed to set focus to infinity.\n");
        } else {
            printf("Focus set to infinity.\n");
        }
        comStatus = runCommand("fp\r", fileDescriptor, returnVal);
        if (comStatus == -1) {
            printf("Failed to print focus after setting to infinity.\n");
        } 
    }  else {
        // calculate the shift needed to get from current focus position to user-specified focus position
        focus_shift = all_camera_params.focus_position - all_camera_params.prev_focus_pos;
        printf("Focus change (internal calculation, not command): %f\n", focus_shift);
        char focus_str_cmd[10]; // (4 characters for "mf +" and then 6 for 6 digits of precision in focus shift number)
        // account for the sign of the shift in making the string command for runCommand()
        if (focus_shift != 0) {
            if (focus_shift > 0) {
                sprintf(focus_str_cmd, "mf +%f\r", focus_shift);
            } else {
                sprintf(focus_str_cmd, "mf %f\r", focus_shift);
            }
            // shift the focus (perform the command)
            comStatus = runCommand(focus_str_cmd, fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to move the focus to the desired position.\n");
            } else {
                printf("Focus moved to desired absolute position.\n");
            }
            // print focus position
            comStatus = runCommand("fp\r", fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to print the new focus position.\n");
            }  
        }
    }

    // initialize the aperture motor
    //comStatus = runCommand("in\r", fileDescriptor, returnVal);
    //if (comStatus == -1) {
    //    printf("Failed to initialize the motor.\n");
    //}

    // set the aperture to the maximum
    if (all_camera_params.max_aperture == 1) {
        all_camera_params.current_aperture = 28;
        comStatus = runCommand("mo\r", fileDescriptor, returnVal); 
        if (comStatus == -1) {
            printf("Setting the aperture to maximum fails.\n");
        } else {
            printf("Set aperture to maximum.\n");
        }
    } else {
        char aper_str_cmd[4]; 
        if (all_camera_params.aperture_steps != 0) {
            sprintf(aper_str_cmd, "mn%i\r", all_camera_params.aperture_steps);
            printf("%s\n", aper_str_cmd);
            // perform the aperture command
            comStatus = runCommand(aper_str_cmd, fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to adjust the aperture.\n");
            } else {
                printf("Adjusted the aperture successfully.\n");
            }
            // print aperture position
            comStatus = runCommand("pa\r", fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to print the new aperture position.\n");
            }
            // now that command in aperture has been executed, set aperture steps field to 0 since it should not
            // move again unless user sends another command
            all_camera_params.aperture_steps = 0;  
        }
    }
}

// defines runCommand() to execute built-in camera commands
int runCommand(const char * command, int file, char * returnStr){
    fd_set input, output;
    FD_ZERO(&output);
    FD_SET(file, &output);

    if (!FD_ISSET(file, &output)) {
        printf("Communication error in runCommand.\n");
        return -1;
    }

    tcflush(file, TCIOFLUSH);
    int status = write(file, command, strlen(command));
    if (status < 0) {
        printf("Write of command on file %d has failed; error %d\n", file, errno);
        return -1;
    }

    usleep(1000000);

    FD_ZERO(&input);
    FD_SET(file, &input);

    if (!FD_ISSET(file, &input)) {
        printf("Communication error in runCommand on read.\n");
        return -1;
    }

    buffer = malloc(100);
    buffer[0] = '\0';
    status = read(file, buffer, 99);
    if (status <= 0) {
        printf("Read fails in runCommand; errno = %d\n", errno);
        return -1;
    }

    buffer[99] = '\0';
    buffer[status] = '\0';	


    if (strstr(buffer, "ERR") != NULL) {
        printf("Read returned error %s.\n", buffer);
        return -1;
    } else if (strstr(buffer, "OK") != NULL) {
        // printf("getting ok in runcommand, length %d, buffer contents:%send\n", status, buffer);
    }
    
    strcpy(returnStr, buffer);

    if (strcmp(command, "fp\r") == 0) {
        printf("%s\n", returnStr);
        sscanf(returnStr, "fp\nOK\nfmin:%d  fmax:%d  current:%i %*s", &all_camera_params.min_focus_pos, &all_camera_params.max_focus_pos, &all_camera_params.focus_position);
        printf("in camera params, min focus pos is: %i\n", all_camera_params.min_focus_pos);
        printf("in camera params, max focus pos is: %i\n", all_camera_params.max_focus_pos);
        printf("in camera params, curr focus pos is: %i\n", all_camera_params.focus_position);
        printf("in camera params, prev focus pos was: %i\n", all_camera_params.prev_focus_pos);
        all_camera_params.prev_focus_pos = all_camera_params.focus_position;
        printf("in camera params, prev focus pos is now: %i\n", all_camera_params.prev_focus_pos);
    } else if (strcmp(command, "pa\r") == 0) {
        printf("%s\n", returnStr);
        if (strncmp(returnStr, "pa\nOK\nDONE", 10) == 0) {
            sscanf(returnStr, "pa\nOK\nDONE%*i,f%d", &all_camera_params.current_aperture);
        } else if (strncmp(returnStr, "pa\nOK\n", 6) == 0) {
            sscanf(returnStr, "pa\nOK\n%*i,f%d %*s", &all_camera_params.current_aperture);
        }
        printf("in camera params, curr aper is: %i\n", all_camera_params.current_aperture);
    } else if (strncmp(command, "mf", 2) == 0) {
        printf("%s\n", returnStr);
    }

    free(buffer);

    return 0;
}