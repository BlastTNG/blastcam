/* HOW TO USE THIS PROGRAM:

This program is designed so that only two-letter commands need to be input to execute camera changes.
e.g., comStatus = runCommand("mi\r", fileDescriptor, returnVal);
where 'mi' is the command.

All camera settings are listed in the Canon EF 232 user manual.
https://birger.com/products/lens_controller/manuals/canon_ef232/Canon%20EF-232%20Library%20User%20Manual%201.3.pdf

COMMANDS:
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
		mf: move focus incremental (mf200 moves the focus by +200, not to +200; mf-200 increments down 200)
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

// include necessary libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <ueye.h>

// include our header files
#include "lens_adapter.h"
#include "camera.h"

// initialize the camera parameters global structure (defined in camera.h)
struct camera_params all_camera_params = {
    .prev_focus_pos = 0,       // need to save previous focus value to determine by how much we need to move in mf command below
    .focus_position = 0,       // current focus position
    .focus_inf = 0,            // (bool) default is not to set focus to infinity (ideal position is 80 counts below infinity)
    .aperture_steps = 0,       // number of aperture steps to shift by in mn command
    .max_aperture = 0,         // (bool) default is not to maximize aperture, since it will already be maximized from init_camera()
    .current_aperture = 0,     // current aperture position
    .min_focus_pos = 0,        // current min focus position (focus range changes with each power cycle of the camera)
    .max_focus_pos = 0,        // current max focus position
    .exposure_time = 800,      // current exposure time (700 msec is default)
    .change_exposure_bool = 0, // don't want to change the exposure from default value unless user commands it
};

char * returnVal;
int comStatus;
char * buffer;
int fileDescriptor;
int focusShift;
double currentExposure;

// initialize the lens adapter and run commands to achieve default settings
int init_lensAdapter(char * path) {
    // open file descriptor with given path
    fileDescriptor = open(path, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
    if (fileDescriptor == -1) {
        printf("Error opening the file descriptor for lens %s; error %d\n", path, errno);
        return -1;
    }

    struct termios options;

    tcgetattr(fileDescriptor, &options);
    cfsetispeed(&options, B115200);                         // input speed
    cfsetospeed(&options, B115200);                         // output speed

    // standard setting for DSP 1750
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 cha
    options.c_iflag |= IGNBRK;                              // ignore break signal
    options.c_lflag = 0;                                    // no signaling chars, no echo,
                                                            // no canonical processing
    options.c_oflag = 0;                                    // no remapping, no delays
    options.c_cc[VMIN]  = 1;                                // read doesn't block
    options.c_cc[VTIME] = 5;                                // tenths of seconds for read timeout (intege
    options.c_iflag &= ~(IXON | IXOFF | IXANY);             // shut off xon/xoff ctrl
    options.c_cflag &= ~CSTOPB;
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cflag |= (CLOCAL | CREAD);                    // ignore modem controls,
                                                            // enable reading
    options.c_cflag &= ~(PARENB | PARODD);                  // shut off parity
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;                            // turns off flow control maybe?
                                                            // http://pubs.opengroup.org/onlinepubs/009695399/basedefs/xbd_chap11.html#tag_11_02_04
                                                            // ^list of possible c_cflag options (doesnt include crtscts)
                                                            // crtscts does not exist in termios.h documentation
    options.c_iflag |= ICRNL;
    // sets a read timeout of 2 seconds so it doesn't block forever
    options.c_lflag &= ~ICANON;
    options.c_cc[VTIME] = 2;
    options.c_cc[VMIN] = 0;
    // options.c_iflag |= IGNCR;

    // apply changes
    tcsetattr(fileDescriptor, TCSANOW, &options);

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
    printf("Focus at 80 counts below infinity:\n");
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

    // run the aperture maximization (fully open) command
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

// function handle user commands for changing settings
void handleFocusAndAperture(int fileDescriptor) {
    // local variables for executing commands
    char focus_str_cmd[10]; 
    char aper_str_cmd[4]; 

    // if user set focus infinity command to true (1), execute this command and none of the other focus commands 
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
    } else {
        // calculate the shift needed to get from current focus position to user-specified position
        focusShift = all_camera_params.focus_position - all_camera_params.prev_focus_pos;
        printf("Focus change (internal calculation, not command): %i\n", focusShift);

        if (focusShift != 0) {
            sprintf(focus_str_cmd, "mf %i\r", focusShift);
            // shift the focus 
            comStatus = runCommand(focus_str_cmd, fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to move the focus to the desired position.\n");
            } else {
                printf("Focus moved to desired absolute position.\n");
            }
            // print focus position for confirmation
            comStatus = runCommand("fp\r", fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to print the new focus position.\n");
            }  
        }
    }

    // if the user wants to set the aperture to the maximum
    if (all_camera_params.max_aperture == 1) {
        // might as well change struct field here since we know what the maximum aperture position is
        // (don't have to get it with pa command)
        all_camera_params.current_aperture = 28;
        comStatus = runCommand("mo\r", fileDescriptor, returnVal); 
        if (comStatus == -1) {
            printf("Setting the aperture to maximum fails.\n");
        } else {
            printf("Set aperture to maximum.\n");
        }
    } else {
        if (all_camera_params.aperture_steps != 0) {
            sprintf(aper_str_cmd, "mn%i\r", all_camera_params.aperture_steps);
            // perform the aperture command
            comStatus = runCommand(aper_str_cmd, fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to adjust the aperture.\n");
            } else {
                printf("Adjusted the aperture successfully.\n");
            }
            // print new aperture position
            comStatus = runCommand("pa\r", fileDescriptor, returnVal);
            if (comStatus == -1) {
                printf("Failed to print the new aperture position.\n");
            }
            // now that command in aperture has been executed, set aperture steps field to 0 since it should not
            // move again unless user sends another command
            all_camera_params.aperture_steps = 0;  
        }
    }

    if (all_camera_params.change_exposure_bool) {
        // change boolean to 0 so exposure isn't adjusted again until user sends another command
        all_camera_params.change_exposure_bool = 0;
        // run uEye function to update camera exposure
        if (is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void *) &all_camera_params.exposure_time, sizeof(double)) != IS_SUCCESS) {
            printf("Adjusting exposure to user command unsuccessful.\n");
        }
        // check with currentExposure that exposure has been adjusted to desired value
        is_Exposure(cameraHandle, IS_EXPOSURE_CMD_GET_EXPOSURE, &currentExposure, sizeof(double));
        printf("Exposure is now %f msec.\n", currentExposure);
    }
}

// function to execute built-in camera commands
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
        printf("Write of command on file %d has failed; error %d.\n", file, errno);
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
    }
    
    // copy buffer over to returnStr for printing to terminal
    strcpy(returnStr, buffer);
    if (strcmp(command, "fp\r") == 0) {
        printf("%s\n", returnStr);
        // parse the returnStr for new focus range numbers
        sscanf(returnStr, "fp\nOK\nfmin:%d  fmax:%d  current:%i %*s", &all_camera_params.min_focus_pos, 
                                                                      &all_camera_params.max_focus_pos, 
                                                                      &all_camera_params.focus_position);
        printf("in camera params, min focus pos is: %i\n", all_camera_params.min_focus_pos);
        printf("in camera params, max focus pos is: %i\n", all_camera_params.max_focus_pos);
        printf("in camera params, curr focus pos is: %i\n", all_camera_params.focus_position);
        printf("in camera params, prev focus pos was: %i\n", all_camera_params.prev_focus_pos);
        // update previous focus position to current one 
        all_camera_params.prev_focus_pos = all_camera_params.focus_position;
        printf("in camera params, prev focus pos is now: %i\n", all_camera_params.prev_focus_pos);
    } else if (strcmp(command, "pa\r") == 0) {
        printf("%s\n", returnStr);
        // store the current aperture from the returnStr in all_camera_params struct
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