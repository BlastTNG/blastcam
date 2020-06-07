/* HOW TO USE THIS PROGRAM:
** 
** This program is designed so that only two-letter commands need to be input to execute camera changes.
** e.g., cmd_status = runCommand("mi\r", file_descriptor, birger_output);
** where 'mi' is the command.
** 
** All camera settings are listed in the Canon EF 232 user manual.
** https://birger.com/products/lens_controller/manuals/canon_ef232/Canon%20EF-232%20Library%20User%20Manual%201.3.pdf
** 
** COMMANDS:
**  Aperture:
** 		in: initialize aperture motor; aperture will fully open
** 		da: print aperture information
** 		ma: move aperture to absolute position
** 		mc: move aperture to fully stopped down limit
** 		mn: move aperture incremental (mn2 moves the aperture by +2, not to +2)
** 		mo: move aperture to completely open
** 		pa: print aperture position
**  Focus:
** 		eh: set absolute lens focus position (0...0x3FFF)
** 		fa: move focus to absolute position
** 		fd: print focus distance range
** 		ff: fast focus
** 		fp: print the raw focus positions
** 		la: learn the focus range
** 		mf: move focus incremental (mf200 moves the focus by +200, not to +200; mf-200 increments down 200)
** 		mi: move focus to the infinity stop
** 		mz: move focus to the zero stop
** 		pf: print focus position
** 		sf: set the focus counter
**  Misc:
** 		bv: print the bootloader version
** 		de: dump EEPROM
** 		ds: prints distance stops
** 		dz: prints the zoom range
** 		ex: exit to the bootloader
** 		gs: echo current device and lens statuses
** 		hv: print the hardware version
** 		id: print basic lens identification (zoom and f-number)
** 		is: turn image stabilization off/on
** 		ll: library loaded check
** 		lp: lens presence
** 		ls: query lens for status immediately and print
** 		lv: print the library version string
** 		pl: lens power
** 		rm: set response modes
** 		se: temporarily set non-volatile (EEMPROM) byte
** 		sg: set GPIO
** 		sm: set special modes
** 		sn: print the device serial number
** 		sr: set spontaneous responses off/on
** 		vs: print the short version string
** 		we: write non-volatile parameters to EEPROM
*/

// include necessary libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <ueye.h>
#include <math.h>
#include <time.h>

// include our header files
#include "lens_adapter.h"
#include "camera.h"
#include "commands.h"

/* Camera parameters global structure (defined in camera.h) */
struct camera_params all_camera_params = {
    .prev_focus_pos = 0,       // need to save previous focus value to determine by how much we need to move in mf command below
    .focus_position = 0,       // current focus position
    .focus_inf = 0,            // (bool) default is not to set focus to infinity (ideal position is 80 counts below infinity)
    .aperture_steps = 0,       // number of aperture steps to shift by in mn command
    .max_aperture = 0,         // (bool) default is not to maximize aperture, since it will already be maximized from initCamera()
    .current_aperture = 0,     // current aperture position
    .min_focus_pos = 0,        // current min focus position (focus range changes with each power cycle of the camera)
    .max_focus_pos = 0,        // current max focus position
    .exposure_time = 800,      // current exposure time (700 msec is default)
    .change_exposure_bool = 0, // don't want to change the exposure from default value unless user commands it
    .focus_mode = 1,           // camera should begin in auto-focusing mode by default (and then leave it)
    .start_focus_pos = 0,      // starting focus position for auto-focusing search range is set to its default value below in initLensAdapter()
    .end_focus_pos = 0,        // ending focus position also set below
    .focus_step = 0,           // by default, check every fifth focus position
};

// variable for returning from Birger commands
char * birger_output;
// variable to check status after running lens adapter commands (error checking)
int cmd_status;
// variable for runCommand() function below
char * buffer;
// file for lens adapter
int file_descriptor;
// global variables for solution to quadratic regression for auto-focusing
double a, b, c;
// for processing the auto-focusing images
int buffer_num;
char * memory;
char * waiting_mem;
// for saving the auto-focusing images
IMAGE_FILE_PARAMS ImageFileParams;

/* Helper function to print a 1D array.
** Input: The array to be printed.
** Output: None (void). Array is printed to terminal.
*/
void printArray(double * arr, int len) {
    printf("\n");
    for (int i = 0; i < len; i++) {
        printf("%lf\n", arr[i]);
    }
    printf("\n");
}

/* Helper function to print focus and flux data from the auto-focusing file.
** Input: The 1D integer flux and focus arrays.
** Output: None (void). Prints the arrays to the terminal for verification.
*/
void printFluxFocus(int * flux_arr, int * focus_arr, int num_elements) {
    printf("\nFlux and focus position data to compare with auto-focusing file:\n");

    for (int i = 0; i < num_elements; i++) {
        printf("%3d\t%6d\n", flux_arr[i], focus_arr[i]);
    }

    printf("\n");
}

/* Helper function to print matrices row-wise.
** Input: The matrix to be printed to the terminal with its size (m rows x n columns).
** Output: None (void). Prints the matrix to the terminal for verification.
*/
void printMatrix(int m, int n, double matrix[m][n]) {
    for (int row = 0; row < m; row++) {
        printf("[");
        for (int col = 0; col < n; col++) {
            printf("%25lf\t", matrix[row][col]);
        }
        printf("]\n");
    }
}

/* Function to perform Gaussian elimination on the input augmented matrix and solve the 
** system of equations for a, b, c (the elements of the input solution vector, x).
** Input: the number of rows (m), the number of columns (n), the augmented matrix (m x n),
** and a vector to hold the solution (x).
** Output: None (void). Populates the solution vector. 
*/
int gaussianElimination(int m, int n, double A[m][n], double x[m]) {
    // indices
    int i, j, k;

    for (i = 0; i < m - 1; i++) {
        // perform partial pivoting
        for (k = i + 1; k < m; k++) {
            // if the absolute value of the diagonal element is smaller than any of the terms below it,
            // interchange the rows
            if (fabs(A[i][i]) < fabs(A[k][i])) {
                for (j = 0; j < n; j++) {                
                    double temp;
                    temp = A[i][j];
                    A[i][j] = A[k][j];
                    A[k][j] = temp;
                }
            }
        }

        // if after partial pivoting the diagonal element is 0.0, the matrix can't be solved
        if (A[i][i] == 0.0) {
            printf("Mathematical error in matrix.\n");
            return 0;
        }

        // begin Gaussian elimination
        for (k = i + 1; k < m; k++) {
            double multiplier = A[k][i]/A[i][i];
            for (j = 0; j < n;j++){
                A[k][j] = A[k][j] - multiplier*A[i][j];
            }
        }
         
    }

    // begin back substitution
    for (i = m - 1; i >= 0; i--) {
        x[i] = A[i][n-1];
        for (j = i + 1; j < n - 1; j++) {
            x[i] = x[i] - A[i][j]*x[j];
        }
        x[i] = x[i]/A[i][i];
    }   

    return 1;        
}

/* Function to perform quadratic regressions during auto-focusing.
** Input: 1D arrays of flux and focus values, along with their length.
** Output: A flag indicating a successful solution via Gaussian elimination. 
** Calls Gaussian elimination function on the generated system of equations 
** to get a, b, c, the coefficients of the quadratic equation that fits the 
** flux data. 
*/
int quadRegression(int * flux_arr, int * focus_arr, int len) {
    printFluxFocus(flux_arr, focus_arr, len);

    // summation quantities
    double sumfocus;
    double sumflux;
    double sumfocus2;
    double sumfocus3;
    double sumfocus4;
    double sumfluxfocus;
    double sumfocus2flux;
    // vertical vector to hold solution values (gaussianElimination will populate this)
    double solution[3];

    // calculate the quantities for the normal equations
    for (int i = 0; i < len; i++) {
        sumfocus += focus_arr[i];
        sumflux += flux_arr[i];
        sumfocus2 += focus_arr[i]*focus_arr[i];
        sumfocus3 += focus_arr[i]*focus_arr[i]*focus_arr[i];
        sumfocus4 += focus_arr[i]*focus_arr[i]*focus_arr[i]*focus_arr[i];
        sumfluxfocus += focus_arr[i]*flux_arr[i];
        sumfocus2flux += (focus_arr[i]*focus_arr[i])*flux_arr[i];
    }

    // create augmented matrix with this data to be solved
    double augmatrix[3][4] = {{sumfocus4, sumfocus3, sumfocus2, sumfocus2flux},
                              {sumfocus3, sumfocus2, sumfocus,  sumfluxfocus },
                              {sumfocus2, sumfocus,  len,       sumflux      }};
    printf("The original auto-focusing system of equations:\n");
    printMatrix(3, 4, augmatrix);
    
    // perform Gaussian elimination on this matrix
    if (gaussianElimination(3, 4, augmatrix, solution) < 1) {
        printf("Unable to perform Gaussian elimination on the given matrix.\n");
        return 0;
    }

    // print the reduced upper triangular matrix and the solution vector
    printf("\n The upper triangular matrix after Gaussian elimination:\n");
    printMatrix(3, 4, augmatrix);
    printf("\nThe solution vector:");
    printArray(solution, 3);

    // set global variables for coefficients to these values
    a = solution[0];
    b = solution[1];
    c = solution[2];

    return 1;
}

/* Function to initialize the lens adapter and run commands to achieve default settings.
** Input: Path to the file descriptor for the lens.
** Output: Flag indicating successful initialization of the lens.
*/
int initLensAdapter(char * path) {
    struct termios options;

    // open file descriptor with given path
    file_descriptor = open(path, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
    if (file_descriptor < 0) {
        return -1;
    }

    if (tcgetattr(file_descriptor, &options) < 0) {
        printf("Uable to store parameters in termios structure options, error # %d\n", errno);
    }

    // set input speed
    if (cfsetispeed(&options, B115200) < 0) {
        printf("Unable to set input baud rate in termios structure options to desired speed, error # %d\n", errno);
    }

    // set output speed
    if (cfsetospeed(&options, B115200) < 0) {
        printf("Unable to set output baud rate in termios structure options to desired speed, errnor # %d\n", errno);
    }                      

    // standard setting for DSP 1750
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 cha
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
    if (tcsetattr(file_descriptor, TCSANOW, &options) < 0) {
        printf("Unable to apply changes to termios structure options, errnor # %d\n", errno);
    }

    // flush the buffer (in case of unclean shutdown)
    if (tcflush(file_descriptor, TCIOFLUSH) < 0) {
        printf("Buffer fails to flush in add lens.\n");
    }

    // allocate space for returning values after running Birger commands
    birger_output = malloc(100);

    // set focus to 80 below infinity (hard-coded value focus position determined by testing with iDS and uEye software)
    cmd_status = runCommand("la\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to learn current focus range.\n");
    }
    cmd_status = runCommand("mi\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to move focus position to infinity.\n");
    }
    cmd_status = runCommand("mf -80\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to move the focus to the desired default position.\n");
    } else {
        printf("Focus moved to desired default position.\n");
    }

    // print focus position
    printf("Focus at 80 counts below infinity:\n");
    cmd_status = runCommand("fp\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to print the new focus position.\n");
    } 

    // update auto-focusing values now that camera parameters struct is populated
    all_camera_params.start_focus_pos = all_camera_params.focus_position - 100;
    all_camera_params.end_focus_pos = all_camera_params.max_focus_pos - 20;
    all_camera_params.focus_step = 10;

    // set aperture parameter to maximum
    all_camera_params.max_aperture = 1;

    // initialize the aperture motor
    cmd_status = runCommand("in\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to initialize the motor.\n");
    }

    // run the aperture maximization (fully open) command
    cmd_status = runCommand("mo\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Setting the aperture to maximum fails.\n");
    }

    // print aperture position
    cmd_status = runCommand("pa\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to print the new aperture position.\n");
    } 

    // free up birger_output variable
    free(birger_output);
    return file_descriptor;
}

/* Function to auto-focus the lens when the camera starts its run and whenever the user commands.
** Input: None.
** Output: Flag indicating successful completion of the auto-focusing process.
*/
int autofocus(struct tm * tm_info) {
    // variables for changing focus position
    char focus_str_cmd[10];
    int focus_diff;
    int num_focus_pos;
    // auto-focusing file pointer
    FILE * af_file;
    // variable for blob brightness
    int brightest_blob;
    // for naming the focus pictures
    wchar_t image_name[200] = L"";
    char date[256];
    // for reading the auto-focusing file after we've written to it
    char * af_line;
    size_t af_len;
    int af_read;
    int flux, focus;
    int ind;
    // pointers for the star coordinates/magnitudes and the output buffer (image)
    static double * star_x = NULL, * star_y = NULL, * star_mags = NULL;
    static char * output_buffer = NULL;

    // get to beginning of auto-focusing range
    sprintf(focus_str_cmd, "mf %i\r", all_camera_params.start_focus_pos);
    cmd_status = runCommand(focus_str_cmd, file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to move the focus to the beginning of the auto-focusing range.\n");
    } else {
        printf("Focus moved to beginning of auto-focusing range.\n");
    }

    // print the focus to get new focus values and re-populate the camera params struct
    cmd_status = runCommand("fp\r", file_descriptor, birger_output);
    if (cmd_status == -1) {
        printf("Failed to print the new focus position.\n");
    } 

    // open the auto-focusing text file
    af_file = fopen(AUTO_FOCUSING, "w");

    // calculate distance between the end of the auto-focusing range and where the focus is now
    focus_diff = all_camera_params.end_focus_pos - all_camera_params.focus_position;

    // step through each focus position from this position on
    while (focus_diff >= all_camera_params.focus_step) {
        // take three pictures per focus position
        // for (int photo = 0; photo < 3; photo++) {
        //     if (is_FreezeVideo(camera_handle, IS_WAIT) != IS_SUCCESS) {
        //         printf("Error in taking auto-focusing picture %d.\n", photo);
        //         return 0;
        //     }

        //     // get the image from memory
        //     if (is_GetActSeqBuf(camera_handle, &buffer_num, &waiting_mem, &memory) != IS_SUCCESS) {
        //         printf("Error retrieving the active image memory.\n");
        //     }

        //     // find the blobs in the image
        //     int num_blobs = findBlobs(memory, CAMERA_WIDTH, CAMERA_HEIGHT, &star_x, &star_y, &star_mags, output_buffer);

        //     // find the brightest blob per picture
        //     double brightest_blob;
        //     for (int blob = 0; blob < num_blobs; blob++) {
        //         if (star_mags[blob] > brightest_blob) {
        //             brightest_blob = star_mags[blob];
        //         }
        //     }

        //     // make kst display the filtered image 
        //     memcpy(memory, output_buffer, CAMERA_WIDTH*CAMERA_HEIGHT); 

        //     // pointer for transmitting to user should point to where image is in memory
        //     camera_raw = output_buffer;

        //     // name picture accordingly and save
        //     strftime(date, sizeof(date), "%Y-%m-%d_%H:%M:%S", tm_info);
        //     swprintf(image_name, 200, L"/home/xscblast/Desktop/blastcam/BMPs/focus_%d_image%d_%s", 
        //              all_camera_params.focus_position, photo, date);
    	//     ImageFileParams.pwchFileName = image_name;
    	//     if (is_ImageFile(camera_handle, IS_IMAGE_FILE_CMD_SAVE, (void *) &ImageFileParams, sizeof(ImageFileParams)) == -1) {
        //         char * last_error_str;
        //         int last_err = 0;
            
        //         is_GetError(camera_handle, &last_err, &last_error_str);
        //         printf("Failed to save image, error string: %s\n", last_error_str);
        //         exit(2);
    	//     }
        }

        // find the brightest of the three brightest blobs and write it to text file with its corresponding focus
        brightest_blob = -((all_camera_params.focus_position*all_camera_params.focus_position) - 2*all_camera_params.focus_position);
        fprintf(af_file, "%3d\t%5d\n", brightest_blob, all_camera_params.focus_position);

        // shift to next focus position according to desired step size
        sprintf(focus_str_cmd, "mf %i\r", all_camera_params.focus_step);
        cmd_status = runCommand(focus_str_cmd, file_descriptor, birger_output);
        if (cmd_status == -1) {
            printf("Failed to move the focus to the next focus position in auto-focusing range.\n");
            return 0;
        } else {
            printf("Focus moved to next focus position in auto-focusing range.\n");
        }

        // print the focus to get new focus values
        cmd_status = runCommand("fp\r", file_descriptor, birger_output);
        if (cmd_status == -1) {
            printf("Failed to print the new focus position.\n");
        } 

        // keep track of how many focus positions we have stepped through thus far
        num_focus_pos++;

        // update difference between maximum and current focus position
        focus_diff = all_camera_params.end_focus_pos - all_camera_params.focus_position;
    }

    // do the process one last time for the final (current) focus position
    brightest_blob = -((all_camera_params.focus_position*all_camera_params.focus_position) - 2*all_camera_params.focus_position);
    fprintf(af_file, "%3d\t%5d\n", brightest_blob, all_camera_params.focus_position);

    // flush and close the auto-focusing file
    fflush(af_file);
    fclose(af_file);
    
    // now read in data from auto-focusing file (re-open in reading mode)
    af_file = fopen(AUTO_FOCUSING, "r");
    if (af_file == NULL) {
        fprintf(stderr, "Could not open observing file: %s.\n", strerror(errno));
        return 0;
    }

    // we can only initialize these arrays for flux and focus once we know the exact length (number of focus positions + 1)
    int flux_y[num_focus_pos + 1];
    int focus_x[num_focus_pos + 1];
    // read every line in the files
    printf("\n");
    while ((af_read = getline(&af_line, &af_len, af_file)) != -1) {
        sscanf(af_line, "%d\t%d\n", &flux, &focus);
        printf("Auto-focusing data read in: %3d\t%5d\n", flux, focus);
        flux_y[ind] = flux;
        focus_x[ind] = focus;
        ind++;
    }
    fflush(af_file);
    fclose(af_file);

    // perform quadratic regression on this data to find best fit curve
    if (quadRegression(flux_y, focus_x, num_focus_pos + 1) < 1) {
        printf("Unable to perform quadratic regression on auto-focusing data.\n");
        return 0;
    }

    // print the results of this regression
    printf("Best-fit equation for auto-focusing data is: flux = %.3f*x^2 + %.3f*x + %.3f, where x = focus.\n", a, b, c);

    // find the maximum of this curve and its corresponding x (focus) coordinate:
    // the derivative yprime = 2ax + b -> set this to 0 and solve.
    // 2ax + b = 0 -> 2ax = -b -> x = -b/2a.
    double focus_max = -b/(2*a);
    double flux_max = a*(focus_max*focus_max) + b*focus_max + c;
    // take second derivative to ensure this is a maximum
    double ydoubleprime = 2*a;
    if (ydoubleprime < 0) {
        // if the second derivative at the maximum is negative, it is truly a maximum
        printf("The largest flux found is %.3f.\n", flux_max);
        printf("Focus position corresponding to maximum brightness is %.3f. The nearest integer to this value is %.3f.\n", 
                focus_max, round(focus_max));
        return round(focus_max);
    } else {
        // did not successfully find the max
        return 0;
    }
}

/* Function to process and execute user commands for the camera and lens settings. Note: does not
** include adjustments to the blob-finding parameters and image processing; this is done directly
** in commands.c in the client handler function.
** Input: None.
** Output: None (void). Executes the commands and re-populates the camera params struct with the 
** updated values.
*/
void adjustCameraHardware() {
    // local variables for executing commands
    char focus_str_cmd[10]; 
    char aper_str_cmd[4]; 
    double current_exposure;
    // for calculating desired change in focus for user
    int focus_shift;

    // if user set focus infinity command to true (1), execute this command and none of the other focus commands 
    // that would contradict this one
    if (all_camera_params.focus_inf == 1) {
        cmd_status = runCommand("mi\r", file_descriptor, birger_output);
        if (cmd_status == -1) {
            printf("Failed to set focus to infinity.\n");
        } else {
            printf("Focus set to infinity.\n");
        }

        cmd_status = runCommand("fp\r", file_descriptor, birger_output);
        if (cmd_status == -1) {
            printf("Failed to print focus after setting to infinity.\n");
        } 
    } else {
        // calculate the shift needed to get from current focus position to user-specified position
        focus_shift = all_camera_params.focus_position - all_camera_params.prev_focus_pos;
        printf("Focus change (internal calculation, not command): %i\n", focus_shift);

        if (focus_shift != 0) {
            sprintf(focus_str_cmd, "mf %i\r", focus_shift);

            // shift the focus 
            cmd_status = runCommand(focus_str_cmd, file_descriptor, birger_output);
            if (cmd_status == -1) {
                printf("Failed to move the focus to the desired position.\n");
            } else {
                printf("Focus moved to desired absolute position.\n");
            }

            // print focus position for confirmation
            cmd_status = runCommand("fp\r", file_descriptor, birger_output);
            if (cmd_status == -1) {
                printf("Failed to print the new focus position.\n");
            }  
        }
    }

    // if the user wants to set the aperture to the maximum
    if (all_camera_params.max_aperture == 1) {
        // might as well change struct field here since we know what the maximum aperture position is
        // (don't have to get it with pa command)
        all_camera_params.current_aperture = 28;

        cmd_status = runCommand("mo\r", file_descriptor, birger_output); 
        if (cmd_status == -1) {
            printf("Setting the aperture to maximum fails.\n");
        } else {
            printf("Set aperture to maximum.\n");
        }
    } else {
        if (all_camera_params.aperture_steps != 0) {
            sprintf(aper_str_cmd, "mn%i\r", all_camera_params.aperture_steps);

            // perform the aperture command
            cmd_status = runCommand(aper_str_cmd, file_descriptor, birger_output);
            if (cmd_status == -1) {
                printf("Failed to adjust the aperture.\n");
            } else {
                printf("Adjusted the aperture successfully.\n");
            }

            // print new aperture position
            cmd_status = runCommand("pa\r", file_descriptor, birger_output);
            if (cmd_status == -1) {
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
        if (is_Exposure(camera_handle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void *) &all_camera_params.exposure_time, 
                        sizeof(double)) != IS_SUCCESS) {
            printf("Adjusting exposure to user command unsuccessful.\n");
        }

        // check with current_exposure that exposure has been adjusted to desired value
        if (is_Exposure(camera_handle, IS_EXPOSURE_CMD_GET_EXPOSURE, &current_exposure, sizeof(double)) != IS_SUCCESS) {
            printf("Could not check current exposure value.\n");
        }

        printf("Exposure is now %f msec.\n", current_exposure);
    }
}

/* Function to execute built-in Birger commands.
** Input: The string identifier for the command, the file descriptor for the lens
** adapter, and a string to print the Birger output to for verification.
** Output: Flag indicating successful execution of the command.
*/
int runCommand(const char * command, int file, char * return_str) {
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
    strcpy(return_str, buffer);
    if (strcmp(command, "fp\r") == 0) {
        printf("%s\n", return_str);

        // parse the return_str for new focus range numbers
        sscanf(return_str, "fp\nOK\nfmin:%d  fmax:%d  current:%i %*s", &all_camera_params.min_focus_pos, 
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
        printf("%s\n", return_str);

        // store the current aperture from the return_str in all_camera_params struct
        if (strncmp(return_str, "pa\nOK\nDONE", 10) == 0) {
            sscanf(return_str, "pa\nOK\nDONE%*i,f%d", &all_camera_params.current_aperture);
        } else if (strncmp(return_str, "pa\nOK\n", 6) == 0) {
            sscanf(return_str, "pa\nOK\n%*i,f%d %*s", &all_camera_params.current_aperture);
        }

        printf("in camera params, curr aper is: %i\n", all_camera_params.current_aperture);
    } else if (strncmp(command, "mf", 2) == 0) {
        printf("%s\n", return_str);
    }

    free(buffer);
    return 0;
}