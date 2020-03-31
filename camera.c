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
#include <limits.h>
#include <float.h>
#include <inttypes.h>

#include "camera.h"
#include "astrometry.h"
#include "commands.h"
#include "lens_adapter.h"

// define some functions for sorting blobs later
void merge(double A[], int p, int q, int r, double X[],double Y[]);
void part(double A[], int p, int r, double X[], double Y[]);

#define TRUE  1
#define FALSE 0

HIDS cameraHandle = 12;          // set by ueyesetid
IS_POINT_2D locationRectangle;
IMAGE_FILE_PARAMS ImageFileParams;
SENSORINFO sensorInfo;
FILE * fptr;

// global variables
int bufferNumber = 0;
char * memory = NULL;
char * waitingMem = NULL;
int status;                        // variable to verify completion of methods
extern int shutting_down;
unsigned char * mask = NULL;
int first_run = 1;                 // for file set-up purposes
char * memoryStartingPointer;
int memoryId;
// for writing camera settings to observing file
double currentExposure;
int currPixelClock;
double actualFPS;
double currAutoGain;
double currShutter;
double actualAutoFR;
int currColorMode;
int currExtTrig;
int currTrigDelay;
int currMasterGain;
int currRedGain;
int currGreenGain;
int currBlueGain;
int currGamma;
int currGainBoost;
unsigned int currTimeout;
int actualBLOffset;
int actualBLMode;

// initialize global struct for blob parameters with default values
struct blob_params all_blob_params = {
    .spike_limit = 4,              // how agressive is the dynamic hot pixel finder.  Small is more agressive
    .dynamic_hot_pixels = 0,       // 0 == off, 1 == on
    .r_smooth = 2,                 // image smooth filter radius [px]
    .high_pass_filter = 0,         // 0 = off, 1 = on
    .r_high_pass_filter = 10,      // image high pass filter radius [px]
    .centroid_search_border = 1,   // distance from image edge from which to start looking for stars [px]
    .filter_return_image = 0,      // 1 = true; 0 = false
    .n_sigma = 2.0,                // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
    .unique_star_spacing = 15,     // minimum spacing between two unique stars [px]
    .make_static_hp_mask = 0,     // make the mask (once, value should be 20), set to 0 when finished with test pictures
    .use_static_hp_mask = 0,       // always use the hot pixel mask 
};

// function to determine if the current year is a leap year (2020 is a leap year)
int isLeapYear(int year) {
    if (year % 4 != 0) {
        return false;
    } else if (year % 400 == 0) {
        return true;
    } else if (year % 100 == 0) {
        return false;
    } else {
        return true;
    }
}

void verifyBlobParams() {
    printf("in camera.c, all_blob_params.spike_limit is: %d\n", all_blob_params.spike_limit);
    printf("in camera.c, all_blob_params.dynamic_hot_pixels is: %d\n", all_blob_params.dynamic_hot_pixels);
    printf("in camera.c, all_blob_params.centroid_search_border is: %d\n", all_blob_params.centroid_search_border);
    printf("in camera.c, all_blob_params.high_pass_filter is: %d\n", all_blob_params.high_pass_filter);
    printf("in camera.c, all_blob_params.r_smooth is: %d\n", all_blob_params.r_smooth);
    printf("in camera.c, all_blob_params.filter_return_image is: %d\n", all_blob_params.filter_return_image);
    printf("in camera.c, all_blob_params.r_high_pass_filter is: %d\n", all_blob_params.r_high_pass_filter);
    printf("in camera.c, all_blob_params.n_sigma is: %f\n", all_blob_params.n_sigma);
    printf("in camera.c, all_blob_params.unique_star_spacing is: %d\n", all_blob_params.unique_star_spacing);
}

// initialize the camera
void init_camera() {
    // local variables for exposure parameters
    double min_exposure;
    double max_exposure;
    double exposure_step_size;

    // flag to enable long exposures
    unsigned int enable = 1;   

    // load the camera parameters
    load_camera();  

    // set exposure time based on struct field
    status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void *) &all_camera_params.exposure_time, sizeof(double));
    // enable long exposure 
    status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_LONG_EXPOSURE_ENABLE, (void *) &enable, sizeof(unsigned int));
    // get current exposure, max possible exposure (with long exposure enabled), and min exposure
    status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, &min_exposure, sizeof(double));
    status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_GET_EXPOSURE, &currentExposure, sizeof(double));
    status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, &max_exposure, sizeof(double));
  
	if (status != IS_SUCCESS) {
		printf("Setting the exposure time fails.\n");
	} else {
        printf("Current exposure time: %f msec | Min possible exposure: %f msec | Max possible exposure: %f msec\n", 
                currentExposure, min_exposure, max_exposure);
    }

    // initializes astrometry
    init_astrometry();

    // sets save image params
	saveImage();
}

// cleaning_up function for ctrl+c exception
void clean_up() {
    printf("\ncommands.c terminated, performing clean-up.\n");
    shutting_down = 1;
    is_FreeImageMem(cameraHandle, memoryStartingPointer, memoryId);
    is_ExitCamera(cameraHandle);
    exit(0);
}

// sets starting values for certain camera atributes
void set_camera_params(unsigned int cameraHandle) {
    // set the trigger delay to 0 (microseconds) -- deactivates trigger delay
	is_SetTriggerDelay(cameraHandle, 0); 
    // check the trigger delay is 0
    currTrigDelay = is_SetTriggerDelay(cameraHandle, IS_GET_TRIGGER_DELAY); 
    printf("Trigger delay (should be 0): %i\n", currTrigDelay);

    // set camera integrated amplifier 
    status = is_SetHardwareGain(cameraHandle, 0, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
    if (status != IS_SUCCESS) {
        printf("Setting gain failed.\n");
    }
    // check color gains
    currMasterGain = is_SetHardwareGain(cameraHandle, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
    currRedGain = is_SetHardwareGain(cameraHandle, IS_IGNORE_PARAMETER, IS_GET_RED_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
    currGreenGain = is_SetHardwareGain(cameraHandle, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_GET_GREEN_GAIN, IS_IGNORE_PARAMETER);
    currBlueGain = is_SetHardwareGain(cameraHandle, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_GET_BLUE_GAIN);
    printf("Gain settings: %i for master gain, %i for red gain, %i for green gain, and %i for blue gain.\n", 
            currMasterGain, currRedGain, currGreenGain, currBlueGain);

    // disable camera's auto gain (0 = disabled)
    double autoGain = 0;
    status = is_SetAutoParameter(cameraHandle, IS_SET_ENABLE_AUTO_GAIN, &autoGain, NULL);
    if (status != IS_SUCCESS) {
        printf("Disabling auto gain fails.\n");
    }
    // check auto gain is off
    is_SetAutoParameter(cameraHandle, IS_GET_ENABLE_AUTO_GAIN, &currAutoGain, NULL);
    printf("Auto gain (should be disabled): %f\n", currAutoGain);

    // set camera's gamma value to off 
    status = is_SetHardwareGamma(cameraHandle, IS_SET_HW_GAMMA_OFF);
    if (status != IS_SUCCESS) {
        printf("Disabling hardware gamma corection fails.\n");
    }
    // check hardware gamma is off
    currGamma = is_SetHardwareGamma(cameraHandle, IS_GET_HW_GAMMA);
    printf("Hardware gamma (should be off): %i\n", currGamma);

    // disable auto shutter
    double autoShutter = 0;
    status = is_SetAutoParameter(cameraHandle, IS_SET_ENABLE_AUTO_SHUTTER, &autoShutter, NULL);
    if (status != IS_SUCCESS) {
        printf("Disabling auto shutter speed fails.\n");
    }
    // check auto shutter is off
    is_SetAutoParameter(cameraHandle, IS_GET_ENABLE_AUTO_SHUTTER, &currShutter, NULL);
    printf("Auto shutter (should be off): %.1f\n", currShutter);

    // disable auto frame rate
    double autoFramerate = 0;
    status = is_SetAutoParameter(cameraHandle, IS_SET_ENABLE_AUTO_FRAMERATE, &autoFramerate, NULL);
    if (status != IS_SUCCESS) {
        printf("Disabling auto framerate fails.\n");
    }
    // check auto frame rate is off
    is_SetAutoParameter(cameraHandle, IS_GET_ENABLE_AUTO_FRAMERATE, &actualAutoFR, NULL);
    printf("Auto frame rate (should be off): %.1f\n", actualAutoFR);

    // disable gain boost to reduce noise
    status = is_SetGainBoost(cameraHandle, IS_SET_GAINBOOST_OFF);
    if (status != IS_SUCCESS) {
        printf("Disabling gain boost fails.\n");
    }
    // check gain boost is off
    currGainBoost = is_SetGainBoost(cameraHandle, IS_GET_GAINBOOST);
    printf("Gain boost (should be disabled): %i\n", currGainBoost);	

    // turn off auto black level
    int blackLevelMode = IS_AUTO_BLACKLEVEL_OFF;
    status = is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_SET_MODE, (void *) &blackLevelMode, sizeof(blackLevelMode));
    if (status != IS_SUCCESS) {
        printf("Turning off auto black level mode fails, status %d.\n", status);
    }
    // check auto black level is off
    actualBLMode = is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_GET_MODE, (void *) &actualBLMode, sizeof(actualBLMode));
    printf("Auto black level (should be off): %i\n", actualBLMode);

    // set black level offset to 50
    int blackLevelOffset = 50;
    status = is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_SET_OFFSET, (void *) &blackLevelOffset, sizeof(blackLevelOffset));
    if (status != IS_SUCCESS) {
        printf("Setting black level offset to 50 fails, status %d.\n", status);
    }
    // check black level offset
    is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_GET_OFFSET, (void *) &actualBLOffset, sizeof(actualBLOffset));
    printf("Black level offset (desired is 50): %i\n", actualBLOffset);

    // This for some reason actually affects the time it takes to set aois only on the focal plane camera. 
    // Don't use if for the trigger timeout. It is also not in milliseconds.
    status = is_SetTimeout(cameraHandle, IS_TRIGGER_TIMEOUT, 500);
    if (status != IS_SUCCESS) {
        printf("Setting trigger timeout fails with status %d.\n", status);
    }
    // check time out
    is_GetTimeout(cameraHandle, IS_TRIGGER_TIMEOUT, &currTimeout);
    printf("Current trigger timeout: %i\n", currTimeout);
}

// load the camera
int load_camera() {
    // local variable for verifying functions executed correctly or not
    int status;	

    // initialize camera
	if ((status = is_InitCamera(&cameraHandle, NULL)) != IS_SUCCESS){
        printf("Camera initialization failed, error number: %d\n", errno);
        exit(2);
	}
  
    // get sensor info
	if ((status = is_GetSensorInfo(cameraHandle, &sensorInfo)) != IS_SUCCESS) {
        printf("Failed to receive camera sensor information.\n");
        exit(2);
	} 

    // set various other camera parameters
	set_camera_params(cameraHandle);

    // set display mode and then get it to verify
	if ((status = is_SetColorMode(cameraHandle, IS_CM_SENSOR_RAW8)) != IS_SUCCESS) {
        printf("Setting display mode failed.\n");
        exit(2);
	}
    currColorMode = is_SetColorMode(cameraHandle, IS_GET_COLOR_MODE);
    printf("Camera model: %s\n", sensorInfo.strSensorName);
    printf("Sensor ID/type: %i\n", sensorInfo.SensorID);
    printf("Sensor color mode (from is_GetSensorInfo and is_SetColorMode): %i / %i\n", sensorInfo.nColorMode, currColorMode);
    printf("Maximum image width and height: %i, %i\n", sensorInfo.nMaxWidth, sensorInfo.nMaxHeight);
    printf("Pixel size (micrometers): %hu\n", sensorInfo.wPixelSize);
 	
    // allocate camera memory
	int colourDepth = 8; 
	if ((status = is_AllocImageMem(cameraHandle, sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, colourDepth, &memoryStartingPointer, &memoryId)) != IS_SUCCESS) {
		printf("Allocating camera memory failed.\n");
        exit(2);
	}

    // set memory for image (make memory pointer active)
	if ((status = is_SetImageMem(cameraHandle, memoryStartingPointer, memoryId)) != IS_SUCCESS) {
        printf("Setting memory to active failed.\n");
        exit(2);
    }

    // get image memory
	void * activeMemoryLocation;
	if ((status = is_GetImageMem(cameraHandle, &activeMemoryLocation)) != IS_SUCCESS) {
        printf("Getting image memory failed.\n");
        exit(2);
    }
    printf("Image memory from activeMemoryLocation: %p | Image memory from memoryStartingPointer: %p (these should be the same)\n", 
            (void *) &activeMemoryLocation, (void *) &memoryStartingPointer);

    // how clear images can be is affected by pixelclock and fps 
    int pixelclock = 30;
	if ((status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*) &pixelclock, sizeof(pixelclock))) != IS_SUCCESS) {
        printf("Pixel clock failed to set\n");
        exit(2);
    }
    // get current pixel clock to check
    is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_GET, (void *) &currPixelClock, sizeof(currPixelClock));
    printf("Pixel clock: %i\n", currPixelClock);

    // set frame rate
	double newFPS = 10;
	if ((status = is_SetFrameRate(cameraHandle, IS_GET_FRAMERATE, (void*) &newFPS)) != IS_SUCCESS) {
        printf("Frame rate failed to set.\n");
        exit(2);
    }

    // set trigger to software mode (call is_FreezeVideo to take a single picture in single frame mode)
	if ((status = is_SetExternalTrigger(cameraHandle, IS_SET_TRIGGER_SOFTWARE)) != IS_SUCCESS) {
        printf("Trigger failed to set to desired mode.\n");
        exit(2);
	}
    // get the current trigger setting
    currExtTrig = is_SetExternalTrigger(cameraHandle, IS_GET_EXTERNALTRIGGER);
    printf("Current external trigger mode: %i\n", currExtTrig);

    // confirmation message that camera is initialized
    printf("Done initializing camera.\n");
}

// save image as bmp 
int saveImage() {
    ImageFileParams.pwchFileName = L"save1.bmp";
    ImageFileParams.pnImageID = NULL;
    ImageFileParams.ppcImageMem = NULL;
    ImageFileParams.nQuality = 80;
    ImageFileParams.nFileType = IS_IMG_BMP;
}

void makeMask(char * ib, int i0, int j0, int i1, int j1, int x0, int y0, bool subframe) {
    static int first_time = 1;
    // Declare static hot pixel coordinates
    static int * x_p = NULL;
    static int * y_p = NULL;
    // number of pixels stored
    static int num_p = 0;
    // number of pixels allocated
    static int num_alloc = 0;

    if (first_time) {
        mask = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, 1);

        // // read from file with bright pixel coordinates in it
        // FILE * f = fopen(STATIC_HP_MASK, "r");
        // int i, j;

        // x_p = calloc(100, sizeof(int));
        // y_p = calloc(100, sizeof(int));
        // num_alloc = 100;

        // if (f) {
        //     printf("*---------------------- Loading static hot pixel map... ----------------------*\n");
        //     char * line = NULL;
        //     size_t len = 0;
        //     int read = 0;

        //     // read every line in the file
        //     while ((read = getline(&line, &len, f)) != -1) {
        //         sscanf(line, "%d,%d\n", &i, &j);

        //         if (num_p >= num_alloc) {
        //             num_p += 100;
        //             x_p = realloc(x_p, sizeof(int) * num_alloc);
        //             y_p = realloc(y_p, sizeof(int) * num_alloc);
        //         }

        //         x_p[num_p] = i;
        //         y_p[num_p] = CAMERA_HEIGHT - j; // map y coordinate to image in memory from Kst blob

        //         printf("Coordinates read from hp file: [%i, %i]\n", i, j);

        //         num_p++;
        //     }
        //     fflush(f);
        //     fclose(f);
        // }

        first_time = 0;
    }

    int i, j;
    int p0, p1, p2, p3, p4;
    int a, b;
    
    int cutoff = all_blob_params.spike_limit*100.0;

    for (i = i0; i < i1; i++) {
        mask[i + CAMERA_WIDTH*j0] = mask[i + (j1-1)*CAMERA_WIDTH] = 0;
    }

    for (j = j0; j < j1; j++) {
        mask[i0 + j*CAMERA_WIDTH] = mask[i1-1 + j*CAMERA_WIDTH] = 0;
    }

    i0++;
    j0++;
    i1--;
    j1--;
  
    if (all_blob_params.dynamic_hot_pixels) {
        int nhp = 0;
        for (j = j0; j < j1; j++) {
            for (i = i0; i < i1; i++) {
                p0 = 100*ib[i + j*CAMERA_WIDTH]/cutoff;
                p1 = ib[i - 1 + (j)*CAMERA_WIDTH];
                p2 = ib[i + 1 + (j)*CAMERA_WIDTH];
                p3 = ib[i + (j+1)*CAMERA_WIDTH];
                p4 = ib[i + (j-1)*CAMERA_WIDTH];
                a = p1 + p2 + p3 + p4 + 4;
                p1 = ib[i - 1 + (j-1)*CAMERA_WIDTH];
                p2 = ib[i + 1 + (j+1)*CAMERA_WIDTH];
                p3 = ib[i - 1 + (j+1)*CAMERA_WIDTH];
                p4 = ib[i + 1 + (j-1)*CAMERA_WIDTH];
                b = p1 + p2 + p3 + p4 + 4;
                mask[i + j*CAMERA_WIDTH] = ((p0 < a) && (p0 < b));
                if (p0 > a || p0 > b) nhp++;
            }
        }
    } else {
        for (j = j0; j < j1; j++) {
            for (i = i0; i < i1; i++) {
                mask[i + j*CAMERA_WIDTH] = 1;
            }
        }
    }

    if (all_blob_params.use_static_hp_mask) {
        printf("*---------------------- Masking %d pixels... ----------------------*\n", num_p);
        for (int i = 0; i < num_p; i++) {
            // set all static hot pixels to 0
            printf("Coordinates going into mask index: [%i, %i]\n", x_p[i], CAMERA_HEIGHT - y_p[i]);
            int ind = CAMERA_WIDTH * y_p[i] + x_p[i];
            mask[ind] = 0;
        }
    }
/*
    // TODO: decide if we want a static hot pixel map
    if (use_hpList) {
        int ix, iy;
        int n = hpList.size();
        if (subframe) {
            for (i=0; i<n; i++) {
                ix = hpList[i].x - x0;
                iy = hpList[i].y-y0;
                if ((ix>=0) && (iy >=0) && (ix < i1) && (ix < j1)) {
                    mask[ix + CAMERA_WIDTH * (iy)] = 0;
                }
            }
        } else {
            for (i=0; i<n; i++) {
                mask[hpList[i].x - x0 + CAMERA_WIDTH * (hpList[i].y-y0)] = 0;
            }
        }
    }
*/
}

void boxcarFilterImage(char * ib, int i0, int j0, int i1, int j1, int r_f, double * filtered_image) {
    static int first_time = 1;
    static char * nc = NULL;
    static uint64_t * ibc1 = NULL;

    if (first_time) {
        nc = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, 1);
        ibc1 = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, sizeof(uint64_t));
        first_time = 0;
    }

    int b = r_f;
    int64_t isx;
    int s, n;
    double ds, dn;
    double last_ds = 0;

    for (int j = j0; j < j1; j++) {
        n = 0;
        isx = 0;
        for (int i = i0; i < i0 + 2*r_f + 1; i++) {
            n += mask[i + j*CAMERA_WIDTH];
            isx += ib[i + j*CAMERA_WIDTH]*mask[i + j*CAMERA_WIDTH];
        }

        int idx = CAMERA_WIDTH*j + i0 + r_f;

        for (int i = r_f + i0; i < i1 - r_f - 1; i++) {
            ibc1[idx] = isx;
            nc[idx] = n;
            isx = isx + mask[idx + r_f + 1]*ib[idx + r_f + 1] - mask[idx - r_f]*ib[idx - r_f];
            n = n + mask[idx + r_f + 1] - mask[idx - r_f];
            idx++;
        }

        ibc1[idx] = isx;
        nc[idx] = n;
    }

    for (int j = j0+b; j < j1-b; j++) {
        for (int i = i0+b; i < i1-b; i++) {
            n = s = 0;
            for (int jp =- r_f; jp <= r_f; jp++) {
                int idx = i + (j+jp)*CAMERA_WIDTH;
                s += ibc1[idx];
                n += nc[idx];
            }
            ds = s;
            dn = n;
            if (dn > 0.0) {
                ds /= dn;
                last_ds = ds;
            } else {
                ds = last_ds;
            }
            filtered_image[i + j*CAMERA_WIDTH] = ds;
        }
    }
}

int get_blobs(char * input_buffer, int w, int h, double ** starX, double ** starY, double ** starMag, char * output_buffer, char * filename) {
    static int first_time = 1;
    static double * ic = NULL;
    static double * ic2 = NULL;
    static int num_blobs_alloc = 0;

	// create timer
	int msec = 0, trigger = 60000; // ms
	clock_t before = clock();
 	clock_t difference = clock() - before;
	msec = difference  * 1000/CLOCKS_PER_SEC;
 	int diff = trigger-msec;

    difference = clock() - before;
    msec = difference * 1000/CLOCKS_PER_SEC;

    // allocate the proper amount of storage space to start
    if (first_time) {
        ic = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, sizeof(double));
        ic2 = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, sizeof(double));
        first_time = 0;
    }
  
    // we use half-width internally, but the api gives us full width.
    int x_size = w/2;
    int y_size = h/2;

    int j0, j1, i0, i1;
    int b = 0; // extra image border

    j0 = i0 = 0;
    j1 = h;
    i1 = w;
    
    b = all_blob_params.centroid_search_border;

    // if we want to make a new hot pixel mask
    if (all_blob_params.make_static_hp_mask) {
        printf("*---------------------- Making static hot pixel map... ----------------------*\n");
        // make a file and write bright pixel coordinates to it
        FILE * f = fopen(STATIC_HP_MASK, "a"); // change file opening mode to "w" when not testing with dark images
        fprintf(f, "Image file: %s\n", filename); // comment out when not taking dark images
        for (int j = 0; j < CAMERA_HEIGHT; j++) {
            for (int i = 0; i < CAMERA_WIDTH; i++) {
                // index in the array where we are
                int ind = CAMERA_WIDTH*j + i; 
                // check pixel value to see if it's above hot pixel threshold
                if (input_buffer[ind] > all_blob_params.make_static_hp_mask) {
                    int x_p = i;
                    int y_p = CAMERA_HEIGHT - j; // make this agree with blob coordinates in Kst
                    fprintf(f, "%d,%d\n", x_p, y_p);
                    printf("Value of pixel (should be > %i): %d | Coordinates: [%d, %d]\n", all_blob_params.make_static_hp_mask, 
                                                                                            input_buffer[ind], x_p, y_p);
                }
            }
        }
        fflush(f);
        fclose(f);
        // do not want to recreate hp mask automatically, so set field to 0
        all_blob_params.make_static_hp_mask = 0;
    }

    makeMask(input_buffer, i0, j0, i1, j1, 0, 0, 0);

    double sx = 0, sx2 = 0;
    double sx_raw = 0; // sum of non-highpass filtered field
    int num_pix = 0;

    // lowpass filter the image - reduce noise.
    boxcarFilterImage(input_buffer, i0, j0, i1, j1, all_blob_params.r_smooth, ic);

    if (all_blob_params.high_pass_filter) { // only high-pass filter full frames
        b += all_blob_params.r_high_pass_filter;
        boxcarFilterImage(input_buffer, i0, j0, i1, j1, all_blob_params.r_high_pass_filter, ic2);

        for (int j = j0+b; j < j1-b; j++) {
            for (int i = i0+b; i < i1-b; i++) {
                int idx = i + j*w;
                sx_raw += ic[idx]*mask[idx];
                ic[idx] -= ic2[idx];
                sx += ic[idx]*mask[idx];
                sx2 += ic[idx]*ic[idx]*mask[idx];
                num_pix += mask[idx];
            }
        }
    } else {
        for (int j = j0+b; j < j1-b; j++) {
            for (int i = i0+b; i < i1-b; i++) {
                int idx = i + j*w;
                sx += ic[idx]*mask[idx];
                sx2 += ic[idx]*ic[idx]*mask[idx];
                num_pix += mask[idx];
            }
        }
        sx_raw = sx;
    }

    double mean = sx/num_pix;
    double mean_raw = sx_raw/num_pix;
    double sigma = sqrt((sx2 - sx*sx/num_pix)/num_pix);

    // fill output buffer if the variable is defined 
    if (output_buffer) {
        int pixel_offset = 0;
        if (all_blob_params.high_pass_filter) pixel_offset = 50;

        if (all_blob_params.filter_return_image) {
            for (int j = j0 + 1; j < j1 - 1; j++) {
                for (int i = i0 + 1; i < i1 - 1; i++) {
                  output_buffer[i + j*w] = ic[i + j*w]+pixel_offset;
                }
            }
            for (int j = 0; j < b; j++) {
                for (int i = i0; i < i1; i++) {
                  output_buffer[i + (j + j0)*w] = output_buffer[i + (j1 - j - 1)*w] = mean + pixel_offset;
                }
            }
            for (int j = j0; j < j1; j++) {
                for (int i = 0; i < b; i++) {
                  output_buffer[i + i0 + j*w] = output_buffer[i1 - i - 1 + j*w] = mean + pixel_offset;
                }
            }
        } else {
            for (int j = j0; j < j1; j++) {
                for (int i = i0; i < i1; i++) {
                  int idx = i + j*w;
                  output_buffer[idx] = input_buffer[idx];
                }
            }
        }
    }

    // Find the blobs 
    double ic0;
    int blob_count = 0;
    for (int j = j0+b; j < j1-b-1; j++) {
        for (int i = i0+b; i < i1-b-1; i++) {
        	difference = clock() - before;
            msec = difference  * 1000/ CLOCKS_PER_SEC;		      
            diff = trigger - msec;
        	if (diff < 0) {
        	    printf("Blob-finder timed out!\n");
        	    return 0;
        	}

            // if pixel exceeds threshold
            if (((double) ic[i + j*w] > mean + all_blob_params.n_sigma*sigma) || ((double) ic[i + j*w] > 254) || 
            /* this is pretty arbitrary. its purely based on the normal value for dark sky being 6 or 7 -->*/ ((double) ic[i + j*w] > 7)) {
                ic0 = ic[i + j*w];
                // If pixel is a local maximum or saturated
                if (((ic0 >= ic[i-1 + (j-1)*w]) &&
                     (ic0 >= ic[i   + (j-1)*w]) &&
                     (ic0 >= ic[i+1 + (j-1)*w]) &&
                     (ic0 >= ic[i-1 + (j  )*w]) &&
                     (ic0 >  ic[i+1 + (j  )*w]) &&
                     (ic0 >  ic[i-1 + (j+1)*w]) &&
                     (ic0 >  ic[i   + (j+1)*w]) &&
                     (ic0 >  ic[i+1 + (j+1)*w])) ||
                    (ic0 > 254)){
                      
                    int unique = 1;

                    // realloc array if necessary (when the camera looks at a really bright image, this slows everything down severely)
                    if (blob_count >= num_blobs_alloc) {
                        num_blobs_alloc += 500;
                        *starX = realloc(*starX, sizeof(double)*num_blobs_alloc);
                        *starY = realloc(*starY, sizeof(double)*num_blobs_alloc);
                        *starMag = realloc(*starMag, sizeof(double)*num_blobs_alloc);
                     }

                    (*starX)[blob_count] = i;
                    (*starY)[blob_count] = j;
                    (*starMag)[blob_count] = 100*ic[i + j*CAMERA_WIDTH];

                    // FIXME: not sure why this is necessary..
                    if((*starMag)[blob_count] < 0) {
                        (*starMag)[blob_count] = UINT32_MAX;
                    }

                    // If we already found a blob within SPACING and this one is bigger, replace it.
                    int spacing = all_blob_params.unique_star_spacing;
                    if ((*starMag)[blob_count] > 25400) {
                        spacing = spacing * 4;
                    }
                    for (int ib = 0; ib < blob_count; ib++) {
                        if ((abs((*starX)[blob_count]-(*starX)[ib]) < spacing) &&
                            (abs((*starY)[blob_count]-(*starY)[ib]) < spacing)) {
                            unique = 0;
                            // keep the brighter one
                            if ((*starMag)[blob_count] > (*starMag)[ib]) {
                                (*starX)[ib] = (*starX)[blob_count];
                                (*starY)[ib] = (*starY)[blob_count];
                                (*starMag)[ib] = (*starMag)[blob_count];
                            }
                        }
                    }
                    // if we didn't find a close one, it is unique.
                    if (unique) {
                        blob_count++;
                    }
                }
            }
        }
    }
    // this loop flips the vertical position of the blobs back to their normal location
    for (int ibb = 0; ibb < blob_count; ibb++) {
        (*starY)[ibb] = CAMERA_HEIGHT - (*starY)[ibb];
    }

    // merge sort
    part(*starMag, 0, blob_count-1, *starX, *starY); 
    printf("Number of blobs found in image: %i\n", blob_count);
    return blob_count;
}

// sorting implementation
void merge(double * A, int p, int q, int r, double * X, double * Y){
    int n1 = q - p + 1, n2 = r - q;
    int lin = n1 + 1, rin = n2 + 1;
    double LM[lin], LX[lin], LY[lin], RM[rin], RX[rin], RY[rin];
    int i, j, k;
    LM[n1] = 0;
    RM[n2] = 0;

    for (i = 0; i < n1; i++) {
        LM[i] = A[p+i];
        LX[i] = X[p+i];
        LY[i] = Y[p+i];
    }

    for (j = 0; j < n2; j++) {
        RM[j] = A[q+j+1];
        RX[j] = X[q+j+1];
        RY[j] = Y[q+j+1];
    }

    i = 0; j = 0;
    for (k = p; k <= r; k++) {
        if (LM[i] >= RM[j] ){
            A[k] = LM[i];
            X[k] = LX[i];
            Y[k] = LY[i];
            i++;
        } else {
            A[k] = RM[j];
            X[k] = RX[j];
            Y[k] = RY[j];
            j++;
        }
    }
}

// recursive part of sorting 
void part(double * A, int p, int r, double * X, double * Y) {
    if (p < r) {
        int q = (p + r)/2;
        part(A, p, q, X, Y);
        part(A, q + 1, r, X, Y);
        merge(A, p, q, r, X, Y);
    }
}

// for testing saved images in astrometry implementation 
int loadDummyPicture(char * filename, char * buffer) {
    FILE * fp = NULL;

    if (!(fp = fopen(filename, "rb"))) {
        printf("Could not load dummy picture file %s.\n", filename);
        return 0;
    }   

    // find the offset to image data (first 8 bytes)
    char header[14] = {0};
    fread(header, 1, 14, fp);
    int offset = (int) *((int*)((long long int)(header + 10))); 

    // go to image data
    fseek(fp, offset, SEEK_SET);  

    // read image data
    fread(buffer, 1, CAMERA_WIDTH * CAMERA_HEIGHT, fp); 

    // close image file and return
    fclose(fp);
    return 1;
}

// make a table of stars (mostly used for testing)
int makeTable(char * filename, char * blobfile, char * buffer, double * starMag, double * starX, double * starY, int blob_count) {
    FILE * fp = NULL;

    if (!(fp = fopen(filename, "w"))) {
        printf("Could not load blob-writing table file %s.\n", filename);
        return 0;
    }

    fptr = fopen(blobfile, "a");
    fprintf(fptr, "------------------------------------------\n");
    if (all_blob_params.use_static_hp_mask) {
        fprintf(fptr, "Using static hot pixel map.\n");
    } else {
        fprintf(fptr, "Not using static hot pixel map.\n");
    }
    for (int i = 0; i < blob_count; i++) {
        fprintf(fp, "%f,%f,%f\n", starMag[i], starX[i], starY[i]);
        // write the specific blob pixel coordinates to a separate data file 
        fprintf(fptr, "[%i, %i]\n", (int) starX[i], (int) starY[i]);
    }

    // close both files and return
    fclose(fp);
    fclose(fptr);
    return 1;
}

void doCameraAndAstrometry() {
    /* uncomment line below for testing the values of each field in the global structure for blob_params. */ 
    // verifyBlobParams();

    // for testing purposes (below)
    int stop = 1; 

    // int img_counter = 0;
  
    // starX, starY, and starMag get allocated and set in get_blobs()
    static double * starX = NULL, * starY = NULL, * starMag = NULL;
    static char * output_buffer = NULL;
    static int first_time = 1;
    // for writing to data txt file
    char datafile[100];
    char blobfile[100];
    // for formatting date in filename
    char date[256];
    char buff[100];
    // leap year boolean
    int leap_year;
    
    if (first_time) {
        output_buffer = calloc(1, CAMERA_WIDTH * CAMERA_HEIGHT);
        first_time = 0;
    }

    wchar_t filename[200] = L"";

    // set up time
    time_t seconds = time(NULL);
    struct tm * tm_info;
    tm_info = gmtime(&seconds);
    // determine if it is a leap year
    leap_year = isLeapYear(tm_info->tm_year);
    // printf("Is it a leap year? %s\n", leap_year ? "Yes":"No");
    // if it is a leap year, adjust tm_info accordingly before it is passed to calculations in lost_in_space
    if (leap_year) {
        if (tm_info->tm_yday == 59) {   // if we are on Feb 29th
            tm_info->tm_yday++;         // there are 366, not 365, days in a leap year
            tm_info->tm_mon -= 1;       // we are still in February 59 days after January 1st (Feb 29)
            tm_info->tm_mday = 29;
        } else if (tm_info->tm_yday > 59) {
            tm_info->tm_yday++;         // there are 366, not 365, days in a leap year, and this extra day is added 
                                        // if we are on Feb 29 or after
        }
    }

    // captures an image 
    status = is_FreezeVideo(cameraHandle, IS_WAIT);
    if (status == -1) {
      printf("Failed to capture image.");
      exit(2);
    }

    // name image file with time
    // change back to this after static hot pixel map: "/home/xscblast/Desktop/blastcam/BMPs/saved_image_%Y-%m-%d_%H:%M:%S.bmp"
    strftime(date, sizeof(date), "/home/xscblast/Desktop/blastcam/BMPs/static_hot_pixel_test_%Y-%m-%d_%H:%M:%S.bmp", tm_info);
    swprintf(filename, 200, L"%s", date);
    ImageFileParams.pwchFileName = filename;

    // get the image from memory
    status = is_GetActSeqBuf(cameraHandle, &bufferNumber, &waitingMem, &memory);

    // testing pictures that have already been taken 
    // loadDummyPicture("/home/xscblast/Desktop/blastcam/BMPs/saved_image_2019-07-01-23-34-22.bmp", memory); 
    // loadDummyPicture("/home/xscblast/Desktop/blastcam/BMPs/static_hot_pixel_test_2020-03-27_23:08:10.bmp", memory);
    loadDummyPicture("/home/xscblast/Desktop/blastcam/BMPs/saved_image_2020-03-22_00:05:16.bmp", memory);
    // for logodds testing:
    // loadDummyPicture("/home/xscblast/Desktop/blastcam/BMPs/saved_image_2020-03-08_05:54:58.bmp", memory);
    // pointer for transmitting to user should point to where image is in memory
    camera_raw = memory;

    // find the blobs in the image
    int blob_count = get_blobs(memory, CAMERA_WIDTH, CAMERA_HEIGHT, &starX, &starY, &starMag, output_buffer, date); // remove date as input once done testing
    /* uncomment to make kst display the filtered image. */
    // memcpy(memory, output_buffer, CAMERA_WIDTH * CAMERA_HEIGHT); 

    // save image
    status = is_ImageFile(cameraHandle, IS_IMAGE_FILE_CMD_SAVE, (void*) &ImageFileParams, sizeof(ImageFileParams));
    // all_blob_params.make_static_hp_mask = 20; // to re-make hp mask again (only for taking dark test images)
    if (status == -1) {
        char * lastErrorString;
        int lastError = 0;
        is_GetError(cameraHandle, &lastError, &lastErrorString);
        printf("Failed to save image, error string: %s\n", lastErrorString);
        exit(2);
    }

    wprintf(L"Saving to \"%s\"\n", filename);
    // unlink whatever the latest saved image was linked to before
    unlink("/home/xscblast/Desktop/blastcam/BMPs/latest_saved_image.bmp");
    // symbolically link current date to latest image for kst to pull from for live updates
    symlink(date, "/home/xscblast/Desktop/blastcam/BMPs/latest_saved_image.bmp");

    // data file for writing to pass to makeTable and lost_in_space
    strftime(datafile, sizeof(datafile), "/home/xscblast/Desktop/blastcam/data_%b-%d.txt", tm_info); 
    // write number of blobs to observing data file
    fptr = fopen(datafile, "a");
    // if this is the first round of solving, write some preliminary information to the observing file
    if (first_run) {
        // get frame rate again
        is_SetFrameRate(cameraHandle, IS_GET_FRAMERATE, (void *) &actualFPS);

        // write observing information to data file
        strftime(buff, sizeof(buff), "%B %d Observing Session - beginning %H:%M:%S GMT", tm_info); 
        fprintf(fptr, "************************ %s ************************\n", buff);
        fprintf(fptr, "Camera model: %s\n", sensorInfo.strSensorName);
        fprintf(fptr, "----------------------------------------------------\n");
        fprintf(fptr, "Exposure: %f milliseconds\n", currentExposure);
        fprintf(fptr, "Pixel clock: %i\n", currPixelClock);
        fprintf(fptr, "Frame rate achieved (desired is 10): %f\n", actualFPS);
        fprintf(fptr, "Trigger delay (microseconds): %i\n", currTrigDelay);
        fprintf(fptr, "Current trigger mode setting: %i\n", currExtTrig);
        fprintf(fptr, "Current trigger timeout: %i\n", currTimeout);
        fprintf(fptr, "Auto shutter (should be disabled): %.1f\n", currShutter);
        fprintf(fptr, "Auto frame rate (should be disabled): %.1f\n", actualAutoFR);
        fprintf(fptr, "----------------------------------------------------\n");
        fprintf(fptr, "Sensor ID/type: %hu\n", sensorInfo.SensorID);
        fprintf(fptr, "Sensor color mode (from is_GetSensorInfo and is_SetColorMode): %i / %i\n", sensorInfo.nColorMode, currColorMode);
        fprintf(fptr, "Maximum image width and height: %i, %i\n", sensorInfo.nMaxWidth, sensorInfo.nMaxHeight);
        fprintf(fptr, "Pixel size (micrometers): %hu\n", sensorInfo.wPixelSize);
        fprintf(fptr, "Gain settings: %i for master gain, %i for red gain, %i for green gain, and %i for blue gain.\n", 
                       currMasterGain, currRedGain, currGreenGain, currBlueGain);
        fprintf(fptr, "Auto gain (should be disabled): %i\n", (int) currAutoGain);
        fprintf(fptr, "Gain boost (should be disabled): %i\n", currGainBoost);
        fprintf(fptr, "Hardware gamma (should be disabled): %i\n", currGamma);
        fprintf(fptr, "----------------------------------------------------\n");
        fprintf(fptr, "Auto black level (should be off): %i\n", actualBLMode);
        fprintf(fptr, "Black level offset (desired is 50): %i\n", actualBLOffset);

        // write header to data file
        fprintf(fptr, "Blob #|GMT            |LST (deg)    |RA (deg)  |DEC (deg)|FR (deg)   |PS      |ALT (deg)   |AZ (deg)    |IR (deg)|Solve time (msec)");
    
        // reset buffer for later writing
        memset(buff, 0, sizeof(buff));
    }

    // write blob and time information to data file
    strftime(buff, sizeof(buff), "%b %d %H:%M:%S", tm_info); 
    fprintf(fptr, "\r%i     |%s|", blob_count, buff);
    fclose(fptr);

    // make a table of the blobs for kst2
    // change back to this after static hot pixel map: blobs_%b-%d.txt
    strftime(blobfile, sizeof(blobfile), "/home/xscblast/Desktop/blastcam/blobs_static_hot_pixel_testing_%b-%d.txt", tm_info); 
    makeTable("makeTable.txt", blobfile, memory, starMag, starX, starY, blob_count);    

    // solve astrometry
    printf("Trying to solve astrometry...\n");
    status = lost_in_space_astrometry(starX, starY, starMag, blob_count, tm_info, datafile);

    if (status) {
        // change to 0 to make loop stop on solve (for testing a single solution)
        stop = 1;
    } else {
        printf("Could not solve astrometry.\n");
    }

    // after this round we are no longer on the first solve of the session
    first_run = 0;

    memset(filename, 0, sizeof(filename));
    memset(buff, 0, sizeof(buff));
}