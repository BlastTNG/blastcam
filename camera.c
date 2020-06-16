#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <ueye.h>
#include <stdbool.h>

#include "camera.h"
#include "astrometry.h"
#include "commands.h"
#include "lens_adapter.h"
#include "matrix.h"

void merge(double A[], int p, int q, int r, double X[],double Y[]);
void part(double A[], int p, int r, double X[], double Y[]);

// 1...254 are possible camera IDs
HIDS camera_handle = 12;            // set by ueyesetid (can also be set with is_SetCameraID)
// breaking our convention of using underscores for struct names because this is how iDS does it in their documentation
IMAGE_FILE_PARAMS ImageFileParams;
// same with sensorInfo struct
SENSORINFO sensorInfo;
FILE * fptr;

// global variables
int buffer_num;
char * memory;
char * waiting_mem;
int shutting_down;
unsigned char * mask;
char * mem_starting_ptr;
int mem_id;
// for printing camera errors
const char * cam_error;
// 'curr' = current, 'pc' = pixel clock, 'fps' = frames per sec, 'ag' = auto gain, 'bl' = black level
double curr_exposure;
int curr_pc;
double actual_fps;
double curr_ag;
double curr_shutter;
double auto_fr;
int curr_color_mode;
int curr_ext_trig;
int curr_trig_delay;
int curr_master_gain;
int curr_red_gain;
int curr_green_gain;
int curr_blue_gain;
int curr_gamma;
int curr_gain_boost;
unsigned int curr_timeout;
int bl_offset;
int bl_mode;

/* Blob parameters global structure (defined in camera.h) */
struct blob_params all_blob_params = {
    .spike_limit = 3,              // how agressive is the dynamic hot pixel finder.  Small is more agressive
    .dynamic_hot_pixels = 1,       // 0 = off, 1 = on
    .r_smooth = 2,                 // image smooth filter radius [px]
    .high_pass_filter = 0,         // 0 = off, 1 = on
    .r_high_pass_filter = 10,      // image high pass filter radius [px]
    .centroid_search_border = 1,   // distance from image edge from which to start looking for stars [px]
    .filter_return_image = 0,      // 1 = true; 0 = false
    .n_sigma = 2.0,                // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
    .unique_star_spacing = 15,     // minimum spacing between two unique stars [px]
    .make_static_hp_mask = 0,      // make the mask once (threshold value should be 20), set to 0 when finished with test pictures
    .use_static_hp_mask = 1,       // always use the hot pixel mask 
};

/* Helper function to determine if the current year is a leap year (2020 is a leap year).
** Input: The year.
** Output: A flag indicating the input year is a leap year or not.
**/
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

/* Helper function to test blob-finding parameters.
** Input: None.
** Output: None (void). Prints the current blob-finding parameters to the terminal.
**/
void verifyBlobParams() {
    printf("\nin camera.c, all_blob_params.spike_limit is: %d\n", all_blob_params.spike_limit);
    printf("in camera.c, all_blob_params.dynamic_hot_pixels is: %d\n", all_blob_params.dynamic_hot_pixels);
    printf("in camera.c, all_blob_params.centroid_search_border is: %d\n", all_blob_params.centroid_search_border);
    printf("in camera.c, all_blob_params.high_pass_filter is: %d\n", all_blob_params.high_pass_filter);
    printf("in camera.c, all_blob_params.r_smooth is: %d\n", all_blob_params.r_smooth);
    printf("in camera.c, all_blob_params.filter_return_image is: %d\n", all_blob_params.filter_return_image);
    printf("in camera.c, all_blob_params.r_high_pass_filter is: %d\n", all_blob_params.r_high_pass_filter);
    printf("in camera.c, all_blob_params.n_sigma is: %f\n", all_blob_params.n_sigma);
    printf("in camera.c, all_blob_params.unique_star_spacing is: %d\n", all_blob_params.unique_star_spacing);
    printf("in camera.c, all_blob_params.make_static_hp_mask is: %i\n", all_blob_params.make_static_hp_mask);
    printf("in camera.c, all_blob_params.use_static_hp_mask is: %i\n\n", all_blob_params.use_static_hp_mask);
}

/* Helper function to print camera errors.
** Input: None.
** Output: The error from the camera.
*/
const char * printCameraError() {
    char * last_error_str;
    int last_err = 0;

    is_GetError(camera_handle, &last_err, &last_error_str);
    return last_error_str;
}

/* Function to initialize the camera and its various parameters.
** Input: None.
** Output: A flag indicating successful camera initialization or not.
*/
int initCamera() {
    double min_exposure;
    double max_exposure;
    unsigned int enable = 1;   

    // load the camera parameters
    if (loadCamera() < 0) {
        return -1;
    }

    // enable long exposure 
    if (is_Exposure(camera_handle, IS_EXPOSURE_CMD_SET_LONG_EXPOSURE_ENABLE, (void *) &enable, sizeof(unsigned int)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Unable to enable long exposure: %s.\n", cam_error);
        return -1; 
    }

    // set exposure time based on struct field
    if (is_Exposure(camera_handle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void *) &all_camera_params.exposure_time, sizeof(double)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Unable to set default exposure: %s.\n", cam_error);
        return -1;
    }

    // get current exposure, max possible exposure (with long exposure enabled), and min exposure
    if (is_Exposure(camera_handle, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, &min_exposure, sizeof(double)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Unable to get minimum possible exposure: %s.\n", cam_error);
        return -1;
    }

    if (is_Exposure(camera_handle, IS_EXPOSURE_CMD_GET_EXPOSURE, &curr_exposure, sizeof(double)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Unable to get current exposure value: %s.\n", cam_error);
        return -1;
    }

    if (is_Exposure(camera_handle, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, &max_exposure, sizeof(double)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Unable to get maximum exposure value: %s.\n", cam_error);
        return -1;
    } else {
        printf("Current exposure time: %f msec | Min possible exposure: %f msec | Max possible exposure: %f msec\n\n", 
                curr_exposure, min_exposure, max_exposure);
    }

    // initialize astrometry
    if (initAstrometry() < 0) {
        return -1;
    }

    // set how images are saved
	setSaveImage();

    return 1;
}

/* Cleaning up function for ctrl+c exception.
** Input: None.
** Output: None (void). Changes the state of the camera to shutting down.
**/
void clean() {
    printf("\nEntering shutting down state...\n");
    // camera is in shutting down state (stop solving)
    shutting_down = 1;
}

/* Function to close the camera when shutting down.
** Input: None.
** Output: None (void).
*/
void closeCamera() {
    printf("Closing camera with handle %d...\n", camera_handle);
    // don't close a camera that doesn't exist yet!
    if ((mem_starting_ptr != NULL) && (camera_handle <= 254)) { 
        is_FreeImageMem(camera_handle, mem_starting_ptr, mem_id);
        is_ExitCamera(camera_handle);
    }
}

/* Function to set starting values for certain camera atributes.
** Input: None.
** Output: A flag indicating successful setting of the camera parameters.
*/
int setCameraParams() {
    // explicitly set these to 0 in initialization to emphasize disabling purpose
    double ag = 0;            
    double auto_shutter = 0;
    double afr = 0;          
    int blo;                  
    int blm;                  

    // set the trigger delay to 0 (microseconds) = deactivates trigger delay
    if (is_SetTriggerDelay(camera_handle, 0) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting the trigger to 0 microseconds: %s.\n", cam_error);
        return -1;
    }
    // check the trigger delay is 0
    curr_trig_delay = is_SetTriggerDelay(camera_handle, IS_GET_TRIGGER_DELAY); 
    printf("Trigger delay (should be 0): %i\n", curr_trig_delay);

    // set camera integrated amplifier 
    if (is_SetHardwareGain(camera_handle, 0, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting gain: %s.\n", cam_error);
        return -1;
    }
    // check color gains
    curr_master_gain = is_SetHardwareGain(camera_handle, IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
    curr_red_gain = is_SetHardwareGain(camera_handle, IS_IGNORE_PARAMETER, IS_GET_RED_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
    curr_green_gain = is_SetHardwareGain(camera_handle, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_GET_GREEN_GAIN, IS_IGNORE_PARAMETER);
    curr_blue_gain = is_SetHardwareGain(camera_handle, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_GET_BLUE_GAIN);
    printf("Gain settings: %i for master gain, %i for red gain, %i for green gain, and %i for blue gain.\n", 
            curr_master_gain, curr_red_gain, curr_green_gain, curr_blue_gain);

    // disable camera's auto gain (0 = disabled)
    if (is_SetAutoParameter(camera_handle, IS_SET_ENABLE_AUTO_GAIN, &ag, NULL) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error disabling auto gain: %s.\n", cam_error);
        return -1;
    }
    // check auto gain is off
    is_SetAutoParameter(camera_handle, IS_GET_ENABLE_AUTO_GAIN, &curr_ag, NULL);
    printf("Auto gain (should be disabled): %f\n", curr_ag);

    // set camera's gamma value to off 
    if (is_SetHardwareGamma(camera_handle, IS_SET_HW_GAMMA_OFF) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error disabling hardware gamma correction: %s.\n", cam_error);
        return -1;
    }
    // check hardware gamma is off
    curr_gamma = is_SetHardwareGamma(camera_handle, IS_GET_HW_GAMMA);
    printf("Hardware gamma (should be off): %i\n", curr_gamma);

    // disable auto shutter
    if (is_SetAutoParameter(camera_handle, IS_SET_ENABLE_AUTO_SHUTTER, &auto_shutter, NULL) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error disabling auto shutter: %s.\n", cam_error);
        return -1;
    }
    // check auto shutter is off
    is_SetAutoParameter(camera_handle, IS_GET_ENABLE_AUTO_SHUTTER, &curr_shutter, NULL);
    printf("Auto shutter (should be off): %.1f\n", curr_shutter);

    // disable auto frame rate
    if (is_SetAutoParameter(camera_handle, IS_SET_ENABLE_AUTO_FRAMERATE, &afr, NULL) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error disabling auto frame rate: %s.\n", cam_error);
        return -1;
    }
    // check auto frame rate is off
    is_SetAutoParameter(camera_handle, IS_GET_ENABLE_AUTO_FRAMERATE, &auto_fr, NULL);
    printf("Auto frame rate (should be off): %.1f\n", auto_fr);

    // disable gain boost to reduce noise
    if (is_SetGainBoost(camera_handle, IS_SET_GAINBOOST_OFF) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error disabling gain boost: %s.\n", cam_error);
        return -1;
    }
    // check gain boost is off
    curr_gain_boost = is_SetGainBoost(camera_handle, IS_GET_GAINBOOST);
    printf("Gain boost (should be disabled): %i\n", curr_gain_boost);	

    // turn off auto black level
    blm = IS_AUTO_BLACKLEVEL_OFF;
    if (is_Blacklevel(camera_handle, IS_BLACKLEVEL_CMD_SET_MODE, (void *) &blm, sizeof(blm)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error turning off auto black level mode: %s.\n", cam_error);
        return -1;
    }
    // check auto black level is off
    bl_mode = is_Blacklevel(camera_handle, IS_BLACKLEVEL_CMD_GET_MODE, (void *) &bl_mode, sizeof(bl_mode));
    printf("Auto black level (should be off): %i\n", bl_mode);

    // set black level offset to 50
    blo = 50;
    if (is_Blacklevel(camera_handle, IS_BLACKLEVEL_CMD_SET_OFFSET, (void *) &blo, sizeof(blo)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting black level offset to 50: %s.\n", cam_error);
        return -1;
    }
    // check black level offset
    is_Blacklevel(camera_handle, IS_BLACKLEVEL_CMD_GET_OFFSET, (void *) &bl_offset, sizeof(bl_offset));
    printf("Black level offset (desired is 50): %i\n", bl_offset);

    // This for some reason actually affects the time it takes to set aois only on the focal plane camera. 
    // Don't use if for the trigger timeout. 
    if (is_SetTimeout(camera_handle, IS_TRIGGER_TIMEOUT, 500) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting trigger timeout: %s.\n", cam_error);
        return -1;
    }
    // check time out
    is_GetTimeout(camera_handle, IS_TRIGGER_TIMEOUT, &curr_timeout);
    printf("Current trigger timeout: %i\n", curr_timeout);

    return 1;
}

/* Function to load the camera at the beginning of the Star Camera session.
** Input: None.
** Output: A flag indicating successful loading of the camera or not.
*/
int loadCamera() {
    int color_depth;
    void * active_mem_loc;
    int pixelclock;
    double fps;

    // initialize camera
	if (is_InitCamera(&camera_handle, NULL) != IS_SUCCESS) {
        return -1;
	}
  
    // get sensor info
	if (is_GetSensorInfo(camera_handle, &sensorInfo) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error getting camera sensor information %s.\n", cam_error);
        return -1;
	} 

    // set various other camera parameters
	if (setCameraParams() < 0) {
        return -1;
    }

    // set display mode and then get it to verify
	if (is_SetColorMode(camera_handle, IS_CM_SENSOR_RAW8) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting color mode: %s.\n", cam_error);
        return -1;
	}
    curr_color_mode = is_SetColorMode(camera_handle, IS_GET_COLOR_MODE);
    printf("Camera model: %s\n", sensorInfo.strSensorName);
    printf("Sensor ID/type: %i\n", sensorInfo.SensorID);
    printf("Sensor color mode (from is_GetSensorInfo and is_SetColorMode): %i / %i\n", sensorInfo.nColorMode, curr_color_mode);
    printf("Maximum image width and height: %i, %i\n", sensorInfo.nMaxWidth, sensorInfo.nMaxHeight);
    printf("Pixel size (micrometers): %.2f\n", ((double) sensorInfo.wPixelSize)/100.0);
 	
    // allocate camera memory
	color_depth = 8; 
	if (is_AllocImageMem(camera_handle, sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, color_depth, &mem_starting_ptr, &mem_id) != IS_SUCCESS) {
		cam_error = printCameraError();
        printf("Error allocating image memory: %s.\n", cam_error);
        return -1;
	}

    // set memory for image (make memory pointer active)
	if (is_SetImageMem(camera_handle, mem_starting_ptr, mem_id) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting image memory: %s.\n", cam_error);
        return -1;
    }

    // get image memory
	if (is_GetImageMem(camera_handle, &active_mem_loc) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error getting image memory: %s.\n", cam_error);
        return -1;
    }

    // how clear images can be is affected by pixelclock and fps 
    pixelclock = 30;
	if (is_PixelClock(camera_handle, IS_PIXELCLOCK_CMD_SET, (void *) &pixelclock, sizeof(pixelclock)) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting pixel clock: %s.\n", cam_error);
        return -1;
    }
    // get current pixel clock to check
    is_PixelClock(camera_handle, IS_PIXELCLOCK_CMD_GET, (void *) &curr_pc, sizeof(curr_pc));
    printf("Pixel clock: %i\n", curr_pc);

    // set frame rate
	fps = 10;
	if (is_SetFrameRate(camera_handle, IS_GET_FRAMERATE, (void *) &fps) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting frame rate: %s.\n", cam_error);
        return -1;
    }

    // set trigger to software mode (call is_FreezeVideo to take a single picture in single frame mode)
	if (is_SetExternalTrigger(camera_handle, IS_SET_TRIGGER_SOFTWARE) != IS_SUCCESS) {
        cam_error = printCameraError();
        printf("Error setting external trigger mode: %s.\n", cam_error);
        return -1;
	}
    // get the current trigger setting
    curr_ext_trig = is_SetExternalTrigger(camera_handle, IS_GET_EXTERNALTRIGGER);
    printf("Current external trigger mode: %i\n", curr_ext_trig);

    // confirmation message that camera is initialized
    printf("Done initializing camera.\n");
    printf("----------------------------\n");
    return 1;
}

/* Function to establish the parameters for saving images taken by the camera.
** Input: None.
** Output: None (void).
*/
void setSaveImage() {
    ImageFileParams.pwchFileName = L"save1.bmp";
    ImageFileParams.pnImageID = NULL;
    ImageFileParams.ppcImageMem = NULL;
    ImageFileParams.nQuality = 80;
    ImageFileParams.nFileType = IS_IMG_BMP;
}

/* Function to mask hot pixels accordinging to static and dynamic maps.
** Input: The image bytes (ib), the image border indices (i0, j0, i1, j1) - the rest are 0.
** Output: None (void). Makes the dynamic and static hot pixel masks for the Star Camera image.
*/
void makeMask(char * ib, int i0, int j0, int i1, int j1, int x0, int y0, bool subframe) {
    static int first_time = 1;
    static int * x_p = NULL;
    static int * y_p = NULL;
    static int num_p = 0;
    static int num_alloc = 0;

    if (first_time) {
        mask = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, 1);
        x_p = calloc(100, sizeof(int));
        y_p = calloc(100, sizeof(int));
        num_alloc = 100;
    }

    // load static hot pixel mask
    if (first_time || all_blob_params.make_static_hp_mask) {
        // read from file with bright pixel coordinates in it
        FILE * f = fopen(STATIC_HP_MASK, "r");
        int i, j;
        num_p = 0;

        if (f) {
            printf("******************************* Loading static hot pixel map... *******************************\n");
            char * line = NULL;
            size_t len = 0;
            int read = 0;

            // read every line in the file
            while ((read = getline(&line, &len, f)) != -1) {
                sscanf(line, "%d,%d\n", &i, &j);

                if (num_p >= num_alloc) {
                    num_alloc += 100;
                    x_p = realloc(x_p, sizeof(int) * num_alloc);
                    y_p = realloc(y_p, sizeof(int) * num_alloc);
                }

                x_p[num_p] = i;
                // map y coordinate to image in memory from Kst blob
                y_p[num_p] = CAMERA_HEIGHT - j; 

                printf("Coordinates read from hp file: [%i, %i]\n", i, j);

                num_p++;
            }
            fflush(f);
            fclose(f);
        }

        // do not want to recreate hp mask automatically, so set field to 0
        all_blob_params.make_static_hp_mask = 0;
        // no longer the first time we are running
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
        mask[i0 + j*CAMERA_WIDTH] = mask[i1 - 1 + j*CAMERA_WIDTH] = 0;
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
        printf("******************************* Masking %d pixels... *******************************\n", num_p);
        for (int i = 0; i < num_p; i++) {
            // set all static hot pixels to 0
            printf("Coordinates going into mask index: [%i, %i]\n", x_p[i], CAMERA_HEIGHT - y_p[i]);
            int ind = CAMERA_WIDTH * y_p[i] + x_p[i];
            mask[ind] = 0;
        }
    }
}

/* Function to process the image with a filter to reduce noise.
** Input:
** Output:
*/
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

/* Function to find the blobs in an image.
** Inputs: the original image prior to processing (input_biffer), the dimensions of the image (w & h)
** pointers to arrays for the x coordinates, y coordinates, and magnitudes (pixel values) of the blobs,
** and an array for the bytes of the image after processing (masking, filtering, et cetera).
** Output: the number of blobs detected in the image.
*/
int findBlobs(char * input_buffer, int w, int h, double ** star_x, double ** star_y, double ** star_mags, char * output_buffer) { 
    static int first_time = 1;
    static double * ic = NULL;
    static double * ic2 = NULL;
    static int num_blobs_alloc = 0;

    // allocate the proper amount of storage space to start
    if (first_time) {
        ic = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, sizeof(double));
        ic2 = calloc(CAMERA_WIDTH*CAMERA_HEIGHT, sizeof(double));
        first_time = 0;
    }
  
    // we use half-width internally, but the API gives us full width.
    int x_size = w/2;
    int y_size = h/2;

    int j0, j1, i0, i1;
    int b = 0;           // extra image border

    j0 = i0 = 0;
    j1 = h;
    i1 = w;
    
    b = all_blob_params.centroid_search_border;

    // if we want to make a new hot pixel mask
    if (all_blob_params.make_static_hp_mask) {
        printf("******************************* Making static hot pixel map... *******************************\n");
        // make a file and write bright pixel coordinates to it
        FILE * f = fopen(STATIC_HP_MASK, "w"); 

        for (int yp = 0; yp < CAMERA_HEIGHT; yp++) {
            for (int xp = 0; xp < CAMERA_WIDTH; xp++) {
                // index in the array where we are
                int ind = CAMERA_WIDTH*yp + xp; 
                // check pixel value to see if it's above hot pixel threshold
                if (input_buffer[ind] > all_blob_params.make_static_hp_mask) {
                    int i = xp;
                    // make this agree with blob coordinates in Kst
                    int j = CAMERA_HEIGHT - yp; 
                    fprintf(f, "%d,%d\n", i, j);
                    printf("Value of pixel (should be > %i): %d | Coordinates: [%d, %d]\n", all_blob_params.make_static_hp_mask, 
                                                                                            input_buffer[ind], i, j);
                }
            }
        }
        fflush(f);
        fclose(f);
    }

    makeMask(input_buffer, i0, j0, i1, j1, 0, 0, 0);

    double sx = 0, sx2 = 0;
    // sum of non-highpass filtered field
    double sx_raw = 0;                            
    int num_pix = 0;

    // lowpass filter the image - reduce noise.
    boxcarFilterImage(input_buffer, i0, j0, i1, j1, all_blob_params.r_smooth, ic);

    // only high-pass filter full frames
    if (all_blob_params.high_pass_filter) {       
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
            printf("Filtering returned image...\n");

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
            printf("Not filtering the returned image...\n");
            for (int j = j0; j < j1; j++) {
                for (int i = i0; i < i1; i++) {
                  int idx = i + j*w;
                  output_buffer[idx] = input_buffer[idx];
                }
            }
        }
    }

    // find the blobs 
    double ic0;
    int blob_count = 0;
    for (int j = j0 + b; j < j1-b-1; j++) {
        for (int i = i0 + b; i < i1-b-1; i++) {
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
                     (ic0 > 254)) {
                      
                    int unique = 1;

                    // realloc array if necessary (when the camera looks at a really bright image, this slows everything down severely)
                    if (blob_count >= num_blobs_alloc) {
                        num_blobs_alloc += 500;
                        *star_x = realloc(*star_x, sizeof(double)*num_blobs_alloc);
                        *star_y = realloc(*star_y, sizeof(double)*num_blobs_alloc);
                        *star_mags = realloc(*star_mags, sizeof(double)*num_blobs_alloc);
                     }

                    (*star_x)[blob_count] = i;
                    (*star_y)[blob_count] = j;
                    (*star_mags)[blob_count] = 100*ic[i + j*CAMERA_WIDTH];

                    // FIXME: not sure why this is necessary..
                    if ((*star_mags)[blob_count] < 0) {
                        (*star_mags)[blob_count] = UINT32_MAX;
                    }

                    // If we already found a blob within SPACING and this one is bigger, replace it.
                    int spacing = all_blob_params.unique_star_spacing;
                    if ((*star_mags)[blob_count] > 25400) {
                        spacing = spacing * 4;
                    }
                    for (int ib = 0; ib < blob_count; ib++) {
                        if ((abs((*star_x)[blob_count]-(*star_x)[ib]) < spacing) &&
                            (abs((*star_y)[blob_count]-(*star_y)[ib]) < spacing)) {
                            unique = 0;
                            // keep the brighter one
                            if ((*star_mags)[blob_count] > (*star_mags)[ib]) {
                                (*star_x)[ib] = (*star_x)[blob_count];
                                (*star_y)[ib] = (*star_y)[blob_count];
                                (*star_mags)[ib] = (*star_mags)[blob_count];
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
        (*star_y)[ibb] = CAMERA_HEIGHT - (*star_y)[ibb];
    }

    // merge sort
    part(*star_mags, 0, blob_count - 1, *star_x, *star_y); 
    printf("Number of blobs found in image: %i\n", blob_count);
    return blob_count;
}

/* Function for sorting implementation.
** Input: 
** Output:
*/
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

/* Recursive part of sorting.
** Input:
** Output:
*/
void part(double * A, int p, int r, double * X, double * Y) {
    if (p < r) {
        int q = (p + r)/2;
        part(A, p, q, X, Y);
        part(A, q + 1, r, X, Y);
        merge(A, p, q, r, X, Y);
    }
}

/* Function to load saved images (likely from previous observing sessions, but could 
** also be test pictures).
** Inputs: the name of the image to be loaded (filename) and the active image memory (buffer).
** Outputs: A flag indicating successful loading of the image or not.
*/
int loadDummyPicture(wchar_t * filename, char ** buffer) {
    ImageFileParams.ppcImageMem = buffer;
    ImageFileParams.pwchFileName = filename;
    ImageFileParams.ppcImageMem = NULL;
    if (is_ImageFile(camera_handle, IS_IMAGE_FILE_CMD_LOAD, (void *) &ImageFileParams, sizeof(ImageFileParams)) != IS_SUCCESS) {
        return -1;
    } else {
        return 1;
    }
    ImageFileParams.ppcImageMem = NULL;
}

/* Function to make a table of stars in an image for displaying in Kst (mostly for testing).
** Inputs: The name of the blob table file to write to, the array of blob magnitudes, the array
** of blob x coordinates, the array of blob y coordinates, and the number of blobs.
** Output: A flag indicating successful writing of the blob table file.
*/
int makeTable(char * filename, double * star_mags, double * star_x, double * star_y, int blob_count) {
    FILE * fp = NULL;

    if ((fp = fopen(filename, "w")) == NULL) {
    	fprintf(stderr, "Could not open blob-writing table file: %s.\n", strerror(errno));
    	return -1;
    }

    for (int i = 0; i < blob_count; i++) {
        fprintf(fp, "%f,%f,%f\n", star_mags[i], star_x[i], star_y[i]);
    }

    // close both files and return
    fflush(fp);
    fclose(fp);
    return 1;
}

/* Function to take observing images and solve for the pointing location using Astrometry.
** Main function for the Astrometry thread in commands.c.
** Input: None.
** Output: A flag indicating a successful round of image + solution by the camera (e.g. if the
** camera can't open the observing file, the function will automatically return with -1).
*/
int doCameraAndAstrometry() {
    // star_x, star_y, and star_mags get allocated and set in findBlobs() below
    static double * star_x = NULL, * star_y = NULL, * star_mags = NULL;
    static char * output_buffer = NULL;
    static int first_time = 1;
    char datafile[100];
    char date[256];
    char buff[100];
    wchar_t filename[200] = L"";
    int leap_year;
    // for timing how long the camera takes to operate (end of exposure to after solving finishes)
    struct timespec camera_tp_beginning, camera_tp_end; 

    // uncomment line below for testing the values of each field in the global structure for blob_params
    verifyBlobParams();

    if (first_time) {
        output_buffer = calloc(1, CAMERA_WIDTH*CAMERA_HEIGHT);
        camera_raw = calloc(1, CAMERA_WIDTH*CAMERA_HEIGHT);
    }

    time_t seconds = time(NULL);
    struct tm * tm_info;
    tm_info = gmtime(&seconds);
    all_astro_params.rawtime = seconds;
    leap_year = isLeapYear(tm_info->tm_year);
    // if it is a leap year, adjust tm_info accordingly before it is passed to calculations in lostInSpace
    if (leap_year) {
        if (tm_info->tm_yday == 59) {        // if we are on Feb 29th
            tm_info->tm_yday++;              // there are 366, not 365, days in a leap year
            tm_info->tm_mon -= 1;            // we are still in February 59 days after January 1st (Feb 29)
            tm_info->tm_mday = 29;
        } else if (tm_info->tm_yday > 59) {
            tm_info->tm_yday++;              // there are 366, not 365, days in a leap year, and this extra day is added 
                                             // if we are on Feb 29 or after
        }
    }

    // if auto-focusing flag is on, enter auto-focusing mode
    if (all_camera_params.focus_mode) {
        // check the auto-focusing parameters
        printf("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Now in auto-focusing mode in doCameraAndAstrometry(). ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        // perform auto-focusing function from lens_adapter.c
        int af_position = autofocus(tm_info);

        if (af_position == -1000) {
            printf("Error in auto-focusing process. Previous position before process will be kept.\n");
        } else {
            // move to the position determined by auto-focusing
            printf("Successfully determined optimal focus position: %d. Moving to it now...\n", af_position);

        }

        // set auto-focusing flag to 0 so the process is not accidentally repeated (unless the user specifies so)
        all_camera_params.focus_mode = 0;
    } else {
        printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ No longer auto-focusing. Proceeding to take observing images... ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        
        // capture an image 
        printf("Taking a new image...\n");
        if (is_FreezeVideo(camera_handle, IS_WAIT) != IS_SUCCESS) {
            printf("Failed to capture new image.");
        } 

        // get current time right after exposure
        if (clock_gettime(CLOCK_REALTIME, &camera_tp_beginning) == -1) {
            fprintf(stderr, "Error starting camera timer: %s.\n", strerror(errno));
        }

        // name new image file with time
        strftime(date, sizeof(date), "/home/xscblast/Desktop/blastcam/BMPs/saved_image_%Y-%m-%d_%H:%M:%S.bmp", tm_info);
        swprintf(filename, 200, L"%s", date);

        // get the image from memory
        if (is_GetActSeqBuf(camera_handle, &buffer_num, &waiting_mem, &memory) != IS_SUCCESS) {
            fprintf(stderr, "Error retrieving the active image memory: %s.\n", strerror(errno));
        }

        // testing pictures that have already been taken 
        // if (loadDummyPicture(L"/home/xscblast/Desktop/blastcam/BMPs/saved_image_2020-06-13_03:10:34.bmp", &memory) == 1) {
        //     printf("Successfully loaded test picture.\n");
        // } else {
        //     fprintf(stderr, "Error loading test picture: %s.\n", strerror(errno));
        //     // can't solve without a picture to solve on!
        //     usleep(1000000);
        //     return -1;
        // }

        // find the blobs in the image
        int blob_count = findBlobs(memory, CAMERA_WIDTH, CAMERA_HEIGHT, &star_x, &star_y, &star_mags, output_buffer);
    
        // make kst display the filtered image 
        memcpy(memory, output_buffer, CAMERA_WIDTH*CAMERA_HEIGHT); 

        // pointer for transmitting to user should point to where image is in memory
        camera_raw = output_buffer;

    	// save image
    	ImageFileParams.pwchFileName = filename;
    	if (is_ImageFile(camera_handle, IS_IMAGE_FILE_CMD_SAVE, (void *) &ImageFileParams, sizeof(ImageFileParams)) == -1) {
            const char * last_error_str = printCameraError();
            printf("Failed to save image: %s\n", last_error_str);
            return -1;
    	}

        wprintf(L"Saving to \"%s\"\n", filename);
        // unlink whatever the latest saved image was linked to before
        unlink("/home/xscblast/Desktop/blastcam/BMPs/latest_saved_image.bmp");
        // symbolically link current date to latest image for kst to pull from for live updates
        symlink(date, "/home/xscblast/Desktop/blastcam/BMPs/latest_saved_image.bmp");

        // data file for writing to pass to lostInSpace
        strftime(datafile, sizeof(datafile), "/home/xscblast/Desktop/blastcam/data_%b-%d.txt", tm_info); 
        // write number of blobs to observing data file
        if ((fptr = fopen(datafile, "a")) == NULL) {
    	    fprintf(stderr, "Could not open observing file: %s.\n", strerror(errno));
    	    return -1;
    	}

        // if this is the first round of solving, write some preliminary information to the observing file
        if (first_time) {
            // get frame rate again
            is_SetFrameRate(camera_handle, IS_GET_FRAMERATE, (void *) &actual_fps);

            // write observing information to data file
            strftime(buff, sizeof(buff), "%B %d Observing Session - beginning %H:%M:%S GMT", tm_info); 
            fprintf(fptr, "************************ %s ************************\n", buff);
            fprintf(fptr, "Camera model: %s\n", sensorInfo.strSensorName);
            fprintf(fptr, "----------------------------------------------------\n");
            fprintf(fptr, "Exposure: %f milliseconds\n", curr_exposure);
            fprintf(fptr, "Pixel clock: %i\n", curr_pc);
            fprintf(fptr, "Frame rate achieved (desired is 10): %f\n", actual_fps);
            fprintf(fptr, "Trigger delay (microseconds): %i\n", curr_trig_delay);
            fprintf(fptr, "Current trigger mode setting: %i\n", curr_ext_trig);
            fprintf(fptr, "Current trigger timeout: %i\n", curr_timeout);
            fprintf(fptr, "Auto shutter (should be disabled): %.1f\n", curr_shutter);
            fprintf(fptr, "Auto frame rate (should be disabled): %.1f\n", auto_fr);
            fprintf(fptr, "----------------------------------------------------\n");
            fprintf(fptr, "Sensor ID/type: %u\n", sensorInfo.SensorID);
            fprintf(fptr, "Sensor color mode (from is_GetSensorInfo and is_SetColorMode): %i | %i\n", sensorInfo.nColorMode, curr_color_mode);
            fprintf(fptr, "Maximum image width and height: %i, %i\n", sensorInfo.nMaxWidth, sensorInfo.nMaxHeight);
            fprintf(fptr, "Pixel size (micrometers): %.2f\n", ((double) sensorInfo.wPixelSize)/100.0);
            fprintf(fptr, "Gain settings: %i for master gain, %i for red gain, %i for green gain, and %i for blue gain.\n", 
                           curr_master_gain, curr_red_gain, curr_green_gain, curr_blue_gain);
            fprintf(fptr, "Auto gain (should be disabled): %i\n", (int) curr_ag);
            fprintf(fptr, "Gain boost (should be disabled): %i\n", curr_gain_boost);
            fprintf(fptr, "Hardware gamma (should be disabled): %i\n", curr_gamma);
            fprintf(fptr, "----------------------------------------------------\n");
            fprintf(fptr, "Auto black level (should be off): %i\n", bl_mode);
            fprintf(fptr, "Black level offset (desired is 50): %i\n", bl_offset);

            // write header to data file
            if (fprintf(fptr, "\nC time|GMT|Blob #|RA (deg)|DEC (deg)|FR (deg)|PS|ALT (deg)|AZ (deg)|IR (deg)|Astrom. solve time (msec)|Camera time (msec)") < 0) {
                fprintf(stderr, "Error writing header to observing file: %s.\n", strerror(errno));
            }

            // reset buffer for later writing
            memset(buff, 0, sizeof(buff));

            // no longer first time code is running (all first time tasks are completed)
            first_time = 0;
        }

        // write blob and time information to data file
        strftime(buff, sizeof(buff), "%b %d %H:%M:%S", tm_info); 
        printf("Time going into lostInSpace: %s\n", buff);
        if (fprintf(fptr, "\r%li|%s|", seconds, buff) < 0) {
            fprintf(stderr, "Unable to write time and blob count to observing file: %s.\n", strerror(errno));
        }
        fflush(fptr);
        fclose(fptr);

        // make a table of the blobs for kst2
        if (makeTable("makeTable.txt", star_mags, star_x, star_y, blob_count) != 1) {
            fprintf(stderr, "Error writing blob table for Kst: %s.\n", strerror(errno));
        }   

        // solve astrometry
        printf("Trying to solve astrometry...\n");
        //blob_count = min(blob_count, 30);
        if (lostInSpace(star_x, star_y, star_mags, blob_count, tm_info, datafile) != 1) {
            printf("Could not solve Astrometry.\n");
        }

        // get current time right after solving
        if (clock_gettime(CLOCK_REALTIME, &camera_tp_end) == -1) {
            fprintf(stderr, "Error ending Astrometry solving timer: %s.\n", strerror(errno));
        }

        // calculate time it took camera program to run in nanoseconds
	    double start = (double) (camera_tp_beginning.tv_sec*1e9) + (double) camera_tp_beginning.tv_nsec;
	    double end = (double) (camera_tp_end.tv_sec*1e9) + (double) camera_tp_end.tv_nsec;
        double camera_time = end - start;
	    printf("Camera completed one iteration in %f msec.\n", camera_time*1e-6);

        // write this time to the data file
        fptr = fopen(datafile, "a");
        if (fprintf(fptr, "|%f", camera_time*1e-6) < 0) {
            fprintf(stderr, "Unable to write Astrometry solution time to observing file: %s.\n", strerror(errno));
        }
        fflush(fptr);
        fclose(fptr);

        memset(filename, 0, sizeof(filename));
        memset(buff, 0, sizeof(buff));
        return 1;
    }
}