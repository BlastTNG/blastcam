#ifndef CAMERA_H
#define CAMERA_H

#define CAMERA_WIDTH   1936 	 // [px]
#define CAMERA_HEIGHT  1216	     // [px]
#define CAMERA_MARGIN  0		 // [px]
#define MIN_PS         6.0       // [arcsec/px]
#define MAX_PS         6.5		 // [arcsec/px]
#define STATIC_HP_MASK "/home/xscblast/Desktop/blastcam/static_hp_mask.txt"
#define dut1           -0.23

extern HIDS camera_handle;
extern int shutting_down;
extern int send_data;

// global structure for blob parameters
#pragma pack(push, 1)
struct blob_params {
    int spike_limit;            // where dynamic hot pixel will designate as hp
    int dynamic_hot_pixels;     // (bool) search for dynamic hot pixels
    int r_smooth;               // image smooth filter radius [px]
    int high_pass_filter;       // 0 == off, 1 == on
    int r_high_pass_filter;     // image high pass filter radius [px]
    int centroid_search_border; // px dist from image edge to start star search
    int filter_return_image;    // 1 == true; 0 = false
    float n_sigma;              // pixels > this*noise + mean = blobs
    int unique_star_spacing;    // min. pixel spacing between stars [px]
    int make_static_hp_mask;    // re-make static hp map with current image
    int use_static_hp_mask;     // flag to use the current static hot pixel map
};
#pragma pack(pop)

// make all blob_params acessible from any file that includes camera.h
extern struct blob_params all_blob_params;

int setCameraParams();
void setSaveImage();
int loadCamera();
int initCamera();
int doCameraAndAstrometry();
void clean();
void closeCamera();
const char * printCameraError();
int isLeapYear(int year);
void verifyBlobParams();
int makeTable(char * filename, double * star_mags, double * star_x, 
              double * star_y, int blob_count);
int findBlobs(char * input_buffer, int w, int h, double ** star_x, 
              double ** star_y, double ** star_mags, char * output_buffer);

#endif 