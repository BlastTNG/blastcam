#ifndef CAMERA_H
#define CAMERA_H

#define CAMERA_WIDTH 1936 	// [px]
#define CAMERA_HEIGHT 1216	// [px]
#define CAMERA_MARGIN 0		// [px]
#define MIN_PS 6.0        	// [as/px]
#define NOM_PS 7.66			// [as/px]
#define MAX_PS 8.0			// [as/px]

// global structure for blob parameters
#pragma pack(push, 1)
struct blob_params {
  int spike_limit; // how agressive is the dynamic hot pixel finder.  Small is more agressive
  int dynamic_hot_pixels; // 0 == off, 1 == on
  int r_smooth;   // image smooth filter radius [px]
  int high_pass_filter; // 0 == off, 1 == on
  int r_high_pass_filter; // image high pass filter radius [px]
  int centroid_search_border; // distance from image edge from which to start looking for stars [px]
  int filter_return_image; // 1 == true; 0 = false
  float n_sigma; // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
  int unique_star_spacing;
};
typedef struct {
    unsigned char  b;
    unsigned char  g;
    unsigned char  r;
} IMAGE;
#pragma pack(pop)

// make all blob_params acessible from any file that includes camera.h
extern struct blob_params all_blob_params;
//extern struct IMAGE_FILE_PARAMS ImageFileParams;
// make image data structure accessible as well
//extern BITMAPFILEHEADER bitmapFileHeader;
//extern BITMAPINFOHEADER bitmapInfoHeader;

// define function prototypes
void set_camera_params(unsigned int cameraHandle);
int saveImage();
int whileLoop();
int load_camera();
void init_camera();
// unsigned char * LoadBitmapFile(char *filename, BITMAPINFOHEADER *bitmapInfoHeader);
void doCameraAndAstrometry();
void clean_up();

#endif /* CAMERA_H */