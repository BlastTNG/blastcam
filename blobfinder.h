#pragma once

//#define NO_PARAMHANDLER

#include "../bitsc/camera_defs.h"
#include "../bitsc/locatorStructs.h"

#ifndef NO_PARAMHANDLER
#include "../bitsc/paramHandler.h"
#endif

#include "../bitsc/astrometry.h"

#define FILTERS_OFF     0
#define MASK_FILTER     1
#define BOXCAR_FILTER   2
#define BOTH_FILTER     3

#ifdef IMAGES16BIT
typedef unsigned short imageType;
typedef int imageSumType;
#else
typedef unsigned char imageType;
typedef short imageSumType;
#endif

#ifdef __cplusplus


class BlobFinder {
  public:
    BlobFinder(int camera); // allocmem
    ~BlobFinder(); // free dem memoizes

    // write out the blob list: for use by lostInSpace among others
    void writeData(std::vector<blob> blobs, const char *filename, unsigned int n_blobs, bool remove_margin);

    // where are we???
    location lostInSpace(int timeout, imageType *image, location guess);

    // find blobs.  x_size==0 means whole image.
    // x_center and y_center are the center of the subframe.
    // x_size and y_size are the full width and height of the the subframe.
    // (center - size) is added to the blob locations.
    // center and size should all be 0 if doing full image.
    void getBlobs(imageType *image, int x_center, int y_center, int x_size, int y_size, std::vector<blob> &blobs, imageType* output_buffer);
    void getBlobsOld(imageType *image, int x_center, int y_center, int x_size, int y_size, std::vector<blob> &blobs, imageType* output_buffer);

#ifndef NO_PARAMHANDLER
    int setParams(int mode, ParamHandler* params);
#endif
    int setCParams(double n_sig, double sl, int r_cent, int hpf, int filterType, int filterReturn, int border);
    void setNumBlobs(int numBlobs);
    void loadHPList();
		int blob_mask_margin;
    bool use_old_blobfinder;
  private:
    void makeMask(imageType *image, int i0, int j0, int i1, int j1, int x0, int y0, bool subframe);
    void boxcarFilterImage(imageType *ib, int i0, int j0, int i1, int j1, int r_f, double *filtered_image);
    int Box1DCenter(double *x, int nx, int filt, int res, int &max_out);
    int w;
    int h;
    double n_sigma; // pixels brighter than this time the noise in the filtered map are blobs.
    int r_centroid; // border added to make sure we don't search past the edge (misnamed)
    int r_smooth;   // image high pass filter radius
    int border;     // border for full frame search
    bool high_pass_filter; // apply a HPF before full frame search
    bool filter_return_image; // return image is filtered.
    std::vector<icoords> hpList;
    double spike_limit; // how agressive is the dynamic hot pixel finder.  Small is more agressive
    bool dynamic_hot_pixels_full_frame; // search the image for isolated spikes in full frames
    bool dynamic_hot_pixels_sub_frame;  // search the image for isolated spikes in subframes
    bool use_hpList; // use a hot pixel list for the full frame images.

    char camera_name[8]; // "bore" or "roll"

    imageType *nc;
    imageSumType *ibc1;
    double *ic;
    double *ic2;
    unsigned char *mask;

    int blobsToWrite;
    int bin_factor;

    int camera;
    Astrometry* astrometry;
};

#endif

//some suspicious nonsense
#ifdef __cplusplus
extern "C" 
#endif
void* make_blobfinder(int camera);

#ifdef __cplusplus
extern "C" 
#endif
void blobfinder_getBlobs(void* fakeClass, imageType* picture, int x_center, int y_center, int x_size, int y_size, struct blob* blobs, int numBlobs, imageType* outBuffer);

#ifdef __cplusplus
extern "C" 
#endif
struct location blobfinder_lostInSpace(void* fakeClass, int timeout, imageType* image, struct location guess);

#ifdef __cplusplus
extern "C" 
#endif
void blobfinder_setParams(void* fakeClass, double n_sig, double sl, int r_cent, int hpf, int filterType, int filterReturn, int border);
