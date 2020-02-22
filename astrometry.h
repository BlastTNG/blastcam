#ifndef ASTROMETRY_H
#define ASTROMETRY_H

// define function prototypes
void init_astrometry();
void close_astrometry();
int lost_in_space_astrometry(double * starX, double * starY, double * starMag, unsigned numBlobs, time_t rawtime);

// global structure for astrometry parameters
#pragma pack(push, 1)
struct astro_params {
  double latitude;
  double longitude;
  double ra;
  double dec;
  double fr;
  double ps;
  double ir;
  double alt;
  double az;
  // information fields for camera settings
  // int current_focus;
  // int zero_focus_pos;
  // int max_focus_pos;
  // int focus_inf;
  // int max_aperture;
  // int current_aperture;
  // information fields for blob parameters
  // int spike_limit; // how agressive is the dynamic hot pixel finder.  Small is more agressive
  // int dynamic_hot_pixels; // 0 == off, 1 == on
  // int r_smooth;   // image smooth filter radius [px]
  // int high_pass_filter; // 0 == off, 1 == on
  // int r_high_pass_filter; // image high pass filter radius [px]
  // int centroid_search_border; // distance from image edge from which to start looking for stars [px]
  // int filter_return_image; // 1 == true; 0 = false
  // double n_sigma; // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
};
#pragma pack(pop)

// make all blob_params acessible from any file that includes camera.h
extern struct astro_params all_astro_params;

void calc_az(struct astro_params * params, time_t rawtime);
void calc_alt(struct astro_params * params, time_t rawtime);
int jday(int month, int day);

#endif /* ASTROMETRY_H */