#ifndef ASTROMETRY_H
#define ASTROMETRY_H

// define function prototypes
void init_astrometry();
void close_astrometry();
int lost_in_space_astrometry(double * starX, double * starY, double * starMag, unsigned numBlobs, struct tm * tm_info, char * datafile);

// global structure for astrometry parameters
#pragma pack(push, 1)
struct astro_params {
  double logodds;
  double latitude;
  double longitude;
  double ra;
  double dec;
  double fr;
  double ps;
  double ir;
  double alt;
  double az;
};
#pragma pack(pop)

// make all blob_params acessible from any file that includes camera.h
extern struct astro_params all_astro_params;

double siderealtime(struct tm * info, double exposure_time_ms);
void calc_az(struct astro_params * params, struct tm * tm_info);
void calc_alt(struct astro_params * params, struct tm * tm_info);

#endif 