#ifndef ASTROMETRY_H
#define ASTROMETRY_H

int initAstrometry();
void closeAstrometry();
int lostInSpace(double * star_x, double * star_y, double * star_mags, 
                unsigned num_blobs, struct tm * tm_info, char * datafile);

// global structure for astrometry parameters
#pragma pack(push, 1)
struct astrometry {
    double timelimit;
    double rawtime;
    // needed for solving altaz
    double logodds;
    double latitude;
    double longitude;
    double hm;
    // astrometry solution
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
extern struct astrometry all_astro_params;

#endif 