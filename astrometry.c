#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

#include <astrometry/os-features.h>
#include <astrometry/engine.h>
#include <astrometry/solver.h>
#include <astrometry/index.h>
#include <astrometry/starxy.h>
#include <astrometry/matchobj.h>
#include <astrometry/healpix.h>
#include <astrometry/bl.h>
#include <astrometry/log.h>
#include <astrometry/errors.h>
#include <astrometry/fileutils.h>
  
#include "/home/xscblast/astrometry/blind/solver.c"
#include "/home/xscblast/astrometry/blind/engine.c"

#include "camera.h"
#include "astrometry.h"

engine_t * engine = NULL;
solver_t * solver = NULL;

#define _USE_MATH_DEFINES
#define DRL_Lat (39 + (57/60) + (7/3600)) * M_PI/180   // StarCam Room: 39.95166666666667 * M_PI / 180 // normal decimal latitude of DRL in radians
#define DRL_Long -(75 + (11/60) + (18/3600))           // StarCam Room: -75.18944444444445      // normal decimal longitude of DRL (west is -, east +)
#define UT_C0 67310.54841
#define UT_C1 (876600.0*3600.0+8640184.812866)
#define UT_C2 .093104
#define UT_C3 (-6.2E-6)
#define UT_A0 36525.0

FILE * fptr;

// create instance of astrometry parameters global structure, accessible from telemetry.c as well
struct astro_params all_astro_params = {
  .logodds = 1e7,
  .latitude = DRL_Lat,
  .longitude = DRL_Long,
  .ra = 0, 
  .dec = 0,
  .fr = 0,
  .ps = 0,
  .ir = 0,
  .alt = 0,
  .az = 0,
};

void init_astrometry() {
	engine = engine_new();
	solver = solver_new();
	if (engine_parse_config_file(engine, "/usr/local/astrometry/etc/astrometry.cfg")) {
		printf("Bad configuration file in astrometry constructor.\n");
	}
}

void close_astrometry() {	
	engine_free(engine);
	solver_free(solver);
}

double siderealtime(struct tm * info) { //, double exposure_time_ms) {
	// initialize time variables and struct
	double juldate, T_UT1;
	double ThetaGMST;
	// for printing purposes
	char buff[100];
	double exposure_time_ms = 0.0;


	// compute the Julian date
	int jd = (14 - (info->tm_mon + 1))/12;        // tm_mon from 0-11 not 1-12
	int y = ((info->tm_year) + 1900) + 4800 - jd; // tm_year is years from 1900 not 0 (0 = 1 BC, 1 = 1 AD)
	int m = (info->tm_mon + 1) + (12*jd) - 3;

	// compute the Julian day	
	jd = info->tm_mday + ((153*m + 2)/5) + (365*y) + (y/4) - (y/100) + (y/400) - 32045; 
	juldate = jd + (((double) (info->tm_hour - 12 - info->tm_isdst))/24.0) + ((double) info->tm_min)/1440.0 + 
	                ((double) info->tm_sec + (exposure_time_ms/2000.0))/86400.0;
	printf("Julian date: %f\n", juldate);	
	// compute UT1 value
	T_UT1 = (juldate - 2451545.0)/UT_A0;
	// conversion based on 3rd order expansion (online):
	// http://www.mathworks.com/matlabcentral/fileexchange/26458-convert-right-ascension-and-declination-to-azimuth-and-elevation/content/RaDec2AzEl.m
	ThetaGMST = UT_C0 + UT_C1*T_UT1 + UT_C2*(T_UT1*T_UT1) + UT_C3*(T_UT1*T_UT1*T_UT1);
	ThetaGMST = fmod((fmod(ThetaGMST, 86400.0*(ThetaGMST/fabs(ThetaGMST)))/240.0), 360.0);

	// compute the sidereal time
	double LST = ThetaGMST + all_astro_params.longitude;
	// get LST within proper bounds (shift by 360 degrees if necessary)
	while (LST < 0) {
			LST += 360;
	}
	while (LST > 360) {
			LST -= 360;
	}

	strftime(buff, sizeof(buff), "%b %d %H:%M:%S", info); 
 	printf("GMT in siderealtime(), passed from doCameraAndAstrometry(): %s\n", buff); 

	return (LST);
}

void calc_alt(struct astro_params * params, struct tm * info) {
	// local variables
	double ra = params->ra;
	double dec = params->dec;
	dec *= M_PI/180; // convert dec to radians
	char datafile[100];

	// for printing purposes
	char buff[100];

	// calculate local sidereal time (LST)
	double LST = siderealtime(info);
	strftime(buff, sizeof(buff), "%b %d %H:%M:%S", info); 
	printf("GMT in calc_alt: %s\n", buff);
	printf("LST is %f degrees using siderealtime() method in calc_alt().\n", LST);
	strftime(datafile, sizeof(datafile), "/home/xscblast/Desktop/blastcam/data_%b-%d.txt", info); 
	fptr = fopen(datafile, "a");
	fprintf(fptr, "LST for this solution (corresponds to above GMT): %f\n", LST);
	fclose(fptr);
	memset(buff, 0, sizeof(buff));

    // find hour angle
    double ha = LST - ra;
    while (ha < 0) {
        ha += 360;
    }
	while (ha > 360) {
		ha -= 360;
	}
    ha *= M_PI / 180; // convert ha to radians

	// calculate the altitude 
    double alt = asin((sin(all_astro_params.latitude) * sin(dec)) + (cos(all_astro_params.latitude) * cos(dec) * cos(ha)));
    alt *= 180 / M_PI; // convert alt to degrees

	// update altitude field in global astro struct
	all_astro_params.alt = alt;
}

void calc_az(struct astro_params * params, struct tm * info) {
	// define local variables
    double dec = params->dec; 
    double ra = params->ra;
	double AZ;
	
	// calculate sidereal time (should be synchronized with LST in calc_alt())
	double LST = siderealtime(info);

    // find hour angle
    double ha = LST - ra;
    while (ha < 0) {
        ha += 360;
    }
	while (ha > 360) {
		ha -= 360;
	}
    ha *= M_PI / 180; // convert ha to radians

    double A = acos((sin(dec * M_PI/180) - (sin(all_astro_params.alt * M_PI/180) * sin(all_astro_params.latitude))) / 
	                (cos(all_astro_params.alt * M_PI/180) * cos(all_astro_params.latitude)));
    A *= 180 / M_PI; // convert A to degrees

	if (sin(ha) < 0) {
        AZ = A;
    } else {
        AZ = 360 - A;
    } 
	// update azimuth field in global astro params struct
	all_astro_params.az = AZ;
}

// this is the function to call for solving 
int lost_in_space_astrometry(double * starX, double * starY, double * starMag, unsigned numBlobs, struct tm * tm_info, char * datafile) {
	double ra, dec, fr, ps, ir;
	// localize some struct values for ease of calculations (will update at end of function)
	ra = all_astro_params.ra;
	dec = all_astro_params.dec;
	fr = all_astro_params.fr; 
	ps = all_astro_params.ps;
	ir = all_astro_params.ir;
	// create timer
	int msec = 0, trigger = 5000; // in milliseconds
	clock_t before = clock();
 	clock_t difference = clock() - before;
	msec = difference * 1000/CLOCKS_PER_SEC;
 	int diff = trigger - msec;

	// set up solver configuration
	solver->funits_lower = MIN_PS;
	solver->funits_upper = MAX_PS;
	
	// set max number of sources?
	solver->endobj = numBlobs;

	// disallow tiny quads
	solver->quadsize_min = 0.1 * MIN(CAMERA_WIDTH - 2*CAMERA_MARGIN, CAMERA_HEIGHT - 2*CAMERA_MARGIN);

	// set parity which can speed up x2
	solver->parity = PARITY_BOTH; // PARITY_NORMAL or PARITY_FLIP if that is the correct one
	// only PARITY_BOTH seems to work maybe?
	
	// sets the odds ratio we will accept
	solver_set_keep_logodds(solver, log(all_astro_params.logodds));  // previously 1e6 // now user-commandable

	solver->logratio_totune = log(1e6);
	solver->logratio_toprint = log(1e6);
	solver->distance_from_quad_bonus = 1;

	// figure out the index file range to search in
	double hprange = arcsec2dist(MAX_PS * hypot(CAMERA_WIDTH - 2*CAMERA_MARGIN, CAMERA_HEIGHT - 2*CAMERA_MARGIN)/2.0);

	// make list of stars
	starxy_t * field = starxy_new(numBlobs, 1, 0);

	starxy_set_x_array(field, starX);
	starxy_set_y_array(field, starY);
	starxy_set_flux_array(field, starMag);

	starxy_sort_by_flux(field);

	solver_set_field(solver, field);
	solver_set_field_bounds(solver, 0, CAMERA_WIDTH - 2*CAMERA_MARGIN, 0, CAMERA_HEIGHT - 2*CAMERA_MARGIN);

	// add index files that are close to the guess for the target
	for (int i = 0; i < (int)pl_size((*engine).indexes); i++) {
		index_t * index = (index_t*) pl_get((*engine).indexes, i);
		// check the Astrometry timeout
		if (i % 10 == 0) { 
			difference = clock() - before;
  			msec = difference * 1000/CLOCKS_PER_SEC;
  			diff = trigger - msec;
			if (diff < 0) {
				printf("Astrometry timed out.\n");
				return 0;
			}
		}

		if (ra >= -180 && dec >= -90 && index_is_within_range(index, ra, dec, dist2deg(hprange))) {
			solver_add_index(solver, index);
		} else if (ra < -180 || dec < -90) {
			solver_add_index(solver, index);
		}
		index_reload(index);
	}

	solver_log_params(solver);
	solver_run(solver);

	int ret = 0;

	if ((*solver).best_match_solves) {
		double pscale;
		tan_t * wcs;

		wcs = &((*solver).best_match.wcstan);
		tan_pixelxy2radec(wcs, (CAMERA_WIDTH - 2*CAMERA_MARGIN - 1)/2.0, (CAMERA_HEIGHT - 2*CAMERA_MARGIN - 1)/2.0, &ra, &dec);
		// calculate pixel scale and field rotation
		ps = tan_pixel_scale(wcs);
		fr = tan_get_orientation(wcs); 
	
		// update parameters in astrometry global structure
		all_astro_params.ra = ra;
		all_astro_params.dec = dec;
		all_astro_params.fr = fr;
		all_astro_params.ps = ps;

		// calculate altitude and azimuth
		calc_alt(&all_astro_params, tm_info);
		calc_az(&all_astro_params, tm_info);
		all_astro_params.ir = 15.04106858*cos(all_astro_params.az)*cos(all_astro_params.az);
		printf("Astrometry: RA %lf | DEC %lf | FR %f | PS %lf | ALT %lf | AZ %lf | IR %lf\n", 
		        all_astro_params.ra, all_astro_params.dec, all_astro_params.fr, all_astro_params.ps, all_astro_params.alt, 
				all_astro_params.az, all_astro_params.ir);
		// calculate how long solution takes to solve
		difference = clock() - before;
  		msec = difference * 1000/CLOCKS_PER_SEC;
  		diff = trigger - msec;
		int completion = 5000 - diff;
		printf("Solved in %d msec.\n", completion);
		// write astrometry solution to data.txt file
		printf("Writing astrometry solution to data file...\n");
		fptr = fopen(datafile, "a");
		fprintf(fptr, "Astrometry: RA %lf | DEC %lf | FR %lf | PS %lf | ALT %lf | AZ %lf | IR %lf\nSolved in %d msec.\n", 
		        all_astro_params.ra, all_astro_params.dec, all_astro_params.fr, all_astro_params.ps, all_astro_params.alt, 
				all_astro_params.az, all_astro_params.ir, completion);
		fclose(fptr); // close the file
		ret = 1;
	} 

	solver_cleanup_field(solver);
	solver_clear_indexes(solver);

	return ret;
}