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

#define M_PI 3.14159265358979323846
#define DRL_Lat 39.952099 * M_PI / 180 // normal decimal latitude of DRL in radians
#define DRL_Long -75.189507            // normal decimal longitude of DRL (west is -, east +)

FILE * fptr;

// create instance of astrometry parameters global structure, accessible from telemetry.c as well
struct astro_params all_astro_params = {
  .latitude = DRL_Lat,
  .longitude = DRL_Long,
  .ra = 0, 
  .dec = 0,
  .fr = 0,
  .ps = 0,
  .ir = 0,
  .alt = 0,
  .az = 0,
//   .ha = 0, // included as field for help in passing proper information to calc_az() (hour angle)
//   .current_focus = 0,
//   .zero_focus_pos = 0,
//   .max_focus_pos = 0,
//   .focus_inf = 0,
//   .max_aperture = 0,
//   .current_aperture = 0,
//   .spike_limit = 4.0, // how agressive is the dynamic hot pixel finder.  Small is more agressive
//   .dynamic_hot_pixels = 0, // 0 == off, 1 == on
//   .r_smooth = 2,   // image smooth filter radius [px]
//   .high_pass_filter = 0, // 0 == off, 1 == on
//   .r_high_pass_filter = 10, // image high pass filter radius [px]
//   .centroid_search_border = 1, // distance from image edge from which to start looking for stars [px]
//   .filter_return_image = 0, // 1 == true; 0 = false
//   .n_sigma = 2.0, // pixels brighter than this time the noise in the filtered map are blobs (this number * sigma + mean)
//   .unique_star_spacing = 15,
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

int jday(int month, int day) {
	/* Days in each month 1-12 */
	int jday[] = {0,0,31,59,90,120,151,180,211,242,272,303,333 };

	/* Determin decimal years from date */
	return(jday[month]+day);
}

void calc_alt(struct astro_params * params, time_t rawtime) {
	// access ra and dec fields from the astro_params struct and initialize variables for internal calculations
    double dec = params->dec; // pointer for astro_params requires ->
    double ra = params->ra;

    dec *= M_PI / 180;        // dec of saved image converted to radians
    //time_t rawtime;
    //time(&rawtime);
    struct tm *info;
    info = gmtime(&rawtime);

	int YEAR = info->tm_year + 1900;
	int DAY = jday(info->tm_mon, info->tm_mday);
	double HOUR = info->tm_hour + (info->tm_min/60.0) + (info->tm_sec/3600.0);

	// ** current Julian date (actually add 2,400,000
	// ** for true JD); LEAP = leap days since 1949;
	// ** 32916.5 is midnite 0 jan 1949 minus 2.4e6
	int DELTA = YEAR - 1949;
	double LEAP = DELTA / 4;
	double JD = 32916.5 + ((DELTA * 365.0) + LEAP + DAY) + HOUR / 24.0 +1;
	// ** ecliptic coordinates
	// ** 51545.0 + 2.4e6 = noon 1 jan 2000
	double TIME = JD - 51545.0;
	// ** Greenwich mean sidereal time in hours
	double GMST = 6.697375 + 0.0657098242*TIME + HOUR;
	
	// ** Hour not changed to sidereal time since
	// ** 'time' includes the fractional day
	GMST = fmod( GMST, 24.0 );
	if( GMST < 0.0 ){
		GMST = GMST + 24.0;
	}
	// ** local mean sidereal time in radians
	double LMST = GMST + (all_astro_params.longitude / 15.0);
	LMST = fmod( LMST, 24.0 );
	if( LMST < 0.0 ){
	    LMST = LMST + 24.0;
	}
	printf("LMST is %f hours\n", LMST);
	LMST = LMST*15.0;
	printf("LMST is %f degrees\n", LMST);

    // // calculate local siderial time (stargazing.net method)
    // double lst = 100.46 + (0.985647 * totalTimeSince2000) + DRL_Long + (15 * decimalUT);
    // while (lst > 360) {
    //     lst -= 360; // to get within 360 degrees
    // } 
	// lst currently in degrees
    // lst /= 15; // lst to hours
    //lst *= 15; //lst to degrees

	// alternative calculation of lst: LST = GST + longitude of observer
	// lst = gst + DRL_Long;

    // find hour angle
    double ha = LMST - ra;
    if (ha < 0) {
        ha += 360;
    }
    ha *= M_PI / 180; // convert to radians

    double alt = asin((sin(all_astro_params.latitude) * sin(dec)) + (cos(all_astro_params.latitude) * cos(dec) * cos(ha)));
    alt *= 180 / M_PI; // convert to degrees

	all_astro_params.alt = alt;
	// all_astro_params.ha = ha;

	//return alt;
}

void calc_az(struct astro_params * params, time_t rawtime) {
    double dec = params->dec; // pointer for astro_params requires ->
    double ra = params->ra;

	// double altitude = params->alt;
	// double ha = params->ha;

    //time_t rawtime;
    //time(&rawtime);
    struct tm *info;
    info = gmtime(&rawtime);
    int currentHour = info->tm_hour;
    int currentMinute = info->tm_min;
    int currentSecond = info->tm_sec;
	// printf("%d\n", currentSecond);
    //rawtime = time(NULL);
    int daysSinceEpoch1970 = rawtime/(60*60*24);
    double daysBetween1970and2000 = 10957 + .5; // .5 is added because epoch 2000 starts at midday 1/1/2000
    double daysSince2000 = daysSinceEpoch1970 - daysBetween1970and2000;
    double decimalUT = currentHour + (currentMinute / 60.0) + (currentSecond / 3600.0); //.0 makes double

    double totalTimeSince2000 = daysSince2000 + (decimalUT / 24);

    // calculate local siderial time
    double lst = 100.46 + (.985647 * totalTimeSince2000) + all_astro_params.longitude + (15 * decimalUT);
    while (lst > 360) {
        lst -= 360; // to get within 360 degrees
    } 
	// lst currently in degrees
    lst /= 15; // lst to hours // 
	printf("%f hours\n", lst);
    // lst *= 15; // lst to degrees

    // find hour angle
    double ha = lst - ra;
    if (ha < 0) {
        ha += 360;
    }
	printf("ra: %f\n", ra);
	printf("hour angle: %f\n", ha);
    ha *= M_PI / 180; // convert to radians

    double alt = asin((sin(all_astro_params.latitude) * sin(dec)) + (cos(all_astro_params.latitude) * cos(dec) * cos(ha)));
    alt *= 180 / M_PI; // convert to degrees
    alt *= M_PI / 180; // convert input altitude to radians

    double A = acos((sin(dec) - (sin(alt) * sin(all_astro_params.latitude))) / (cos(alt) * cos(all_astro_params.latitude)));
    A *= 180 / M_PI; // convert to degrees
    double AZ;
    double sinHA = sin(ha);
    sinHA *= M_PI / 180; // convert to degrees
	if (sinHA < 0) {
        AZ = A;
    } else {
        AZ = 360 - A;
    } 
	
	all_astro_params.az = AZ;
    //return AZ;
}

// this is the function to call for solving :)  
int lost_in_space_astrometry(double * starX, double * starY, double * starMag, unsigned numBlobs, time_t rawtime) {
	double ra, dec, fr, ps, ir;
	ra = all_astro_params.ra;
	dec = all_astro_params.dec;
	fr = all_astro_params.fr; 
	ps = all_astro_params.ps;
	ir = all_astro_params.ir;

	// create timer
	int msec = 0, trigger = 5000; /* ms */
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
	solver_set_keep_logodds(solver, log(1e6));

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
	// int i = 0;
	// while(i < (int)pl_size((*engine).indexes) && diff > 0)
	for (int i = 0; i < (int)pl_size((*engine).indexes); i++) {
		index_t* index = (index_t*) pl_get((*engine).indexes, i);
		if (i%10 == 0) { // check timeout
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
		printf("ra within lost_in_space: %f\n", ra);
		ps = tan_pixel_scale(wcs);
		fr = tan_get_orientation(wcs); 
	
		// update parameters in astrometry global structure for telemetry.c
		all_astro_params.ra = ra;
		all_astro_params.dec = dec;
		all_astro_params.fr = fr;
		all_astro_params.ps = ps;

		//double alt_to_print = calc_alt(&all_astro_params, rawtime);
		//double az_to_print = calc_az(&all_astro_params, rawtime);
		calc_alt(&all_astro_params, rawtime);
		calc_az(&all_astro_params, rawtime);
		all_astro_params.ir = 15.04106858*cos(all_astro_params.az)*cos(all_astro_params.az);
		printf("lost_in_space_astrometry solution found at ra %lf, dec %lf, fr %f, pixel scale %lf, alt %lf, az %lf, ir %lf.\n", 
		        all_astro_params.ra, all_astro_params.dec, all_astro_params.fr, all_astro_params.ps, all_astro_params.alt, 
				all_astro_params.az, all_astro_params.ir);
		fptr = fopen("/home/xscblast/Desktop/blastcam/data.txt", "a");
		fprintf(fptr, "lost_in_space_astrometry solution found at ra %lf, dec %lf, fr %f, pixel scale %lf, alt %lf, az %lf, ir %lf.\n", 
		        all_astro_params.ra, all_astro_params.dec, all_astro_params.fr, all_astro_params.ps, all_astro_params.alt, 
				all_astro_params.az, all_astro_params.ir);
		difference = clock() - before;
  		msec = difference * 1000/CLOCKS_PER_SEC;
  		diff = trigger - msec;
		int completion = 5000 - diff;
		printf("Solved in %d msec.\n", completion);
		ret = 1;
	} 

	solver_cleanup_field(solver);
	solver_clear_indexes(solver);

	return ret;
}