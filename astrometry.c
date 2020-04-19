// include necessary libraries and astrometry code (solver.c, engine.c)
#include <stdlib.h>
#include <math.h>
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
#include <ueye.h>
#include <sofa/sofa.h>
#include "/home/xscblast/astrometry/blind/solver.c"
#include "/home/xscblast/astrometry/blind/engine.c"

// include our header files
#include "camera.h"
#include "astrometry.h"
#include "lens_adapter.h"

#define _USE_MATH_DEFINES
// more precise longitude and latitude constants (deg)
#define lat84   40.79243469238281
#define long84 -73.68112182617188 // (west is -, east +)
#define hm84    57.77

engine_t * engine = NULL;
solver_t * solver = NULL;

FILE * fptr;

// create instance of astrometry parameters global structure, accessible from telemetry.c as well
struct astro_params all_astro_params = {
	.rawtime = 0,
	.logodds = 1e8,
	.latitude = lat84,
	.longitude = long84,
	.hm = hm84,
	.ra = 0, 
	.dec = 0,
	.fr = 0,
	.ps = 0,
	.ir = 0,
	.alt = 0,
	.az = 0,
};

// function to initialize astrometry
void init_astrometry() {
	engine = engine_new();
	solver = solver_new();
	if (engine_parse_config_file(engine, "/usr/local/astrometry/etc/astrometry.cfg")) {
		printf("Bad configuration file in Astrometry constructor.\n");
	}
}

// function to close astrometry
void close_astrometry() {	
	engine_free(engine);
	solver_free(solver);
}

// function solving for pointing location on the sky
int lost_in_space_astrometry(double * starX, double * starY, double * starMag, unsigned numBlobs, 
                             struct tm * tm_info, char * datafile) {
	double ra, dec, fr, ps, ir;

	// TODO: do we need this?
	// localize some struct values for ease of calculations (will update struct fields at end of function)
	// ra = all_astro_params.ra;
	// dec = all_astro_params.dec;
	// fr = all_astro_params.fr; 
	// ps = all_astro_params.ps;
	// ir = all_astro_params.ir;

	// create timer
	int msec = 0, trigger = 5000;            // in milliseconds
	clock_t before = clock();
 	clock_t difference = clock() - before;
	msec = difference*1000/CLOCKS_PER_SEC;
 	int diff = trigger - msec;

	// set up solver configuration
	solver->funits_lower = MIN_PS;
	solver->funits_upper = MAX_PS;
	
	// set max number of sources?
	solver->endobj = numBlobs;

	// disallow tiny quads
	solver->quadsize_min = 0.1*MIN(CAMERA_WIDTH - 2*CAMERA_MARGIN, CAMERA_HEIGHT - 2*CAMERA_MARGIN);

	// set parity which can speed up x2
	solver->parity = PARITY_BOTH;           // PARITY_NORMAL or PARITY_FLIP if that is the correct one
	                                        // only PARITY_BOTH seems to work maybe?
	
	// sets the odds ratio we will accept (logodds parameter)
	solver_set_keep_logodds(solver, log(all_astro_params.logodds));  

	solver->logratio_totune = log(1e6);
	solver->logratio_toprint = log(1e6);
	solver->distance_from_quad_bonus = 1;

	// figure out the index file range to search in
	double hprange = arcsec2dist(MAX_PS*hypot(CAMERA_WIDTH - 2*CAMERA_MARGIN, CAMERA_HEIGHT - 2*CAMERA_MARGIN)/2.0);

	// make list of stars
	starxy_t * field = starxy_new(numBlobs, 1, 0);

	starxy_set_x_array(field, starX);
	starxy_set_y_array(field, starY);
	starxy_set_flux_array(field, starMag);

	starxy_sort_by_flux(field);

	solver_set_field(solver, field);
	solver_set_field_bounds(solver, 0, CAMERA_WIDTH - 2*CAMERA_MARGIN, 0, CAMERA_HEIGHT - 2*CAMERA_MARGIN);

	// add index files that are close to the guess for the target
	for (int i = 0; i < (int) pl_size((*engine).indexes); i++) {
		index_t * index = (index_t *) pl_get((*engine).indexes, i);

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

	// comment out when not testing fixed time stamps!
	(*solver).best_match_solves = 1;

	int ret = 0;
	if ((*solver).best_match_solves) {
		double pscale;
		tan_t * wcs;

		wcs = &((*solver).best_match.wcstan);
		tan_pixelxy2radec(wcs, (CAMERA_WIDTH - 2*CAMERA_MARGIN - 1)/2.0, (CAMERA_HEIGHT - 2*CAMERA_MARGIN - 1)/2.0, &ra, &dec);
		// calculate pixel scale and field rotation
		ps = tan_pixel_scale(wcs);
		fr = tan_get_orientation(wcs); 
	
		// for testing previous observing data
		ra = currRA;
		dec = currDEC;

		// calculate altitude and azimuth using top-level SOFA function.
		double d1, d2;
		int sofa_juldate_status = iauDtf2d("UTC", tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday, tm_info->tm_hour, 
	                                              tm_info->tm_min, (double) tm_info->tm_sec, &d1, &d2);	
		if (sofa_juldate_status != 0) {
			printf("Julian date not properly calculated.\n");
		}

		double aob, zob, hob, dob, rob, eo;
		iauAtco13(ra*(M_PI/180.0), dec*(M_PI/180.0), 
		          0.0, 0.0, 0.0, 0.0, 
				  d1, d2 + (all_camera_params.exposure_time/(2000.0*3600.0*24.0)), curr_dut1, 
				  all_astro_params.longitude*(M_PI/180.0), all_astro_params.latitude*(M_PI/180.0), all_astro_params.hm, 0.0, 0.0, 
				  0.0, 0.0, 0.0, 0.0,
				  &aob, &zob, &hob, &dob, &rob, &eo);
		/*****************************************************************/

		// TODO: verify this calculation (cos^2(az)?)
		all_astro_params.ir = 15.04106858*cos(all_astro_params.az)*cos(all_astro_params.az);
		all_astro_params.ra = rob*(180.0/M_PI);
		all_astro_params.dec = dob*(180.0/M_PI);
		all_astro_params.alt = 90.0 - (zob*(180.0/M_PI));
		all_astro_params.az = aob*(180.0/M_PI); 
		all_astro_params.fr = fr;
		all_astro_params.ps = ps;

		printf("\n****************************************** TELEMETRY ******************************************\n");
		printf("Obs. RA %lf | Obs. DEC %lf | FR %f | PS %lf | ALT %.15f | AZ %.15f | IR %lf\n", 
		        all_astro_params.ra, all_astro_params.dec, all_astro_params.fr, all_astro_params.ps, all_astro_params.alt, 
				all_astro_params.az, all_astro_params.ir);
		printf("***********************************************************************************************\n\n");

		// calculate how long solution took to solve
		difference = clock() - before;
  		msec = difference*1000/CLOCKS_PER_SEC;
  		diff = trigger - msec;
		int completion = 5000 - diff;
		printf("Solved in %d msec.\n", completion);

		// write astrometry solution to data.txt file
		printf("Writing Astrometry solution to data file...\n");
		fptr = fopen(datafile, "a");
		fprintf(fptr, "%lf|%lf|%lf|%lf|%.15f|%.15f|%lf|%d", 
		        all_astro_params.ra, all_astro_params.dec, all_astro_params.fr, all_astro_params.ps, all_astro_params.alt, 
				all_astro_params.az, all_astro_params.ir, completion);
		fclose(fptr);
		ret = 1;
	} 
	solver_cleanup_field(solver);
	solver_clear_indexes(solver);
	return ret;
}