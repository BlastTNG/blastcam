/* Collects the user-requested data (specified via commands.c) from the other StarCamera functions. 
   Sends this data back to commands.c. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lens_adapter.h"
#include "telemetry.h"
#include "astrometry.h"
#include "camera.h"
#include "commands.h"

// astrometry parameters global structure
struct astro_params all_astro_params;

void printArr(float * arr) {
    int loop;
    // six is hard-coded now as the max number of telemetry data points being output
    for (loop = 0; loop < 9; loop++) {
        printf("%f\n", arr[loop]);
    }
}

double get_ra() {
    return all_astro_params.ra;
}

double get_dec() {
    return all_astro_params.dec;
}

double get_fr() {
    return all_astro_params.fr;
}

double get_az() {
    //double az = calc_az(&all_astro_params);
    return all_astro_params.az;
}

double get_elev() {
    //double elev = calc_alt(&all_astro_params);
    return all_astro_params.alt;
}

double get_ir() {
    return all_astro_params.ir;
}

// int get_curr_focus() {
//     printf("curr focus in astro params %i\n", all_astro_params.current_focus);
//     return all_astro_params.current_focus;
// }

// int get_curr_aperture() {
//     return all_astro_params.current_aperture;
// }

// int get_zero_focus() {
//     printf("min focus in astro params: %i\n", all_astro_params.zero_focus_pos);
//     return all_astro_params.zero_focus_pos;
// }

// int get_max_focus() {
//     printf("max focus in astro params %i\n", all_astro_params.max_focus_pos);
//     return all_astro_params.max_focus_pos;
// }

// int get_inf_focus_bool() {
//     return all_astro_params.focus_inf;
// }

// int get_max_aper_bool() {
//     return all_astro_params.max_aperture;
// }

// not currently used
float * get_telemetry(float * arr) {
    arr[0] = get_ra();
    arr[1] = get_dec();
    arr[2] = get_fr();
    arr[3] = get_az();
    arr[4] = get_elev();
    arr[5] = get_ir();
    return arr;
}