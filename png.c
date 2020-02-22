#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <pthread.h>
#include <sys/time.h>
#include <dirent.h>
#include <sched.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <ueye.h>
#include <stdbool.h>
#include <limits.h>
#include <float.h>
#include "png.h"
#include "readpng.h"


#include "camera.h"
#include "lens_adapter.h"
#include "astrometry.h"
    
IMAGE_FILE_PARAMS ImageFileParams;

int status;

int spike_limit = 1.35; // how agressive is the dynamic hot pixel finder.  Small is more agressive
int dynamic_hot_pixels = 1; // 0 == off, 1 == on
int r_smooth = 2;   // image smooth filter radius [px]
int high_pass_filter = 0; // 0 == off, 1 == on
int r_high_pass_filter = 10; // image high pass filter radius [px]
int centroid_search_border = 1; // distance from image edge from which to start looking for stars [px]
int filter_return_image = 1; // 1 == true; 0 = false
double n_sigma = 6; // pixels brighter than this time the noise in the filtered map are blobs
int unique_star_spacing = 15; // stars closer together than this value are considered one star

unsigned char *mask = NULL;


void makeMask(char *ib, int i0, int j0, int i1, int j1, int x0, int y0, bool subframe)
{
  static int first_time = 1;

  if (first_time) {
    mask = calloc(1, CAMERA_WIDTH*CAMERA_HEIGHT);
    first_time = 0;
  }

  int i, j;
  int p0, p1, p2, p3, p4;
  int a, b;
  int cutoff = spike_limit*100.0;

  for (i=i0; i<i1; i++) {
    mask[i+ CAMERA_WIDTH*j0] = mask[i + (j1-1)*CAMERA_WIDTH] = 0;
  }
  for (j=j0; j<j1; j++) {
    mask[i0+j*CAMERA_WIDTH] = mask[i1-1 + j*CAMERA_WIDTH] = 0;
  }

  i0++;
  j0++;
  i1--;
  j1--;
  
  if (dynamic_hot_pixels) {
    int nhp = 0;
    for (j = j0; j<j1; j++) {
      for (i=i0; i<i1; i++) {
        p0 = 100*ib[i+j*CAMERA_WIDTH]/cutoff;
        p1 = ib[i-1+(j)*CAMERA_WIDTH];
        p2 = ib[i+1+(j)*CAMERA_WIDTH];
        p3 = ib[i+(j+1)*CAMERA_WIDTH];
        p4 = ib[i+(j-1)*CAMERA_WIDTH];
        a = p1+p2+p3+p4+4;
        p1 = ib[i-1+(j-1)*CAMERA_WIDTH];
        p2 = ib[i+1+(j+1)*CAMERA_WIDTH];
        p3 = ib[i-1+(j+1)*CAMERA_WIDTH];
        p4 = ib[i+1+(j-1)*CAMERA_WIDTH];
        b = p1+p2+p3+p4+4;
        mask[i+j*CAMERA_WIDTH] = ((p0 < a) && (p0 < b));
        if (p0>a || p0 > b) nhp++;
      }
    }
    //printf("Dynamic hot pixels (%d)\n", nhp);
  } else {
    for (j = j0; j<j1; j++) {
      for (i=i0; i<i1; i++) {
        mask[i+j*CAMERA_WIDTH] = 1;
      }
    }
  }

}

void boxcarFilterImage(char *ib, int i0, int j0, int i1, int j1, int r_f, double *filtered_image) {

  static int first_time = 1;
  static char *nc = NULL;
  static uint64_t * ibc1 = NULL;

  if (first_time) {
    nc = calloc(1, CAMERA_WIDTH*CAMERA_HEIGHT);
    ibc1 = calloc(sizeof(uint64_t), CAMERA_WIDTH*CAMERA_HEIGHT);
    first_time = 0;
  }

  //printf("in boxcar filter, w = %d, h = %d, r_f = %d\n", w, h, r_f);
  int b = r_f;
  uint64_t isx;
  unsigned int s, n;
  double ds, dn;
  double last_ds=0;

  for (int j = j0; j<j1; j++) {
    n = 0;
    isx = 0;
    for (int i = i0; i<i0+2*r_f+1; i++) {
      n += mask[i+j*CAMERA_WIDTH];
      isx += ib[i+j*CAMERA_WIDTH]*mask[i+j*CAMERA_WIDTH];
    }
    int idx = CAMERA_WIDTH*j+i0+r_f;
    //int iN = i1-r_f-1;
    for (int i=r_f+i0; i<i1-r_f-1; i++) {
      ibc1[idx] = isx;
      nc[idx] = n;
      isx = isx - mask[idx-r_f]*ib[idx-r_f] + mask[idx+r_f+1]*ib[idx+r_f+1];
      n = n - mask[idx-r_f] + mask[idx+r_f+1];
      idx++;
    }
    ibc1[idx] = isx;
    nc[idx] = n;
  }
  for (int j = j0+b; j<j1-b; j++) {
    for (int i=i0+b; i<i1-b; i++) {
      n = s = 0;
      for (int jp=-r_f; jp<=r_f; jp++) {
        int idx = i + (j+jp)*CAMERA_WIDTH;
        s += ibc1[idx];
        n += nc[idx];
      }
      ds = s;
      dn = n;
      if (dn>0.0) {
        ds/=dn;
        last_ds = ds;
      } else {
        ds = last_ds;
      }

      filtered_image[i+j*CAMERA_WIDTH] = ds;
    }
  }
}


int get_blobs(char *input_buffer, int x_center, int y_center, int w, int h, double ** starX, double ** starY, double ** starMag, char* output_buffer)
{

  static int first_time = 1;
  static double * ic = NULL;
  static double * ic2 = NULL;
  static int num_blobs_alloc = 0;

  if (first_time) {
    ic = calloc(sizeof(double), CAMERA_WIDTH*CAMERA_HEIGHT);
    ic2 = calloc(sizeof(double), CAMERA_WIDTH*CAMERA_HEIGHT);
    first_time = 0;
  }

  // we use half-width internally, but the api gives us
  // full width.
  int x_size = w/2;
  int y_size = h/2;

  int j0, j1, i0, i1;
  int b = 0; // extra image border

  j0 = i0 = 0;
  j1 = h;
  i1 = w;
  b = centroid_search_border;

  makeMask(input_buffer, i0, j0, i1, j1, 0, 0, 0);

  double sx=0, sx2=0;
  double sx_raw = 0; // sum of non-highpass filtered field
  int num_pix=0;

  // lowpass filter the image - reduce noise.
  boxcarFilterImage(input_buffer, i0, j0, i1, j1, r_smooth, ic);

  if (high_pass_filter) { // only high-pass filter full frames
    //fprintf (stderr, "hp filter active\n");
    b += r_high_pass_filter;
    boxcarFilterImage(input_buffer, i0, j0, i1, j1, r_high_pass_filter, ic2);

    for (int j = j0+b; j<j1-b; j++) {
      for (int i = i0+b; i<i1-b; i++) {
        int idx = i+j*w;
        sx_raw += ic[idx]*mask[idx];
        ic[idx] -= ic2[idx];
        sx += ic[idx]*mask[idx];
        sx2 += ic[idx]*ic[idx]*mask[idx];
        num_pix += mask[idx];
      }
    }
  } else {
    for (int j = j0+b; j<j1-b; j++) {
      for (int i = i0+b; i<i1-b; i++) {
        int idx = i+j*w;
        sx += ic[idx]*mask[idx];
        sx2 += ic[idx]*ic[idx]*mask[idx];
        num_pix += mask[idx];
      }
    }
    sx_raw = sx;
  }

  double mean = sx/num_pix;
  double mean_raw = sx_raw/num_pix;

  double sigma = sqrt((sx2 - sx*sx/num_pix)/num_pix);
  fprintf(stdout,"sigma: %g mean: %g\n", sigma, mean);

  /*** fill output buffer if the variable is defined ***/
  if (output_buffer) {
    int pixel_offset = 0;
    if (high_pass_filter) pixel_offset = 50;

    if (filter_return_image) {
      for (int j = j0+1; j<j1-1; j++) {
        for (int i=i0+1; i<i1-1; i++) {
          output_buffer[i+j*w] = ic[i+j*w]+pixel_offset;
        }
      }
      for (int j=0; j<b; j++) {
        for (int i=i0; i<i1; i++) {
          output_buffer[i+(j+j0)*w] = output_buffer[i + (j1-j-1)*w] = mean+pixel_offset;
        }
      }
      for (int j=j0; j<j1; j++) {
        for (int i=0; i<b; i++) {
          output_buffer[i+i0+j*w] = output_buffer[i1-i-1 + j*w] = mean+pixel_offset;
        }
      }
    } else {
      for (int j=j0; j<j1; j++) {
        for (int i=i0; i<i1; i++) {
          int idx = i+j*w;
          output_buffer[idx] = input_buffer[idx];
        }
      }
    }
  }

  /*** Find the blobs ***/
  double ic0;
  int blob_count = 0;
  for (int j = j0+b; j<j1-b-1; j++) {
    for (int i=i0+b; i<i1-b-1; i++) {
      // if pixel exceeds threshold
      if (((double)ic[i+j*w] > mean + n_sigma*sigma) || ((double)ic[i+j*w] > 254) || ((double)ic[i+j*w]>7)) {
        ic0 = ic[i+j*w];
        // If pixel is a local maximum or saturated
        if (((ic0 >= ic[i-1 + (j-1)*w]) &&
             (ic0 >= ic[i   + (j-1)*w]) &&
             (ic0 >= ic[i+1 + (j-1)*w]) &&
             (ic0 >= ic[i-1 + (j  )*w]) &&
             (ic0 >  ic[i+1 + (j  )*w]) &&
             (ic0 >  ic[i-1 + (j+1)*w]) &&
             (ic0 >  ic[i   + (j+1)*w]) &&
             (ic0 >  ic[i+1 + (j+1)*w])) ||
            (ic0>254)){

          int unique = 1;

          // realloc array if necessary
          if (blob_count >= num_blobs_alloc) {
              num_blobs_alloc += 500;
              *starX = realloc(*starX, sizeof(double)*num_blobs_alloc);
              *starY = realloc(*starY, sizeof(double)*num_blobs_alloc);
              *starMag = realloc(*starMag, sizeof(double)*num_blobs_alloc);
              printf("REalloc'ed blobs to %d\n", num_blobs_alloc);
          }

          (*starX)[blob_count] = i;
          (*starY)[blob_count] = j;
          (*starMag)[blob_count] = 100*ic[i+j*CAMERA_WIDTH];

          // FIXME: not sure why this is necessary..
          if((*starMag)[blob_count] < 0){
            (*starMag)[blob_count] = UINT32_MAX;
          }

          // If we already found a blob within SPACING and this
          // one is bigger, replace it.
          int spacing = unique_star_spacing;
          if ((*starMag)[blob_count] > 25400) {
            spacing = spacing * 4;
          }
          for (int ib = 0; ib < blob_count; ib++) {
            if ((abs((*starX)[blob_count]-(*starX)[ib]) < spacing) &&
                (abs((*starY)[blob_count]-(*starY)[ib]) < spacing)) {
              unique = 0;
              // keep the brighter one
              if ((*starMag)[blob_count] > (*starMag)[ib]) {
                (*starX)[ib] = (*starX)[blob_count];
                (*starY)[ib] = (*starY)[blob_count];
                (*starMag)[ib] = (*starMag)[blob_count];
              }
            }
          }
          // if we didn't find a close one, it is unique.
          if (unique) {
            blob_count++;
          }
        }
      }
    }
  }

  //merge sort boi
  //part(*starMag,0,blob_count-1,*starX,*starY);

  return blob_count;

}


  void merge(double * A, int p, int q, int r, double * X, double * Y){
    int n1 = q-p+1, n2 = r-q;
    int lin = n1+1, rin = n2+1;
    int LM[lin],LX[lin],LY[lin],RM[rin],RX[rin],RY[rin],i,j,k;
    LM[n1]=(double)INT_MAX;
    LX[n1]=(double)INT_MAX;
    LY[n1]=(double)INT_MAX; 
    RM[n2]=(double)INT_MAX;
    RX[n2]=(double)INT_MAX;
    RY[n2]=(double)INT_MAX; 

    for(i;i<n1;i++){
      LM[i] = A[p+i];
      LX[i] = X[p+i];
      LY[i] = Y[p+i];
    }

    for(j;j<n2;j++){
      RM[j] = A[q+j+1];
      RX[j] = X[q+j+1];
      RY[j] = Y[q+j+1];
    }

    i=0;j=0;

    for(k=p;k<=r;k++){
      if(LM[i]>=RM[j]){
        A[k]=LM[i];
        X[k]=LX[i];
        Y[k]=LY[i];
        i++;
      }
      else{
        A[k]=RM[j];
        X[k]=RX[j];
        Y[k]=RY[j];
        j++;
      }
    }
  }

  void part(double * A, int p, int r, double * X, double * Y){
    if (p<r){
      int q = (p+r)/2;
      part(A,p,q,X,Y);
      part(A,q+1,r,X,Y);
      merge(A,p,q,r,X,Y);
    }
  }


int getImage(){    
    ImageFileParams.pwchFileName = L"/home/xscblast/Desktop/blastcam/Pictures/Image Files/roof1.png";

    ImageFileParams.pnImageID = NULL;

    ImageFileParams.ppcImageMem = NULL;

    ImageFileParams.nQuality = 80;

    ImageFileParams.nFileType = IS_IMG_BMP;
}

int bufferNumber = 0;
char* memory = NULL;
char* waitingMem = NULL;
unsigned long *picWidth;
unsigned long *picHeight;

int main(){

    FILE *roof15;
    roof15 = fopen("/home/xscblast/Desktop/blastcam/Pictures/Image Files/roof1.png", "r");  
     picWidth == 1574;
     picHeight == 1263;
  int gooo = 1;
  char * output_buffer = calloc(1, CAMERA_WIDTH * CAMERA_HEIGHT);
  double *starX = NULL, *starY = NULL, *starMag = NULL;
  double ra = -1, dec = -1, fr = -1, ps = -1;
    while (gooo) {
    
    readpng_init(roof15, picWidth, picHeight);

    // find the blobs in the image
    int blob_count = get_blobs(memory, CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CAMERA_WIDTH, CAMERA_HEIGHT, &starX, &starY, &starMag, output_buffer);
    memcpy(memory, output_buffer, CAMERA_WIDTH * CAMERA_HEIGHT);// this line makes it so kst shows the filtered image

    // solve astrometry
    printf("Trying to solve astrometry...\n");
    status = lost_in_space_astrometry(starX, starY, starMag, blob_count, &ra, &dec, &fr, &ps);

    if (status) {
      printf("Found astrometry solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! RA = %f, Dec = %f, FR = %f, pixel scale = %f\n", ra, dec, fr, ps);
      gooo = 0;
    } else {
      printf("Could not solve astrometry :(\n");
    }
    
    }
}