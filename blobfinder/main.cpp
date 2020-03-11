#include <iostream>
#include <stdio.h>
#include <QImage>
#include <QDebug>
#include <QTime>
#include <QStringList>
#include <math.h>
#include "../bitsc/camera_defs.h"
#include "blobfinder.h"

#include <tiffio.h>

#define IMAGEDIR "../bitsc/imagedump/june23/"
#define IMAGEDIR2 "../bitsc/imagedump/march2016/"
#define IMAGEDIR3 "/home/cbn/programs/bitmcc/imagedump/"

int main(int argc, char *argv[])
{
  QTime T;
  QStringList images[3];
  int i_image = 0;
  int i_cam = 2;
  int camera;
  int dT1=0, dT2=0, dT3=0;
  int dT1b=0, dT2b=0, dT3b=0;

  if (argc>=3) {
    i_image = atoi(argv[2]);
    i_cam = atoi(argv[1]);
  }

  if (i_cam == 1) {
    camera = ROLL_CAMERA;
  } else if (i_cam == 2) {
#ifndef IMAGES16BIT
    fprintf(stderr, "must be compiled with IMAGES16BIT defined for science camera.  Exiting.\n");
    exit(0);
#endif
    camera = SCIENCE_CAMERA;
  } else if (i_cam == 0) {
    camera = BORESIGHT_CAMERA;
  } else {
    camera = ROLL_CAMERA;
  }

#ifdef IMAGES16BIT
  if (i_cam !=2) {
    fprintf(stderr, "must be compiled without IMAGES16BIT defined for roll/bore camera.  Exiting.\n");
    exit(0);
  }
#endif
  images[0]
      << "subframes/borepic_2_470_470.bmp"
      << IMAGEDIR2 "mar262016-10.bmp"
      << IMAGEDIR2 "mar262016-1.bmp"
      << IMAGEDIR2 "mar262016-2.bmp"
      << IMAGEDIR2 "mar262016-3.bmp"
      << IMAGEDIR2 "mar262016-4.bmp"
      << IMAGEDIR2 "mar262016-5.bmp"
      << IMAGEDIR2 "mar262016-6.bmp"
      << IMAGEDIR2 "mar262016-7.bmp"
      << IMAGEDIR2 "mar262016-8.bmp"
      << IMAGEDIR2 "mar262016-9.bmp"
      ;

  images[1]
      << IMAGEDIR "rollImage1435113501.bmp" << IMAGEDIR "rollImage1435115715.bmp" << IMAGEDIR "rollImage1435116699.bmp"
      << IMAGEDIR "rollImage1435116758.bmp" << IMAGEDIR "rollImage1435116788.bmp" << IMAGEDIR "rollImage1435116814.bmp"
      << IMAGEDIR "rollImage1435117017.bmp" << IMAGEDIR "rollImage1435117747.bmp" << IMAGEDIR "rollImage1435123087.bmp"
      << IMAGEDIR "rollImage1435124301.bmp" << IMAGEDIR "rollImage1435890518.bmp" << IMAGEDIR "rollImage1435890625.bmp"
      << IMAGEDIR "rollImage1435890648.bmp" << IMAGEDIR "rollImage1435890727.bmp" << IMAGEDIR "rollImage1435890881.bmp"
      << IMAGEDIR "rollImage1435891255.bmp" << IMAGEDIR "rollImage1435892838.bmp" << IMAGEDIR "rollImage1435892872.bmp"
      << IMAGEDIR "rollImage1435893783.bmp" << IMAGEDIR "rollImage1435893811.bmp" << IMAGEDIR "rollImage1435893837.bmp"
      << IMAGEDIR "rollImage1435893873.bmp" << IMAGEDIR "rollImage1435897004.bmp";

  images[2]
      << IMAGEDIR3 "scicam_16bit.raw"
         ;

  QString image = images[i_cam][i_image];
  fprintf(stderr, "%s\n", image.toLatin1().data());

  imageType *ib;
  imageType *ib2;
  imageType *ob;
  int w;
  int h;

#ifdef IMAGES16BIT

  w = SCIENCE_CAMERA_X_PIXELS;
  h = SCIENCE_CAMERA_Y_PIXELS;
  int fd;

  ib = (imageType *)malloc(h*w*sizeof(imageType));

  fd = open(image.toLatin1().data(), O_RDONLY);
  read(fd, ib, w*h*sizeof(imageType));
  close(fd);

  QImage I(w,h,QImage::Format_ARGB32);

  printf("created image w: %d h: %d\n", I.width(), I.height());

#else
  /***************************************/
  /* read in the image as 8 bit grayscale */

  QImage I(image);
  T.start();


  w = I.size().width();
  h = I.size().height();

  ib = (imageType *)malloc(h*w*sizeof(imageType));

  for (int j=0; j<h; j++) {
    for (int i=0; i<w; i++) {
      ib[i+j*w] = I.pixelIndex(i,j);
    }
  }
#endif

  ob = (imageType *)malloc(h*w*sizeof(imageType));
  ib2 = (imageType *)malloc(h*w*sizeof(imageType));
  for (int j=0; j<h; j++) {
    for (int i=0; i<w; i++) {
      ib2[i+j*w] = ib[i+j*w];
    }
  }

  /***************************************/

  // Add a test gaussian to the image
#if 1
  double s = 2.0;
  double a = 200.0;
  double c = 29.6; // center

  for (int j=0; j<2*c; j++){
    for (int i=0; i<2*c; i++){
      double x = i-c;
      double y = j-c;
      double r2 = x*x + y*y;
      ib[i+j*w]+= a*exp(-(r2/2.0/(s*s)));
      ib2[i+j*w]+= a*exp(-(r2/2.0/(s*s)));
    }
  }
#endif



  for (int i=0; i<w; i++) {
    for (int j=0; j<h; j++) {
#ifdef IMAGES16BIT
      int g = ib[i+j*w]/100;
      I.setPixel(i,h-j-1, qRgb(g,g,g));
#else
      I.setPixel(i,h-j-1, ib[i+j*w]);
#endif

    }
  }
  I.save("./raw.png", "png");

  BlobFinder blobfinder(camera);
  std::vector<blob> blobs;

  dT1 = T.elapsed();
  blobfinder.getBlobs(ib,0,0,0,0,blobs, ib);
  dT1b = T.elapsed();

  blobfinder.writeData(blobs, "blobs", 0, false);

  for (int i=0; i<w; i++) {
    for (int j=0; j<h; j++) {
#ifdef IMAGES16BIT
      int g = ib[i+j*w]/100;
      I.setPixel(i,h-j-1, qRgb(g,g,g));
#else
      I.setPixel(i,h-j-1, ib[i+j*w]);
#endif
    }
  }
  I.save("./filtered.png", "png");

  if (blobs.size()>0) {
    // now try subframes

    printf("Blob 1a: %g %g\n", blobs[0].centroid.x,blobs[0].centroid.y);

    dT2 = T.elapsed();
    //blobfinder.getBlobs(ib2,blobs[0].centroid.x+5,blobs[0].centroid.y+5,50,blobs, ob);
    blobfinder.getBlobs(ib2, 30, 25 ,60,50,blobs, ob);
    dT2b = T.elapsed();

    if (blobs.size()>0) {
      printf("Blob 1b: %g %g\n", blobs[0].centroid.x,blobs[0].centroid.y);
    }
    for (int i=0; i<w; i++) {
      for (int j=0; j<h; j++) {
        I.setPixel(i,h-j-1, ob[i+j*w]);
      }
    }
    I.save("./subframe.png", "png");
  }

  // now try lost in space.  Re-read the image.
  QImage I2(image);

  /*
  for (int j=0; j<h; j++) {
    for (int i=0; i<w; i++) {
      ib[i+j*w] = I.pixelIndex(i,j);
      //ib[i+j*w] = I2.pixelIndex(i,h-j-1);
    }
  }
  */

  dT3 = T.elapsed();
  location psol; //= blobfinder.lostInSpace(30,ib, guess);
  dT3b = T.elapsed();

  //printf("ra: %g dec: %g theta: %g\n", psol.ra, psol.dec, psol.sigma);

  free(ib);
  free(ob);
  printf("full frame: %dms  subframe: %dms  LIS: %dms\n", dT1b-dT1, dT2b-dT2, dT3b-dT3);
}

