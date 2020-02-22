#include "/home/xscblast/Desktop/blastcam/blobfinder/blobfinder.h"

// Blobs closer than this are merged.
#define SPACING 15

// High Pass filter Radius
#define F_R 10

// number of stages for the centroid finding boxcar filter.
#define STAGES 3

#include <algorithm>
#include <vector>
#include <cmath>

#define BLOB_MASK_MARGIN 3

static bool blobCompareFunction(blob blob1, blob blob2){
  //return (blob1.totalIntensity > blob2.totalIntensity);
  return (blob1.intensity > blob2.intensity);
}

BlobFinder::BlobFinder(int camera_in)
{
  //FILE *fp;
  camera = camera_in;

  use_old_blobfinder = false;
  
  switch (camera) {
  case ROLL_CAMERA:
    w = ROLL_CAMERA_X_PIXELS;
    h = ROLL_CAMERA_Y_PIXELS;
    //fp = fopen("/data/etc/maskRoll", "r");
    strncpy(camera_name, "roll", 7);
    border = 10;
    high_pass_filter = true;
    dynamic_hot_pixels_full_frame = false;
    dynamic_hot_pixels_sub_frame = true;
    n_sigma = 6.0;
    break;
  case FOCAL_PLANE_CAMERA:
    w = FPSC_CAMERA_X_PIXELS;
    h = FPSC_CAMERA_Y_PIXELS;
    //fp = fopen("/data/etc/maskRoll", "r");
    strncpy(camera_name, "focl", 7);
    border = 2  ;
    high_pass_filter = false;
    dynamic_hot_pixels_full_frame = false;
    dynamic_hot_pixels_sub_frame = false;
    n_sigma = 6.0;
    break;
  case SCIENCE_CAMERA:
    w = SCIENCE_CAMERA_X_PIXELS;
    h = SCIENCE_CAMERA_Y_PIXELS;
    //fp = fopen("/data/etc/maskRoll", "r");
    strncpy(camera_name, "scicam", 7);
    border = 10;
    high_pass_filter = false;
    dynamic_hot_pixels_full_frame = true;
    dynamic_hot_pixels_sub_frame = true;
    n_sigma = 6.0;
    use_old_blobfinder = true;
    break;
  case BORESIGHT_CAMERA:
  default:
    w = BORESIGHT_CAMERA_X_PIXELS;
    h = BORESIGHT_CAMERA_Y_PIXELS;
    //fp = fopen("/data/etc/maskBore", "r");
    strncpy(camera_name, "bore", 7);
    border = 1;
    high_pass_filter = true;
    dynamic_hot_pixels_full_frame = false;
    dynamic_hot_pixels_sub_frame = true;
    n_sigma = 5.0;
    break;
  }

  r_centroid = 5; // ~not used
  r_smooth = 1; // image high pass filter radius
  filter_return_image = true;
  spike_limit = 0.35;
  blobsToWrite = 10;
  use_hpList = false;
	blob_mask_margin = BLOB_MASK_MARGIN;

  nc = (imageType *)malloc(w * h * sizeof(imageType));
  ibc1 = (imageSumType *)malloc(w * h * sizeof(imageSumType));
  ic = (double *)malloc(w * h * sizeof(double));
  ic2 = (double *)malloc(w * h * sizeof(double));
  mask = (unsigned char *)malloc(w * h * sizeof(unsigned char));

  loadHPList();

  astrometry = new Astrometry(camera_in);
}

BlobFinder::~BlobFinder()
{
  free(mask);
  free(ic2);
  free(ic);
  free(ibc1);
  free(nc);
}

void BlobFinder::loadHPList() {
  FILE *fp;

  if (camera == ROLL_CAMERA) {
    fp = fopen(ROLL_PIXEL_FILE, "r");
  } else if (camera == BORESIGHT_CAMERA) {
    fp = fopen(BORE_PIXEL_FILE, "r");
  } else if (camera == FOCAL_PLANE_CAMERA) {
    fp = fopen(FOCL_PIXEL_FILE, "r");
  } else {
    fp = NULL;
  }

  hpList.clear();

  if (fp == NULL) {
    return;
  }

  int i,j;
  char inbuf[1024];
  struct icoords coord;
  while (fgets(inbuf, 1024, fp)) {
    sscanf(inbuf, "%d %d", &i, &j);
    coord.x = i;
    coord.y = j;
    hpList.push_back(coord);
  }
  fclose(fp);
  fprintf(stderr, "hplist size: %u\n", (unsigned)hpList.size());

}

void BlobFinder::writeData(std::vector<blob> blobs, const char *filename, unsigned int n_blobs, bool remove_margin)
{
  FILE *fp;

  fp = fopen(filename, "w");
  if (fp) {

    if (n_blobs == 0) {
      n_blobs = blobs.size();
    } else if (blobs.size() < n_blobs) {
      n_blobs = blobs.size();
    }

    int margin = border;
    if (!remove_margin) margin = 0;

    for (unsigned int i=0; i<n_blobs; i++) {
      fprintf(fp, "%f %f %d %d\n", blobs[i].centroid.x-margin, blobs[i].centroid.y-margin, blobs[i].intensity, blobs[i].totalIntensity);
    }
    fclose(fp);
  }
  //n blobs 0 - write them all
  // remove_margin: subtract margin
}

location BlobFinder::lostInSpace(int timeout, imageType *image, location guess)
{
  location answer;

  std::vector<blob> blobList;
  getBlobs(image, 0, 0, 0, 0,blobList, image);


  int writeBlobs = blobsToWrite;
  printf("got %d blobs, using %d blobs in lost in space\n", (int) blobList.size(), writeBlobs);
  if((unsigned int) blobsToWrite > blobList.size()){
    writeBlobs = blobList.size();
  }
  if(blobList.size() == 0){
    printf("no blobs found in astrometry::lostInSpace\n");	
    answer.sigma = -1000;
    answer.ra = -1000;
    answer.dec = -1000;
    return answer;
  }
  blobList.resize(writeBlobs);
/*
  writeData(blobList, "blobList.txt", writeBlobs, true);
  system("rm bloblist.fits");
  system("/usr/local/astrometry/bin/text2fits.py -H \"x y\" -f ff blobList.txt bloblist.fits");
  system("rm blobSolution.txt");

  char* cmdBuffer = new char[500];
  sprintf(cmdBuffer, "sudo /usr/local/astrometry/bin/solve-field ./bloblist.fits -l %d --width %d --height %d --scale-units arcsecperpix --scale-low %lf --scale-high %lf --overwrite --no-plots --crpix-center >> blobSolution.txt", timeout, w, h, pixelScale, pixelScale + 1);

  printf("about to astrometry with cmd %s\n", cmdBuffer);
  system(cmdBuffer);

  delete[] cmdBuffer;

  FILE* file = fopen("blobSolution.txt", "r");
  int timeCount = 0;
  while(file == NULL){
    sleep(1);
    timeCount ++;
    printf("waiting for file time = %d\n", timeCount);
    file = fopen("blobSolution.txt", "r");
    if(timeCount > timeout){
      printf("blobsolutions file not opened properly in lostIn space\n");
      answer.sigma = -1;
      return answer;
    }
  }

  //extract output

  double ra, dec, sigma, scale = 0;
  ra = -1;
  char* buffer = new char[500];

  while(fgets(buffer, 500, file) != NULL){
    if(strstr(buffer, "RA,Dec = (") != NULL){
      sscanf(buffer, "  RA,Dec = (%lf,%lf), pixel scale %lf arcsec/pix.\n", &ra, &dec, &scale);
      //printf("found the field center angle line, ra = %lf, dec = %lf\n", ra, dec);
    }else if(strstr(buffer, "Field rotation angle:") != NULL){
      sscanf(buffer, "Field rotation angle: up is %lf degrees E of N\n", &sigma);
      //printf("at the rotation angle line, rotation = %lf\n", sigma);
    }
  }

  fclose(file);

  file = fopen("bloblist.wcs", "r");
  if(file == NULL){
    printf("no wcs file found\n");
    ra = -1;
    sigma = -1;
    dec = -1;
  }else{

    char* doubleBuf = new char[100];
    while(fgets(buffer, 81, file) != NULL){
      if(strstr(buffer, "CRVAL1") != NULL){
        sscanf(buffer, "CRVAL1  = %s / RA  of reference point", doubleBuf);
        ra = atof(doubleBuf);
        //printf("ra read as %.11lf, line %s\n", ra, buffer);
      }else if(strstr(buffer, "CRVAL2") != NULL){
        sscanf(buffer, "CRVAL2  = %s / DEC of reference point", doubleBuf);
        dec = atof(doubleBuf);
        //printf("dec read as %.11lf, line %s\n", dec, buffer);
        break;
      }
    }
    delete[] doubleBuf;
    fclose(file);
  }
  delete[] buffer;

  answer.ra = ra;
  answer.dec = dec;
  answer.sigma = sigma;
*/

  answer = astrometry->lostInSpace(timeout, blobList, border, guess);


  blob maxBlob = blobList[0];
  answer.maxStar.x = maxBlob.centroid.x;
  answer.maxStar.y = maxBlob.centroid.y;

  if(answer.ra == -1000){
    printf("no solution found in lost in space\n");
  }

  return answer;
}

void BlobFinder::boxcarFilterImage(imageType *ib, int i0, int j0, int i1, int j1, int r_f, double *filtered_image) {

  //printf("in boxcar filter, w = %d, h = %d, r_f = %d\n", w, h, r_f);
  int b = r_f;
  imageSumType isx;
  unsigned int s, n;
  double ds, dn;
  double last_ds=0;

  for (int j = j0; j<j1; j++) {
    n = 0;
    isx = 0;
    for (int i = i0; i<i0+2*r_f+1; i++) {
      n += mask[i+j*w];
      isx += ib[i+j*w]*mask[i+j*w];
    }
    int idx = w*j+i0+r_f;
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
        int idx = i + (j+jp)*w;
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

      filtered_image[i+j*w] = ds;
    }
  }
}

void BlobFinder::getBlobs(imageType *input_buffer, int x_center, int y_center, int x_size, int y_size, std::vector<blob> &blobs, imageType* output_buffer)
{
  // identify full frame by setting x_size to 0.
  // if x_size = 0, all other size/position parameters
  // are ignored and a full frame is assumed.
  bool subframe = (x_size!=0);

	subframe = subframe && ((x_size != w) || (y_size != h));

  // we use half-width internally, but the api gives us
  // full width.
  x_size = x_size/2;
  y_size = y_size/2;

  blobs.clear();
  if (subframe) {          
    if ((x_size<0) ||
        (x_center < x_size) ||
        (x_center > w-x_size-1) ||
        (y_center < y_size) ||
        (y_center > h-y_size-1)) {
      printf("** invalid returning **, w= %d, h = %d, x_size=%d, x_center=%d, y_size=%d, y_center=%d\n", w, h, x_size, x_center, y_size, y_center);
      return;
    }
  }

  int j0, j1, i0, i1;
  int b;

  if (!subframe) {
    j0 = i0 = 0;
    j1 = h;
    i1 = w;
    b = border;
  } else {
    i0 = 0;
    i1 = 2*x_size;
    j0 = 0;
    j1 = 2*y_size;
    b = 1;
  }

  makeMask(input_buffer, i0, j0, i1, j1, x_center-x_size, y_center-y_size, subframe);

  double sx=0, sx2=0;
  double sx_raw = 0; // sum of non-highpass filtered field
  int num_pix=0;

  // lowpass filter the image - reduce noise.
  boxcarFilterImage(input_buffer, i0, j0, i1, j1, r_smooth, ic);
	/*
	for (int j=j0;j<j1;j++)
	{
		for (int i=i0;i<i1;i++)
		{
			ic[i+j*w] = input_buffer[i+j*w];
		}
	}
	*/

  if (high_pass_filter && !subframe) { // only high-pass filter full frames
    //fprintf (stderr, "hp filter active\n");
    b += F_R;
    boxcarFilterImage(input_buffer, i0, j0, i1, j1, F_R, ic2);

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
  //fprintf(stdout,"sigma: %g mean: %g\n", sigma, mean);

  /*** fill output buffer if the variable is defined ***/
  if (output_buffer) {
    int pixel_offset = 0;
    if (high_pass_filter) pixel_offset = 50;
    if (subframe) pixel_offset = 0;

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
  blobs.clear();
  blob B;
  double ic0;
	double sig_limit = mean+n_sigma*sigma;
  // find ~unique blobs.

  b += r_centroid;
  for (int j = j0+b; j<j1-b-1; j++) {
    for (int i=i0+b; i<i1-b-1; i++) {
      // if pixel exceeds threshold
      ic0 = ic[i+j*w];
      if ((ic0 > sig_limit) || (ic0 > 254)) {
        // If pixel is a local maximum or saturated	
				int cent = i-1+(j-1)*w;

        if (((ic0 >= ic[cent   ]) && // top left
             (ic0 >= ic[cent+=1]) && // top mid
             (ic0 >= ic[cent+=1]) && // top right
             (ic0 >= ic[cent+=w]) && // mid right
             (ic0 >  ic[cent-=2]) && // mid left
             (ic0 >  ic[cent+=w]) && // bottom left
             (ic0 >  ic[cent+=1]) && // bottom mid
             (ic0 >  ic[cent+=1])) || // bottom right
            (ic0>254)){

          int nb = blobs.size();
          bool unique = true;
          B.icentroid.x = i;
          B.icentroid.y = j;
          B.intensity = B.totalIntensity = 100*ic0;
          if(B.intensity < 0){
            B.intensity = B.totalIntensity = INT_MAX;
          }


          // If we already found a blob within SPACING and this
          // one is bigger, replace it.
          int spacing = SPACING;
          if (B.intensity>25400) {
            spacing = spacing * 4;
          }
          for (int ib = 0; ib < nb; ib++) {
            if ((abs(B.icentroid.x-blobs[ib].icentroid.x) < spacing) &&
                (abs(B.icentroid.y-blobs[ib].icentroid.y) < spacing)) {
              unique = false;
              // keep the brighter one
              if (B.intensity > blobs[ib].intensity) {
                blobs[ib] = B;
              }
            }
          }
          // if we didn't find a close one, it is unique.
          if (unique) {
            blobs.push_back(B);
          }
        }
      }
    }
  }

	// subpixel correction based on gradient intercept method
	// uses a 5x5 image kernel around each potential blob
	// effectively fits a line to the gradient of the image
  int nb = blobs.size();
	double S0 = 0, T0 = 0, F = 0;
	double X4 = 0, X2 = 0, X2Y = 0;
	int dx, dy;
	double pxb, px0;
	double sN = 0;

	double thresh = 0;
	double nxd = 0;

	for (int i=i0+b;i<i1-b-1;i++)
	{
		pxb = ic[i+(j0+b)*w];
		if (pxb <= sig_limit) // not a peak
		{
			thresh +=	pxb;
			nxd++;
		}	
	}
	thresh /= nxd;

  for (int ib = 0; ib < nb; ib++) {
		S0 = 0;
		T0 = 0;
		F = 0;
		X4 = 0;
		X2 = 0;
		X2Y = 0;
		sN = 0;

		px0 = input_buffer[blobs[ib].icentroid.y*w+blobs[ib].icentroid.x]-thresh;

		//printf("mask margin is %d\n",blob_mask_margin);

		for (int j=-blob_mask_margin;j<=blob_mask_margin;j++)
		{
			dy = blobs[ib].icentroid.y+j;
			if ((dy >= j0) && (dy < j1))
			{
				for (int i=-blob_mask_margin;i<=blob_mask_margin;i++)
				{
					dx = blobs[ib].icentroid.x+i;
					if ((dx >= i0) && (dx < i1))
					{
						// ** use original buffer for subpixel correction, which includes a mean filter **
						pxb = input_buffer[dy*w+dx];
						if (pxb > thresh)
						{
							pxb -= thresh;

							if (pxb <= px0)
							{
								double y = log(pxb);
								double x2 = i*i+j*j;
								X2 += x2;
								X4 += x2*x2;
								X2Y += x2*y;
								sN++;
							}
	
							if ((abs(i) <= blob_mask_margin) && (abs(j) <= blob_mask_margin))
							{
								S0 += i*pxb;
								T0 += j*pxb;
								F += pxb;
							}
							
						}
					}
				}
			}
		}

		// correct blob with scale factor correction
		double stddev = 0.0, scalef = 1.0;
		double den = log(px0)*X2-X2Y;
		if (den > 0) 
		{
			stddev = sqrt(0.5*X4/den);
			if (stddev > 2.0) // get extra precision for larger blobs
			{
				stddev = (stddev <= 5.0) ? ceil(stddev): 5.0;
				for (int j=-stddev; j<=stddev; j++)
				{
					if (j == -blob_mask_margin) j = blob_mask_margin+1;

					dy = blobs[ib].icentroid.y+j;
					if ((dy >= j0) && (dy < j1))
      		{
						for (int i=-stddev; i<=stddev; i++)
						{
							if (i == -blob_mask_margin) i = blob_mask_margin+1;

							dx = blobs[ib].icentroid.x+i;
							if ((dx >= i0) && (dx < i1))
							{
								pxb = input_buffer[dy*w+dx];
  		          if (pxb > thresh)
      		      {
          		    pxb -= thresh;

              		if (pxb <= px0)
              		{ 
                		double y = log(pxb);
                		double x2 = i*i+j*j;
                		X2 += x2;
                		X4 += x2*x2;
                		X2Y += x2*y;
                		sN++;
              		}
            		}
							}
							//printf("%d %d\n",i,j);
						}
					}
				}
				den = log(px0)*X2-X2Y;
				if (den > 0) stddev = sqrt(0.5*X4/den);
			}
			

			scalef = 0.5009142914*exp(0.773468273*stddev);
		}

/*
		// experimental section
		int loc;
		double leftx = 0;
		double rightx = 0;
		double pxl;
		double pxr;
		double afs = 0.75; 
		double xcent = 0.0;
		double num = 0;

		double SX4 = 0.0;
		double SX3 = 0.0;
		double SX2 = 0.0;
		double SX = 0.0;
		double S = 0.0;
		double SX2Y = 0.0;
		double SXY = 0.0;
		double SY = 0.0;

		for (int j=-2;j<=2;j++)
		{
			loc = (blobs[ib].icentroid.y+j)*w+blobs[ib].icentroid.x;
			px0 = input_buffer[loc]-thresh;

			if (px0 > 0)
			{

		  SX4 = 0.0;
			SX3 = 0.0;
			SX2 = 0.0;
			SX = 0.0;
			S = 1.0;
			SX2Y = 0.0;
			SXY = 0.0;
			SY = log(px0);
			leftx = 0;
			rightx = 0;
			pxl = 0;
			pxr = 0;	
			for (int i=1;i<10;i++)
			{
				double pxd = input_buffer[loc+i]-thresh;

				double X1 = double(i);
				double X2 = X1*X1;
				double Y = log(pxd);			

				if (pxd > 0)
				{
					SX4 += X2*X2;
					SX3 += X1*X2;
					SX2 += X2;
					SX += X1;
					S += 1.0;

					SX2Y += X2*Y;
					SXY += X1*Y;
					SY += Y;
				}
				if ((pxd < afs*px0) && (pxd > 0))
				{
					rightx = X1;
					pxr = Y;
					break;
				}
			
			}
			for (int i=1;i<10;i++)
			{
				double pxd = input_buffer[loc-i]-thresh;

				double X1 = -double(i);
				double X2 = X1*X1;
				double Y = log(pxd);			
	
				if (pxd > 0)
				{
					SX4 += X2*X2;
					SX3 += X1*X2;
					SX2 += X2;
					SX += X1;
					S += 1.0;

					SX2Y += X2*Y;
					SXY += X1*Y;
					SY += Y;
				}

				if ((pxd < afs*px0) && (pxd > 0))
				{
					leftx = X1;
					pxl = Y;
					break;
				}
			
			}
		

			px0 = log(px0);
			if ((leftx != 0) && (rightx != 0))
			{
				xcent += -0.5*input_buffer[loc]*(SX4*(S*SXY-SX*SY)-SX3*(S*SX2Y-SY*SX2)+SX2*(SX2Y*SX-SX2*SXY))/(SX2Y*(S*SX2-SX*SX)-SXY*(S*SX3-SX2*SX)+SY*(SX3*SX-SX2*SX2));
				num += input_buffer[loc];
			}
			}
		}
		xcent /= double(num);
		xcent += blobs[ib].icentroid.x;
*/

		//printf("%f %f %f\n",stddev,log(px0),sN);
		//printf("DEBUG: %f %f %f\n\n", sN, stddev, scalef);
		F = (F == 0) ? 1 : F;

    double X = blobs[ib].icentroid.x + S0/F*scalef;
    double Y = blobs[ib].icentroid.y + T0/F*scalef;
    
    if ((X >= i0) && (X <= i1) && (Y >= j0) && (Y <= j1)) {
      blobs[ib].centroid.x = X + double(x_center-x_size); //blobs[ib].icentroid.x + S0/F*scalef + double(x_center-x_size);
      blobs[ib].centroid.y = Y + double(y_center-y_size); //blobs[ib].icentroid.y + T0/F*scalef + double(y_center-y_size);
    }  else {
      //printf("*** bad blob coordinate! X: %g Y: %g I: %d S0: %g  T0: %g F: %g scalef:%g X4: %g den: %g***\n", X, Y, blobs[ib].intensity, S0, T0, F, scalef, X4, den);
      blobs[ib].centroid.x = blobs[ib].icentroid.x + double(x_center-x_size);
      blobs[ib].centroid.y = blobs[ib].icentroid.y + double(y_center-y_size);
      //blobs[ib].intensity = 0;
    }
		//printf("%f = %f %f %f %f %f\n",blobs[ib].centroid.x,xcent,leftx,pxl,rightx,pxr);

  }

  std::sort(blobs.begin(), blobs.end(), blobCompareFunction);

  if (blobs.size()==0) { // no blobs: lets see if it was saturated
    if (mean_raw>250.0) {
      B.icentroid.x = (i0+i1)/2;
      B.icentroid.y = (j0+j1)/2;
      B.intensity = B.totalIntensity = 100.0*mean_raw;
      blobs.push_back(B);
    }
  }
  //if (subframe && blobs.size()) printf(" **** found blob in subframe ***\n");
  /*
  for(int i = 0; i< blobs.size(); i++){
    printf("found a blob at %lf, %lf, intensity: %d total intensity: %d\n", blobs[i].centroid.x, blobs[i].centroid.y, blobs[i].intensity, blobs[i].totalIntensity);
  }
  */
}

void BlobFinder::getBlobsOld(imageType *input_buffer, int x_center, int y_center, int x_size, int y_size, std::vector<blob> &blobs, imageType* output_buffer)
{
  // identify full frame by setting x_size to 0.
  // if x_size = 0, all other size/position parameters
  // are ignored and a full frame is assumed.
  bool subframe = (x_size!=0);

	subframe = subframe && ((x_size != w) || (y_size != h));

  // we use half-width internally, but the api gives us
  // full width.
  x_size = x_size/2;
  y_size = y_size/2;

  blobs.clear();
  if (subframe) {          
    if ((x_size<0) ||
        (x_center < x_size) ||
        (x_center > w-x_size-1) ||
        (y_center < y_size) ||
        (y_center > h-y_size-1)) {
      printf("** invalid returning **, w= %d, h = %d, x_size=%d, x_center=%d, y_size=%d, y_center=%d\n", w, h, x_size, x_center, y_size, y_center);
      return;
    }
  }

  int j0, j1, i0, i1;
  int b;

  if (!subframe) {
    j0 = i0 = 0;
    j1 = h;
    i1 = w;
    b = border;
  } else {
    i0 = 0;
    i1 = 2*x_size;
    j0 = 0;
    j1 = 2*y_size;
    b = 1;
  }

  makeMask(input_buffer, i0, j0, i1, j1, x_center-x_size, y_center-y_size, subframe);

  double sx=0, sx2=0;
  double sx_raw = 0; // sum of non-highpass filtered field
  int num_pix=0;

  // lowpass filter the image - reduce noise.
  boxcarFilterImage(input_buffer, i0, j0, i1, j1, r_smooth, ic);

  if (high_pass_filter && !subframe) { // only high-pass filter full frames
    //fprintf (stderr, "hp filter active\n");
    b += F_R;
    boxcarFilterImage(input_buffer, i0, j0, i1, j1, F_R, ic2);

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
  //fprintf(stdout,"sigma: %g mean: %g\n", sigma, mean);

  /*** fill output buffer if the variable is defined ***/
  if (output_buffer) {
    int pixel_offset = 0;
    if (high_pass_filter) pixel_offset = 50;
    if (subframe) pixel_offset = 0;

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
  blobs.clear();
  blob B;
  double ic0;
  // find ~unique blobs.
  b += r_centroid;
  for (int j = j0+b; j<j1-b-1; j++) {
    for (int i=i0+b; i<i1-b-1; i++) {
      // if pixel exceeds threshold
      if (((double)ic[i+j*w] > mean + n_sigma*sigma) || ((double)ic[i+j*w] > 254)) {
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

          int nb = blobs.size();
          bool unique = true;
          B.icentroid.x = i;
          B.icentroid.y = j;
          B.intensity = B.totalIntensity = 100*ic[i+j*w];
          if(B.intensity < 0){
            B.intensity = B.totalIntensity = INT_MAX;
          }


          // If we already found a blob within SPACING and this
          // one is bigger, replace it.
          int spacing = SPACING;
          if (B.intensity>25400) {
            spacing = spacing * 4;
          }
          for (int ib = 0; ib < nb; ib++) {
            if ((abs(B.icentroid.x-blobs[ib].icentroid.x) < spacing) &&
                (abs(B.icentroid.y-blobs[ib].icentroid.y) < spacing)) {
              unique = false;
              // keep the brighter one
              if (B.intensity > blobs[ib].intensity) {
                blobs[ib] = B;
              }
            }
          }
          // if we didn't find a close one, it is unique.
          if (unique) {
            blobs.push_back(B);
          }
        }
      }
    }
  }

  /*** find centroids: ***
   * re-map a sub-image around the blob
   * onto a 'res' times
   * higher resolution map 1D map, then low-pass
   * filter it, and then find the peak.
   * Do this first for X, then for Y.
   * Clearly, this has a precision of
   * pixelScale/res. */
  int nb = blobs.size();
  int res = 10; // subpixels per pixel
  int filt = 2; // filter half-width in pixels
  int w_sp = res*(2*filt*(STAGES-1)+1); // size of the array to filter
  double sp[w_sp];
  for (int ib = 0; ib < nb; ib++) {
    // find X centroid
    // collapse into 1D X array
    int i_min = blobs[ib].icentroid.x - filt*(STAGES-1);
    int j_min = blobs[ib].icentroid.y - filt*(STAGES-1);

    for (int i = 0; i <= 2*filt*(STAGES-1); i++) {
      int i_p = i+i_min;
      double s = 0.0;
      for (int j = 0; j<= 2*filt; j++) {
        int j_p = j+j_min;
        s += ic[i_p + j_p*w];
      }
      for (int k=0; k<res; k++) {
        sp[i*res + k] = s;
      }
    }
    int max = 0;
    int i_xmax = Box1DCenter(sp, w_sp, filt, res, max);

    // find Y centroid
    // collapse into 1D Y array
    for (int j = 0; j <= 2*filt*(STAGES-1); j++) {
      int j_p = j+j_min;
      double s = 0.0;
      for (int i = 0; i<= 2*filt; i++) {
        int i_p = i+i_min;
        s += ic[i_p + j_p*w];
      }
      for (int k=0; k<res; k++) {
        sp[j*res + k] = s;
      }
    }

    int i_ymax = Box1DCenter(sp, w_sp, filt, res, max);
    blobs[ib].totalIntensity = max;

    blobs[ib].centroid.x = blobs[ib].icentroid.x + double(i_xmax - w_sp/2)/double(res)+0.5+double(x_center-x_size);
    if (subframe) {
      blobs[ib].centroid.y = blobs[ib].icentroid.y + double(i_ymax - w_sp/2)/double(res)+0.5+double(y_center-y_size);
    } else {
      blobs[ib].centroid.y = blobs[ib].icentroid.y + double(i_ymax - w_sp/2)/double(res)+0.5+double(y_center-y_size);
    }
  }

  std::sort(blobs.begin(), blobs.end(), blobCompareFunction);

  if (blobs.size()==0) { // no blobs: lets see if it was saturated
    if (mean_raw>250.0) {
      B.icentroid.x = (i0+i1)/2;
      B.icentroid.y = (j0+j1)/2;
      B.intensity = B.totalIntensity = 100.0*mean_raw;
      blobs.push_back(B);
    }
  }
  //if (subframe && blobs.size()) printf(" **** found blob in subframe ***\n");
  /*
  for(int i = 0; i< blobs.size(); i++){
    printf("found a blob at %lf, %lf, intensity: %d total intensity: %d\n", blobs[i].centroid.x, blobs[i].centroid.y, blobs[i].intensity, blobs[i].totalIntensity);
  }
  */
}


void BlobFinder::makeMask(imageType *ib, int i0, int j0, int i1, int j1, int x0, int y0, bool subframe)
{
  int i, j;
  int p0, p1, p2, p3, p4;
  int a, b;
  int cutoff = spike_limit*100.0;

  for (i=i0; i<i1; i++) {
    mask[i+ w*j0] = mask[i + (j1-1)*w] = 0;
  }
  for (j=j0; j<j1; j++) {
    mask[i0+j*w] = mask[i1-1 + j*w] = 0;
  }

  i0++;
  j0++;
  i1--;
  j1--;

  bool dynamic_hot_pixels;
  if (subframe) dynamic_hot_pixels = dynamic_hot_pixels_sub_frame;
  else dynamic_hot_pixels = dynamic_hot_pixels_full_frame;
  
  if (dynamic_hot_pixels) {
    int nhp = 0;
    for (j = j0; j<j1; j++) {
      for (i=i0; i<i1; i++) {
        p0 = 100*ib[i+j*w]/cutoff;
        p1 = ib[i-1+(j)*w];
        p2 = ib[i+1+(j)*w];
        p3 = ib[i+(j+1)*w];
        p4 = ib[i+(j-1)*w];
        a = p1+p2+p3+p4+4;
        p1 = ib[i-1+(j-1)*w];
        p2 = ib[i+1+(j+1)*w];
        p3 = ib[i-1+(j+1)*w];
        p4 = ib[i+1+(j-1)*w];
        b = p1+p2+p3+p4+4;
        mask[i+j*w] = ((p0 < a) && (p0 < b));
        if (p0>a || p0 > b) nhp++;
      }
    }
    //printf("Dynamic hot pixels (%d)\n", nhp);
  } else {
    for (j = j0; j<j1; j++) {
      for (i=i0; i<i1; i++) {
        mask[i+j*w] = 1;
      }
    }
  }

  if (use_hpList) {
    int ix, iy;
    int n = hpList.size();
    if (subframe) {
      for (i=0; i<n; i++) {
        ix = hpList[i].x - x0;
        iy = hpList[i].y-y0;
        if ((ix>=0) && (iy >=0) && (ix < i1) && (ix < j1)) {
          mask[ix + w * (iy)] = 0;
        }
      }
    } else {
      for (i=0; i<n; i++) {
        mask[hpList[i].x - x0 + w * (hpList[i].y-y0)] = 0;
      }
    }
  }
}

#ifndef NO_PARAMHANDLER

int BlobFinder::setParams(int mode, ParamHandler *params)
{

  n_sigma = params->getParam(camera_name, "set_****_blob_param", "BLOB_****_SIGMA");
  spike_limit = params->getParam(camera_name, "set_****_blob_param", "BLOB_****_SPIKE");
  r_centroid = params->getParam(camera_name, "set_****_blob_param", "BLOB_****_SIZE");
  if(params->getParam(camera_name, "set_****_blob_param", "BLOB_****_HPF") == 0){
    high_pass_filter = false;
  }else{
    high_pass_filter = true;
  }
  
  int filter_level = params->getParam(camera_name, "set_****_blob_param", "BLOB_****_FILTER");
  dynamic_hot_pixels_sub_frame = ((filter_level & 0x01)!=0);
  dynamic_hot_pixels_full_frame = ((filter_level & 0x02)!=0);
  use_hpList = ((filter_level & 0x04)!=0);
  
  border = params->getParam(camera_name, "set_****_blob_param", "BLOB_****_MARGIN");

  switch(mode){

  case PARAM_MODE_SANDBOX:
    filter_return_image = true;
    if(params->getParam(camera_name, "sandbox_****_image_capture", "SNBX_****_FILTER") == 0){
      filter_return_image = false;
    }
    bin_factor = params->getParam(camera_name, "sandbox_****_image_capture", "SNBX_****_BINNING");
    break;

  case PARAM_MODE_FAST:
    filter_return_image = true;
    if(params->getParam(camera_name, "fast_****_image_capture", "FAST_****_FILTER") == 0){
      filter_return_image = false;
    }

    bin_factor = params->getParam(camera_name, "fast_****_image_capture", "FAST_****_BINNING");
    border = 1;

    break;

  case PARAM_MODE_LOST:
    blobsToWrite = params->getParam(camera_name, "****_lost_in_space", "LOST_****_BLOBS");
    bin_factor = params->getParam(camera_name, "****_lost_in_space", "LOST_****_BINNING");
    filter_return_image = false;
    break;

  case PARAM_MODE_FOCUS:
    bin_factor = params->getParam(camera_name, "****_sc_focus", "FOCUS_****_BINNING");
    filter_return_image = false;
    break;

  default:
    printf("invalid mode selected in locator:setParams\n");
    return -1;

  }
  return 0;
}

#endif

void BlobFinder::setNumBlobs(int numBlobs){
	blobsToWrite = numBlobs;
}

int BlobFinder::setCParams(double n_sig, double sl, int r_cent, int hpf, int filterType, int filterReturn, int edge){

	n_sigma = n_sig;
	spike_limit = sl;
	r_centroid = r_cent;
	high_pass_filter = hpf;
	filter_return_image = filterReturn;
	border = edge;

  dynamic_hot_pixels_sub_frame = ((filterType & 0x01)!=0);
  dynamic_hot_pixels_full_frame = ((filterType & 0x02)!=0);
  use_hpList = ((filterType & 0x04)!=0);

	return 0;
}

int BlobFinder::Box1DCenter(double *x, int nx, int filt, int res, int &max_out) {
  double sp[STAGES][nx];

  for (int k=0; k<nx; k++) {
    sp[0][k] = x[k];
  }

  for (int stage = 1; stage < STAGES; stage++) {
    double s = 0;
    for (int k = 0; k < filt*res; k++) {
      s += sp[stage-1][k];
      sp[stage][k] = sp[stage-1][k];
    }
    int i_sp0 = filt*res/2;
    sp[stage][i_sp0] = s;
    for (int k = i_sp0+1; k < nx - i_sp0; k++) {
      s = s + sp[stage-1][k + i_sp0] - sp[stage-1][k - i_sp0 - 1];
      sp[stage][k] = s;
    }
  }
  int max = sp[STAGES-1][0];
  int i_xmax = 0;
  for (int k = 0; k<nx; k++) {
    if (sp[STAGES-1][k]>max) {
      max = sp[STAGES-1][k];
      i_xmax = k;
    }
  }
  max_out = max;
  return i_xmax;
}


extern "C" void* make_blobfinder(int camera){
	
	return (void*)new BlobFinder(SCIENCE_CAMERA);
}

extern "C" void blobfinder_getBlobs(void* fakeClass, imageType* picture, int x_center, int y_center, int x_size, int y_size, blob* blobs, int numBlobs, imageType* outBuffer){
	BlobFinder* bf = static_cast<BlobFinder*>(fakeClass);
	std::vector<blob> blobVec;
  if (bf->use_old_blobfinder) { // should be only set for SCIENCE_CAMERA
    bf->getBlobsOld(picture, x_center, y_center, x_size, y_size, blobVec, outBuffer);
  } else {
    bf->getBlobs(picture, x_center, y_center, x_size, y_size, blobVec, outBuffer);
  }
	
	for(int i = blobVec.size(); i<numBlobs; i++){
		blob fake;
		fake.intensity = 0;
		fake.totalIntensity = 0;
		blobVec.push_back(fake);
	}

	//convert from vector to blob*
	for(int i = 0; i<numBlobs; i++){
		memcpy((void*)&blobs[i],  (void*)&blobVec[i], sizeof(blob));
		//printf("setting blob %d intensity to %d, centroid %lf, %lf\n", i, blobs[i].intensity, blobs[i].centroid.x, blobs[i].centroid.y);
	}

}

extern "C" struct location blobfinder_lostInSpace(void* fakeClass, int numBlobs, imageType* image, location guess){
	BlobFinder* bf = static_cast<BlobFinder*>(fakeClass);
	bf->setNumBlobs(numBlobs);
	return bf->lostInSpace(100, image, guess);

}

extern "C" void blobfinder_setParams(void* fakeClass, double n_sig, double sl, int r_cent, int hpf, int filterType, int filterReturn, int border){

	BlobFinder* bf = static_cast<BlobFinder*>(fakeClass);
	bf->setCParams(n_sig, sl, r_cent, hpf, filterType, filterReturn, border);
}
