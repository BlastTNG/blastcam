#pragma once
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
#include "vector.c"

double getMinFocus(int);
int cameraNum;
int blobsPerImage;

struct coords{//the coordinates of the object
    double x;
    double y;
};

struct icoords{ //the coordinates of the object
    int x;
    int y;
};

struct blob{//a structure representing a blob
    struct coords centroid;
    struct icoords icentroid;
    int intensity;
    int totalIntensity;
};

struct location{//a struct that holds ra, dec and field rotation
    double ra;
    double dec;
    double theta;
    double sigma;
    struct coords maxStar;
};

struct blob searchBlob;

bool blobContains(){
    for(int i = 0; i<(int)list.capacity(); i++){

            if(abs(searchBlob.centroid.x - list[i].centroid.x) < 20 && abs(searchBlob.centroid.y - list[i].centroid.y) < 20){
                    return true;
            }

    }
    return false;
}

//this finds the optimal focus from this list of blobs
double findBestFocus(){

    int maxLuminosity = 0;
    double bestFocus = getMinFocus(cameraNum);
    int currLuminosity;
    int blobsUsed;
    vector blobList;
    vector_init(&blobList);
    vector_add(&blobList, searchBlob.intensity);
    vector_add(&blobList, searchBlob.totalIntensity);

    for(int i = 1; i < &blobList.capacity - 1; i++){
            currLuminosity = 0;
            blobsUsed = 0;
            int currNumUsedBlobs = blobsPerImage;
            if(vector_get(&blobList,i) < blobsPerImage){
                    currNumUsedBlobs = vector_get(&blobList,i);
            }
            printf("at this point i = %d, num used is %d, size = %d, list[i].size = %d\n", i, currNumUsedBlobs, (int) &blobList.capacity, (int) &blobList.capacity);
    for(int j = 0; j< currNumUsedBlobs; j++){
                    if(blobContains(vector_get(&blobList,i),vector_get(&blobList,j), vector_get(&blobList,i-1)) && blobContains(vector_get(&blobList,i),vector_get(&blobList,j), vector_get(&blobList,i+1))){
                        currLuminosity += &blobList[5];
            printf("pos = %d, blob=%d, lum = %d\n", i, j, currLuminosity);
                            blobsUsed ++;
                    }else{
                            printf("discarding blob at %lf, %lf intensity %d\n", &blobList[i][j].centroid.x, &blobList[i][j].centroid.y, &blobList[i][j].totalIntensity);

                    }
                    if(blobsUsed > blobsPerImage - 2){
                            break;
                    }
            }
            if(blobsUsed < blobsPerImage - 2){
                    printf("using fewer that optimal number of blobs in findbestfocus\n");
            }
            printf("used %d blobs\n", blobsUsed);


            if(currLuminosity > maxLuminosity){
                    maxLuminosity = currLuminosity;
                    bestFocus =getMinFocus(cameraNum) + i*getFocusStep(cameraNum);
                    printf("current best focus found to be %lf, with lum %d\n", bestFocus, currLuminosity);
            }
    }

    return bestFocus;
}


//A file that holds some struct definitions

