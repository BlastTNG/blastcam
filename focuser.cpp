//focuser.cpp
//virtual class needs a destructor somehow

#include "focuser.h"

Focuser::~Focuser(){

}

//this finds the optimal focus from this list of blobs
double Focuser::findBestFocus(std::vector<std::vector<blob> > blobList, int blobsPerImage, int cameraNum){

        int maxLuminosity = 0;
        double bestFocus = getMinFocus(cameraNum);
        int currLuminosity;
        int blobsUsed;

        for(int i = 1; i < (int)blobList.size() - 1; i++){
                currLuminosity = 0;
                blobsUsed = 0;
                int currNumUsedBlobs = blobsPerImage;
                if((int)blobList[i].size() < blobsPerImage){
                        currNumUsedBlobs = blobList[i].size();
                }
                printf("at this point i = %d, num used is %d, size = %d, list[i].size = %d\n", i, currNumUsedBlobs, (int) blobList.size(), (int) blobList[i].size());
		for(int j = 0; j< currNumUsedBlobs; j++){
                        if(blobContains(blobList[i][j], blobList[i-1]) && blobContains(blobList[i][j], blobList[i+1])){
	                        currLuminosity += blobList[i][j].totalIntensity;
				printf("pos = %d, blob=%d, lum = %d\n", i, j, currLuminosity);
                                blobsUsed ++;
                        }else{
                                printf("discarding blob at %lf, %lf intensity %d\n", blobList[i][j].centroid.x, blobList[i][j].centroid.y, blobList[i][j].totalIntensity);

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

bool Focuser::blobContains(blob searchBlob, std::vector<blob> &list){
        for(int i = 0; i<(int)list.size(); i++){

                if(abs(searchBlob.centroid.x - list[i].centroid.x) < 20 && abs(searchBlob.centroid.y - list[i].centroid.y) < 20){
                        return true;
                }

        }
        return false;
}

