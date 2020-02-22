#include "qdbmp.c"
#include <stdio.h>


//in main
//Defines some variables
int main()
{
    BMP* bmp;
    UINT width, height;
    UINT x, y;
    UCHAR color;

    //Read the image file
    bmp = BMP_ReadFile("/home/xscblast/Desktop/blastcam/Pictures/BMP Files/roof15-1(1).bmp");
    BMP_CHECK_ERROR(stderr, -1);

    //Get image's dimensions
    width = BMP_GetWidth(bmp);
    height = BMP_GetHeight(bmp);

    //Iterate through all pixels
    for (x = 0; x<width; ++x){
        for(y = 0; y<height; ++y){
            //Get color values
            BMP_GetPixelIndex(bmp, x, y, &color);
            //Invert color values
            BMP_SetPixelIndex(bmp, x, y, 255-color);
        }
    }

    //Saves new file
    BMP_WriteFile(bmp, "/home/xscblast/Desktop/blastcam/Pictures/BMP Files/roof15-1.bmp");
    BMP_Free(bmp);
}