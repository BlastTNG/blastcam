#include <stdio.h>

void printArr(unsigned char * arr) {
    int loop;
    for (loop = 0; loop < 500; loop++) {
        printf("%d\n", arr[loop]);
    }
}

int getBMPData(char * filename) {
    // arrays to hold image data
    unsigned char image1[499446];
    
    // read the bitmap image files in binary mode
    FILE *bmpFile1 = fopen(filename, "rb");
    
    // check if the files are open
    if (!bmpFile1) {
        perror("Failed to open file.\n");
    }
    
    // load image data into arrays
    fread(image1, 1, 499446, bmpFile1);
    
    // close files
    fclose(bmpFile1);
    
    printArr(image1);
    
    return 0;
}

int main(void) {
    getBMPData("/home/xscblast/Desktop/blastcam/BMPs/saved_image_2019-07-01-23-34-22.bmp");   
}