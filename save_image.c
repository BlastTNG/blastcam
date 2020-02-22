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

int bufferNumber = 0;
char* memory = NULL;
char* waitingMem = NULL;

IMAGE_FILE_PARAMS ImageFileParams;

int saveImage(){

    ImageFileParams.pwchFileName = L"save1.bmp";

    ImageFileParams.pnImageID = NULL;

    ImageFileParams.ppcImageMem = NULL;

    ImageFileParams.nQuality = 80;

    ImageFileParams.nFileType = IS_IMG_BMP;
}