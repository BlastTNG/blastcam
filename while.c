/*#include <stdio.h>
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

#include "load_camera.h"
#include "save_image.h"

int status;

int whileLoop(){
    while (1) {
    if ((status = is_FreezeVideo(cameraHandle, IS_WAIT)) == IS_SUCCESS) printf("video\n");
    else
        {
        printf("FAILED\n");
        exit(2);
    }
    status = is_GetActSeqBuf(cameraHandle, &bufferNumber, &waitingMem, &memory);

    if ((status = is_ImageFile (cameraHandle, IS_IMAGE_FILE_CMD_SAVE, (void*)&ImageFileParams, sizeof(ImageFileParams))) == IS_SUCCESS) printf("SAVE\n");
    else
        {
            char* lastErrorString;
            int lastError = 0;

            is_GetError(cameraHandle, &lastError, &lastErrorString);
            printf("FAILED %s\n", lastErrorString);
            exit(2);
        }

	}
}*/