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
#include "lens_adapter_main.c"
#include "camera.c"

HIDS cameraHandle = 1;  // set by ueyesetid
IS_POINT_2D locationRectangle;

int load_camera(){
    
    int status;	
    printf("LOADING CAMERA\n");

	SENSORINFO sensorInfo;
	
	if ((status = is_InitCamera(&cameraHandle, NULL)) != IS_SUCCESS){ 
		printf("Camera failed to initialize\n");
		exit(2);
	}


	if ((status = is_GetSensorInfo(cameraHandle, &sensorInfo)) != IS_SUCCESS) {
		printf("Failed to receive camera sensor info\n");
		exit(2);
	}

	set_camera_params(cameraHandle);

	if ((status = is_SetColorMode (cameraHandle, IS_CM_MONO8)) != IS_SUCCESS) {
		printf("Seting display mode failed\n");
		exit(2);
	}
 	
	char* memoryStartingPointer;
	int memoryId;
	int colourDepth = 8;
	if ((status = is_AllocImageMem(cameraHandle, sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, 
	colourDepth, &memoryStartingPointer, &memoryId)) != IS_SUCCESS)
	{
		printf("Failed to allocate camera memory\n");
		exit(2);
	}
	
	if ((status = is_SetImageMem(cameraHandle, memoryStartingPointer, memoryId)) != IS_SUCCESS) {
		printf("Failed to set memory to active\n");
		exit(2);
	}

	void* activeMemoryLocation;
	if ((status = is_GetImageMem(cameraHandle, &activeMemoryLocation)) == IS_SUCCESS) 
	printf("Got image memory\n");
	else
	{
		printf("FAILED %d\n",status);
		exit(2);
	}

  int pixelclock = 30;
	if ((status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock)))
	 != IS_SUCCESS) {
		printf("Pixel Clock failed to set\n");
		exit(2);
	}

	double newFPS = 47;
	if ((status = is_SetFrameRate(cameraHandle, IS_GET_FRAMERATE, (void*)&newFPS)) != IS_SUCCESS) {
		printf("Framerate failed to set\n");
		exit(2);
	}

	if ((status = is_SetExternalTrigger (cameraHandle, IS_SET_TRIGGER_SOFTWARE)) != IS_SUCCESS) {
		printf("trigger set failed\n");
		exit(2);	
	}
}