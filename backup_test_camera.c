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

void set_camera_params(unsigned int cameraHandle)
{
	int status;

	  /*int delay =*/ is_SetTriggerDelay(IS_GET_TRIGGER_DELAY, 0);
  //printf("trigger delay: %d\n", delay);

  status = is_SetHardwareGain(cameraHandle, 0, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  
  if(status != IS_SUCCESS){
    printf("setting gain failed\n");
  }
  double autoGain = 0;
  status = is_SetAutoParameter(cameraHandle, IS_SET_ENABLE_AUTO_GAIN, &autoGain, NULL);

  if(status != IS_SUCCESS){
    printf("disabling auto gain fails\n");
  }

  status = is_SetHardwareGamma(cameraHandle, IS_SET_HW_GAMMA_OFF);

  if(status != IS_SUCCESS){
    printf("disabling hardware gamma corection fails\n");
  }

  double autoShutter = 0;
  status = is_SetAutoParameter(cameraHandle, IS_SET_ENABLE_AUTO_SHUTTER, &autoShutter, NULL);

  if(status != IS_SUCCESS){
    printf("disabling auto shutter speed fails\n");
  }

  double autoFramerate = 0;
        status = is_SetAutoParameter(cameraHandle, IS_SET_ENABLE_AUTO_FRAMERATE, &autoFramerate, NULL);

        if(status != IS_SUCCESS){
                printf("disabling auto framerate fails\n");
        }

  status = is_SetGainBoost(cameraHandle, IS_SET_GAINBOOST_ON);

  if(status != IS_SUCCESS){
    printf("disabling gain boost fails\n");
  }

  int blackLevelMode = IS_AUTO_BLACKLEVEL_OFF;

  status = is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_GET_MODE, (void*) &blackLevelMode, sizeof(blackLevelMode));

  if(status != IS_SUCCESS){
    printf("getting auto black level mode fails, status %d\n", status);
  }
  //printf("mode is %d\n", blackLevelMode);
	
    status = is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_SET_MODE, (void*) &blackLevelMode, sizeof(blackLevelMode));

        if(status != IS_SUCCESS){
                printf("turning off auto black level mode fails, status %d\n", status);
        }


  blackLevelMode = 50;

        status = is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_SET_OFFSET, (void*) &blackLevelMode, sizeof(blackLevelMode));

        if(status != IS_SUCCESS){
                printf("setting black level to 0 fails, status %d\n", status);
        }

  status = is_Blacklevel(cameraHandle, IS_BLACKLEVEL_CMD_GET_OFFSET, (void*) &blackLevelMode, sizeof(blackLevelMode));

  if(status != IS_SUCCESS){
                printf("getting current black level fails, status %d\n", status);
        }else{
    printf("black level is %d\n", blackLevelMode);
  }
  blackLevelMode = 0;


  //this for some reason actually affects the time it takes to set aois only on the focal plane camera. Don't use if for the trigger timout. It is also not in miliseconds 
  //this firmware suuuuckssssssss
  status = is_SetTimeout(cameraHandle, IS_TRIGGER_TIMEOUT, 500);

  if(status != IS_SUCCESS){
    printf("setting trigger timeout fails with status %d\n", status);
  }
}

unsigned int cameraHandle;
int runCommand(const char* command, int file, char* returnStr);
int* lensNumber;
double focus;
int* fileArray;
int* currPosition;
static int maxAdapters;
char file;
const char* command;
char* returnVal;
int comStatus;
char* buffer;
int* focusMax;
int fileDescriptor;


int runCommand(const char* command, int file, char* returnStr){

    printf("running command %s\n", command);

    fd_set input, output;

    FD_ZERO(&output);

    FD_SET(file, &output);

    if(!FD_ISSET(file, &output)){
        printf("communication error in runcommnd\n");
        return -1;
    }
    tcflush(file, TCIOFLUSH);
    int status = write(file, command, strlen(command));
    printf("write return %d\n", status);
    if(status < 0){
        printf("write of command on file %d has failed, error %d\n", file, errno);
        return -1;
    }

    //TODO: do this properly with select and things
    usleep(1000000);

    printf("fd zero\n");
    FD_ZERO(&input);
    printf("fd set\n");
    FD_SET(file, &input);

    if(!FD_ISSET(file, &input)){
        printf("communication error in runCommand on read\n");
        return -1;
    }

    buffer = malloc(100);
    buffer[0] = '\0';
    printf("about to read\n");
    status = read(file, buffer, 99);
    printf("read returns %d\n", status);
    if(status <= 0){
        printf("read fails in runCommand, errno = %d\n", errno);
        return -1;
    }

    buffer[99] = '\0';
    buffer[status] = '\0';	

    printf("return buffer = %s\n", buffer);

    if(strstr(buffer, "ERR") != NULL){
        printf("read returned error %s\n", buffer);
        return -1;
    }else if(strstr(buffer, "OK") != NULL){
        printf("getting ok in runcommand, length %d, buffer contents:%send\n", status, buffer);
    }

    
    strcpy(returnStr, buffer);
    free(buffer);

    return 0;
}


int main ()
{

	//	init_astrometry();

	HIDS cameraHandle = 1;  // set by ueyesetid
	IS_POINT_2D locationRectangle;

	int status;	

	printf("LOADING CAMERA\n");

	SENSORINFO sensorInfo;
	
	if ((status = is_InitCamera(&cameraHandle, NULL)) == IS_SUCCESS) printf("Camera initialized\n");
	else 
	{
		printf("FAILED\n");
		exit(2);
	}


	if ((status = is_GetSensorInfo(cameraHandle, &sensorInfo)) == IS_SUCCESS) printf("Received camera sensor info\n");
	else 
	{
		printf("FAILED\n");
		exit(2);
	}

	set_camera_params(cameraHandle);

	if ((status = is_SetColorMode (cameraHandle, IS_CM_MONO8)) == IS_SUCCESS) printf("Set display mode\n");
	else
		{
		printf("FAILED\n");
		exit(2);
	}
 	
	char* memoryStartingPointer;
	int memoryId;
	int colourDepth = 8;
	if ((status = is_AllocImageMem(cameraHandle, sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, colourDepth, &memoryStartingPointer, &memoryId)) == IS_SUCCESS)
	{
		printf("Allocated camera memory\n");
	}
	else
	{
		printf("FAILED\n");
		exit(2);
	}

	if ((status = is_SetImageMem(cameraHandle, memoryStartingPointer, memoryId)) == IS_SUCCESS) printf("Set memory to active\n");
	else
	{
		printf("FAILED\n");
		exit(2);
	}

	void* activeMemoryLocation;
	if ((status = is_GetImageMem(cameraHandle, &activeMemoryLocation)) == IS_SUCCESS) printf("Got image memory\n");
	else
	{
		printf("FAILED %d\n",status);
		exit(2);
	}

  int pixelclock = 30;
	if ((status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock))) == IS_SUCCESS) printf("Pixel Clock set\n");
	else
	{
		printf("FAILED %d\n",status);
		exit(2);
	}

	double newFPS = 47;
	if ((status = is_SetFrameRate(cameraHandle, IS_GET_FRAMERATE, (void*)&newFPS)) == IS_SUCCESS) printf("Framerate set\n");
	else
	{
		printf("FAILED %d\n",status);
		exit(2);
	}

	if ((status = is_SetExternalTrigger (cameraHandle, IS_SET_TRIGGER_SOFTWARE)) == IS_SUCCESS) printf("trigger1\n");
		else
			{
			printf("FAILED\n");
			exit(2);
		}
	double exposure = 150; 
	//sets exposure time
	status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(double));
	if(status != IS_SUCCESS){
		printf("Set exposure time fails\n");
	}
	printf("Integration time is %lf\n", exposure);





	fileArray = malloc(maxAdapters);
	currPosition = malloc(maxAdapters);
	fileDescriptor = open("/dev/ttyS0", O_RDWR | O_NOCTTY /*| O_NDELAY*/);
	printf("opens file descriptor\n");

	if(fileDescriptor == -1){
			printf("error opening file descriptor for lens %s, error %d\n", "/dev/ttyS0", errno);
			return -1;
	}

	struct termios options;

	tcgetattr(fileDescriptor, &options);
	cfsetispeed(&options, B115200);               //input speed
	cfsetospeed(&options, B115200);               //output speed

	// standard setting for DSP 1750
	//these settings work so for the love of god never change them
	//unless you actually know what you are doing
							options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
							// disable IGNBRK for mismatched speed tests; otherwise receive break
							// as \000 chars

							options.c_iflag |= IGNBRK;         // ignore break signal
							options.c_lflag = 0;                // no signaling chars, no echo,
																																																																							// no canonical processing
							options.c_oflag = 0;                // no remapping, no delays
							options.c_cc[VMIN]  = 1;            // read doesn't block
							options.c_cc[VTIME] = 5;            // tenths of seconds for read timeout (integer)

							options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
							options.c_cflag &= ~CSTOPB;
							options.c_oflag &= ~OPOST;
							options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

							options.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
																																																																							// enable reading
							options.c_cflag &= ~(PARENB | PARODD);      // shut off parity
							options.c_cflag &= ~CSTOPB;
							options.c_cflag &= ~CRTSCTS;//turns off flow control maybe?
	options.c_iflag |= ICRNL;
	//sets a read timeout of 2 seconds so it doesn't block forever
	options.c_lflag &= ~ICANON;
	options.c_cc[VTIME] = 2;
	options.c_cc[VMIN] = 0;
	//options.c_iflag |= IGNCR;

	tcsetattr(fileDescriptor, TCSANOW, &options);     //apply changes
	printf("set file descriptor flags\n");

	//flush the buffer (in case of unclean shutdown)
	if (tcflush(fileDescriptor, TCIOFLUSH) < 0) {
			printf("buffer fails to flush in add lens\n");
	}

	returnVal = malloc(100);

	comStatus = runCommand("mn0\r", fileDescriptor, returnVal);
	if(comStatus < 0){
			printf("move to zero fails in focuser\n");
			return -1;
	}

	comStatus = runCommand("sf0\r", fileDescriptor, returnVal);
	if(comStatus == -1){
			printf("setting focal zero fails\n");
	}

	comStatus = runCommand("mf0\r", fileDescriptor, returnVal);
	if(comStatus == -1){
			printf("move to infinity fails in focuser\n");
	}else{
			printf("parsing output %s\n", returnVal);
			strtok(returnVal, "E");
			char* temp = strtok(NULL, ",");
			int range = abs(atoi(temp));
	}

	free(returnVal);







	int bufferNumber = 0;
	char* memory = NULL;
	char* waitingMem = NULL;

	IMAGE_FILE_PARAMS ImageFileParams;

	ImageFileParams.pwchFileName = L"save1.bmp";

	ImageFileParams.pnImageID = NULL;

	ImageFileParams.ppcImageMem = NULL;

	ImageFileParams.nQuality = 80;

	ImageFileParams.nFileType = IS_IMG_BMP;


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
		


	is_ExitCamera(cameraHandle);
	printf("CLOSED CAMERA\n");


	//close_astrometry();
	
	return 1;
}