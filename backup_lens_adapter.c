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
int status;
int comStatus;
char* buffer;

int main()
{
    fileArray = malloc(maxAdapters);
    currPosition = malloc(maxAdapters);
    int fileDescriptor = open("/dev/ttyS0", O_RDWR | O_NOCTTY /*| O_NDELAY*/);
    printf("opens file descriptor\n");

    if(fileDescriptor == 0){
        printf("error opening file descriptor for lens %s, error %d\n", "/dev/ttyS0", errno);
        return 0;
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

    int* focusMax;
    returnVal = malloc(100);
    returnVal[0] = '\0';
    printf("about to read\n");
    status = read(file, returnVal, 99);
    printf("read returns %d\n", status);
    if(status <= 0){
    	printf("read fails in runCommand, errno = %d\n", errno);
    	return -1;
    }
    // char file;
    // const char* command;

    int comStatus = write(file, command, strlen(command));
    comStatus = runCommand("mz\r", fileDescriptor, returnVal);
    if(comStatus == -1){
        printf("move to zero fails in focuser\n");
    }

    comStatus = runCommand("sf0\r", fileDescriptor, returnVal);
    if(comStatus == -1){
        printf("setting focal zero fails\n");
    }

    comStatus = runCommand("mi\r", fileDescriptor, returnVal);
    if(comStatus == -1){
        printf("move to infinity fails in focuser\n");
    }else{
        //printf("parsing output %s\n", returnVal);
        strtok(returnVal, "E");
        char* temp = strtok(NULL, ",");
        int range = abs(atoi(temp));
        focusMax[cameraHandle] = range;
        currPosition[cameraHandle] = 0;
    }

    int position = 0;
    char* cmdStr = malloc(100);
    char* returnVal = malloc(100);
    int status;


    sprintf(cmdStr, "fa%d\r", (int)focus /*- focusMax[lensNumber]*/);

	status = runCommand(cmdStr, fileArray[*lensNumber], returnVal);
	if(status == -1){
		printf("running fa fails\n");
	}else{
		strtok(returnVal, "E");
               	char* temp = strtok(NULL, ",");
      		position = atoi(temp);
		printf("moved %d to %d\n", position - currPosition[*lensNumber], position);
		currPosition[*lensNumber] = position;
		temp = strtok(NULL, "\0");
		//int flag = atoi(temp);//should be 1 if we hit a stop
		//TODO: maybe something about this
	}

    status = write(file, command, strlen(command));
	printf("write return %d\n", status);
	if(status < 0){
		printf("write of command on file %d has failed, error %d\n", file, errno);
		return -1;
	}

    char* buffer = malloc(100);
	buffer[0] = '\0';
	printf("about to read\n");
	status = read(file, buffer, 99);
	printf("read returns %d\n", status);
	if(status <= 0){
		printf("read fails in runCommand, errno = %d\n", errno);
		return -1;
	}





	//printf("lens moved to %d, was told to move to %d\n", currPosition[*lensNumber], focus);

	free(returnVal);
	free(cmdStr);
	return position;

    int currentStatus = system("ma\r");
    if(currentStatus == 0){
    	printf("move to zero fails in focuser\n");
    }


    currentStatus = system("sf0\r");
    if(currentStatus == 0){
    	printf("setting focal zero fails\n");
    }

    currentStatus = system("mi\r");
    if(currentStatus == 0){
    	printf("move to infinity fails in focuser\n");
    }else{
    	printf("parsing output %s\n", returnVal);
    	strtok(returnVal, "E");
    	char* temp = strtok(NULL, ",");
    	int range = abs(atoi(temp));
    	focusMax[cameraHandle] = range;
    	currPosition[cameraHandle] = 0;
    }

    free(returnVal);

    return cameraHandle;
}

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