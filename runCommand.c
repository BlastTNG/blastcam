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