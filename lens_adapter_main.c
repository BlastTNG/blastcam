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
#include "runCommand.c"

int init_lensAdapter(){

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

    
}