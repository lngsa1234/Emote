/*
   emoteflash - Open Source ST STM32 flash program for *nix
   Copyright (C) 2010 Ling Wang <lngsa.wang@gmail.com>

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>

#include "ymodem.h"
#include "serial_posix.h"
#include "serial.h"
#include "debug.h"


#define BUFSIZE 256

#define PROGRAM_NAME "SERIAL_DIAGNOSIS"

extern serial_t *h;

int main(int argc, char *argv[])
{
	char *device;
	int mode = 1;

	/* command line arguments */    
	if (argc < 3){
		printf("Please input image file name and serial device name: \n");
		printf("FORMAT: emoteflash deviceName mode\n");
		exit(-1);
	}else{
		device = argv[1];
		mode = atoi(argv[2]);
	}

	/* print version informationi */
	printf("Serial Diagnosis 1.0.0, COPYRIGHT (C) Ling Wang 2017.\n");

        /* mode 1, select; mode 2, read repeatlyi */
	if( mode == 1){
		int ret;
		int fd;
		fd_set fdset;
		struct timeval timeout;
		char buf[BUFSIZE];
		int bytes;
		int cnt = 0;

		speed_t	port_baud = B115200;   /* 115200 */
		tcflag_t port_bits = CS8;      /* 8 data bits */
		tcflag_t port_parity = 0;      /* no parity */
		tcflag_t port_stop = 0;        /* 1 stop bit */
		
		h = serial_open(device);
		if(h == NULL){
			printf("[%s]ERROR: failed open serial port %s.\n", PROGRAM_NAME, device);
			return -1;
		}

		if ( serial_setup(h,port_baud,port_bits,port_parity,port_stop)!= 0 ){
			printf("[%s]ERROR: failed init serial port %s.\n", PROGRAM_NAME, device);
			return -1;
		}

		fd = h->fd;
		printf("[%s]INFO: serial fd %d.\n", PROGRAM_NAME, fd);
	
		FD_ZERO(&fdset);
		FD_SET(fd, &fdset);
		
		while (cnt++ < 1000000)
		{
			timeout.tv_sec = 10;
			timeout.tv_usec = 0;	
			ret = select(fd+1, &fdset, NULL, NULL, &timeout);
			printf("[%s]INFO: time stamp: %d s.\n", PROGRAM_NAME,  (int)timeout.tv_sec);
			if( ret > 0 ){
				bytes = SerialRead(buf, BUFSIZE-1);
				if(bytes > 0){
					buf[bytes] = '\0';
					printf("%s", buf);
				}else{
					printf("[%s]WARNING: serial read unnormal situation.\n", PROGRAM_NAME);
				}
			}else if (ret == 0){
				printf("[%s]INFO: timeout, %s.\n", PROGRAM_NAME, strerror(errno));
			}else if (ret == -1){
				printf("[%s]ERROR: serial select.\n", PROGRAM_NAME);
				SerialClose();
				exit(-1);
			}
		}
		printf("[%s]program is exiting.\n", PROGRAM_NAME);
		SerialClose();
	}
	else if(mode == 2){
	
		char buf[BUFSIZE];
		int bytes;
		int cnt = 0;

		/*Init serial port*/
		if (SerialInt(device)<0)
		{
			printf("[%s]ERROR: serial port %s init failed.\n", device, PROGRAM_NAME);
			exit(-1);
		}

		while ( cnt++ < 1000000 ){
			bytes = SerialRead(buf, BUFSIZE-1);
			printf("[%s]INFO:read bytes: %d.\n",PROGRAM_NAME,  bytes);
			if( bytes > 0 ){
				for (int i=0; i< bytes; i++){
					printf("%c", buf[i]);
				}
			}else if (bytes == 0){
				printf("[%s]INFO: timeout, no data.\n", PROGRAM_NAME);
			} else if (bytes < 0){
				printf("[%s]ERROR: serial read.\n", PROGRAM_NAME);
				exit(-1);
			}
		}
	}
	return 0;
}



