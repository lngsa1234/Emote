/*
   emotemonitor -- monitor serial port
   Copyright (C) DNC 2016
	
   Author: Ling Wang <lngsa.wang@gmail.com>

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
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>

#include "serial_posix.h"


#define BUFSIZE 256

#define PROGRAM_NAME "EMOTEMONITOR"


int main(int argc, char *argv[])
{
	char *device;
	char *outfile;
	int minutes;
	FILE *fp;
	serial_t *h;
	int ret;
	int fd;
	fd_set fdset;
	char buf[BUFSIZE];
	int bytes;
	struct timeval select_timeout;
	struct timeval start, current, stop;

	/* command line arguments */    
	if (argc < 4){
		printf("FORMAT: emotemonitor time outfile deviceName.\n");
		exit(-1);
	}else{
		minutes = atoi(argv[1]);
		outfile = argv[2];
		device = argv[3];
	}

	/* print version informationi */
	printf("Emote monitor 3.0.0, COPYRIGHT (C) DNC 2016.\n");

	/* open or creat output file */
	if((fp = fopen(outfile, "w+")) == NULL){
		fprintf(stderr, "[%s]ERROR: failed open output file %s.\n", PROGRAM_NAME, outfile);
		return(-1);
	}

	/* open and configure serial port */

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
        /* dump serial port configuration*/
	printf("[%s]INFO: serial configure info:\n", PROGRAM_NAME);
	serial_configinfo(h);
       
	/* setup select mode reading */
	fd = h->fd;
	FD_ZERO(&fdset);
	FD_SET(fd, &fdset);

       /* monitor serial until timeout*/
	printf("[%s]INFO: start monitoring serial port %s.\n", PROGRAM_NAME, device);
	
	gettimeofday(&start, NULL);
	stop.tv_sec = start.tv_sec + minutes*60;
	
	gettimeofday(&current, NULL);
	while (current.tv_sec < stop.tv_sec)
	{
	//	printf("[%s]DEBUG: monitor time left: %d s.\n", PROGRAM_NAME,  (int)(stop.tv_sec - current.tv_sec));
		select_timeout.tv_sec = 10;
		select_timeout.tv_usec = 0;	
		ret = select(fd+1, &fdset, NULL, NULL, &select_timeout);
	//	printf("[%s]DEBUG: select time stamp: %d s.\n", PROGRAM_NAME,  (int)select_timeout.tv_sec);
		if( ret > 0 ){
			bytes = serial_read(h, buf, BUFSIZE-1);
			if(bytes > 0){
				buf[bytes] = '\0';
	//			printf("%s", buf);
	               		fprintf(fp, "%s", buf);
			}else{
				printf("[%s]WARNING: serial read unnormal situation, other process may be reading the same serial.\n", PROGRAM_NAME);
			}
		}else if (ret == 0){
			printf("[%s]INFO: timeout, %s.\n", PROGRAM_NAME, strerror(errno));
		}else if (ret == -1){
			printf("[%s]ERROR: serial select.\n", PROGRAM_NAME);
			serial_close(h);
			return(-1);
		}
		gettimeofday(&current, NULL);
	}
	
	/* close outfile and serial port */
	printf("[%s]INFO: finished monotoring serial port %s.\n", PROGRAM_NAME, device);
	serial_close(h);
	fclose(fp);
	
	return 0;
}



