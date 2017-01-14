/*
   emotemonitor - Open Source emote uart monotor program for *nix
   Copyright (C) 2016 Ling Wang <lngsa.wang@gmail.com>

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
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include "serial_posix.h"
#include "serial.h"

#define PROGRAM_NAME "EMOTEMONITOR"

bool data_flag = false;
char buf[256];

FILE* fpin;
FILE* fpout;

/*
   static void Serial_ReceiveByte()
   {
   char c;
   if(SerialGetChar(&c)== 1)
   {
   fwrite(&c, 1, sizeof(char), fpout);
   printf("%c", c);
   }
   }
 */

void read_serial_data()
{
	int bytes;
	bytes = SerialRead(buf, 256);
	if (bytes > 0){
		//fwrite(buf, bytes, sizeof(char), stdout);
   		fwrite(buf, bytes, sizeof(char), fpout);
	}else{
		printf("[%s]%s:read serial data error occured, code = %d\n", PROGRAM_NAME, __func__, bytes);
		exit(-1);
	}
}


/* uart interrpt function*/
void signal_handler_IO (int status)
{
	//Serial_ReceiveByte();
	data_flag = true;
}

int main(int argc, char *argv[])
{
	char* device;
	char* outfile;
	struct timeval start, stop;
	int runtime = 0;
	int elapsetime = 0;

	/*Parameters*/    
	if( argc < 4){
		printf("[%s]FORMAT: emotemonitor runtime  outfile serialport. \n", PROGRAM_NAME);
		exit(-1);
	}else{
		runtime = atoi(argv[1]);
		outfile = argv[2];
		device = argv[3];
	}

	/*Print version information*/
	printf("[%s]emote monitor version 2.0.0, COPYRIGHT (C) DNC 2016.\n", PROGRAM_NAME);

	/*Open input file*/

	/*Open output file*/
	fpout = fopen(outfile, "wr");
	if(fpout == NULL)
	{
		fclose(fpout);
		printf("[%s]ERROR:fail to open log file %s.\n", PROGRAM_NAME, outfile);
		exit(-1);
	}

	/*Open serial port*/
	if(SerialInt(device) < 0)
	{
		printf("[%s]ERROR:fail to open serial port %s.\n", PROGRAM_NAME, device);
		exit(-1);
	}


	gettimeofday(&start, NULL);
	while (elapsetime < runtime)
	{
		if (data_flag == true){
			data_flag = false;
			read_serial_data();
		}
		gettimeofday(&stop, NULL);
		elapsetime = (stop.tv_sec - start.tv_sec)/60;
	}


	/*close file and serial device*/
	fclose(fpout);
	SerialClose();
	fpout = NULL;

	printf("[%s]INFO: monitor has finished.\n", PROGRAM_NAME);

	return 0;
}



