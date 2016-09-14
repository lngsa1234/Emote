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
#include <unistd.h>

#include "serial_posix.h"
#include "serial.h"



FILE* fpin;
FILE* fpout;


static void Serial_ReceiveByte()
{
	char c;
	if(SerialGetChar(&c)== 1)
	    {
               fwrite(&c, 1, sizeof(char), fpout);
		//printf("%c", c);
	    }
}

/* uart interrpt function*/
void signal_handler_IO (int status)
{
	Serial_ReceiveByte();
}

int main(int argc, char *argv[])
{
	char* device;
        char* outfile=NULL;
        struct timeval start, stop;
        int runtime=0;
	int elapsetime = 0;

	gettimeofday(&start, NULL);
	/*Parameters*/    
	if( argc < 4){
		printf("FORMAT: emotemonitor runtime  outfile serialdev \n");
		return 0;
	}else{
		runtime = atoi(argv[1]);
		outfile = argv[2];
		device = argv[3];

	}

	/*Print version information*/
	printf("Emote monitor version 1.0.0, COPYRIGHT DNC 2016.\n");

	/*Open input file*/

	/*Open output file*/
	fpout = fopen(outfile, "wr");
	if(fpout == NULL)
	{
		printf("Fail to Open log file. \n");
	}

	/*Open serial port*/
	if(SerialInt(device) < 0)
	{
		printf("Fail to open serial port %s.\n", device);
		return 0;
	}

	while( elapsetime < runtime)
	{
		sleep(10);
                gettimeofday(&stop, NULL);
		elapsetime = (stop.tv_sec - start.tv_sec)/60 ;
	}
        printf("Monitor is ending.\n");

        fclose(fpout);
	SerialClose();
	return 1;
}



