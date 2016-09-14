/*
   stm32flash - Open Source ST STM32 flash program for *nix
   Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>

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



#include <stdio.h>
#include "serial.h"
#include "serial_posix.h"

serial_t *h;

int SerialInt(char *device)
{
	speed_t	port_baud = B115200; /*115200*/
	tcflag_t port_bits = CS8;      /*8 data bits*/
	tcflag_t port_parity = 0;      /*no parity*/
	tcflag_t port_stop = 0;        /*1 stop bit*/
	h = serial_open(device);
	if(h == NULL){
		printf(" can not open the device.");
		return -1;
	}

	/*serial_it_config(h);*/

	if ( serial_setup(h,port_baud,port_bits,port_parity,port_stop)!= 0 ){
		printf(" device initialization failed.");
		return -1;
	}

	return 0;
}

int SerialGetChar(char * c)
{
	return serial_read(h,c,1);
}

int SerialPutChar(const char c)
{
	int cnt;
	cnt = serial_write(h,(void *)&c,1);
	if(cnt !=1)
		printf("write 1 byte failed.\n");
	return cnt;
}

int SerialRead(void *buf,size_t nbyte)
{
	return serial_read(h,buf,nbyte);
}

int SerialWrite(const void *buf,size_t nbyte)
{
	int cnt;
	cnt = serial_write(h,(void *)buf,nbyte);
	if(cnt !=nbyte)
		printf("write %d byte failed.\n",(int)nbyte);
	return cnt;
}

void SerialClose()
{
	serial_close(h);
}
