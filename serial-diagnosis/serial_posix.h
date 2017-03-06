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


#ifndef _SERIAL_POSIX_H
#define _SERIAL_POSIX_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>

struct serial{
	int fd;
	struct termios oldtio;
	struct termios newtio;
	char setup_str[11];
};

typedef struct serial serial_t;

serial_t *serial_open(const char *device);
void serial_close(serial_t *h);
void serial_flush(const serial_t *h);
int serial_setup(serial_t *h,speed_t port_baud,tcflag_t port_bits,tcflag_t port_parity,tcflag_t port_stop);
void serial_it_config(serial_t *h);
int serial_read(const serial_t *h, void *buf,size_t nbyte);
int serial_write(const serial_t *h, void *buf,size_t nbyte);
#endif
