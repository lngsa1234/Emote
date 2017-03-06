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

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/signal.h>

#ifndef _SERIAL_H
#define _SERIAL_H


int SerialInt(char *device);
int SerialGetChar(char * c);
int SerialPutChar(const char c);
int SerialRead(void *buf,size_t nbyte);
int SerialWrite(const void *buf,size_t nbyte);
void SerialClose();
#endif
