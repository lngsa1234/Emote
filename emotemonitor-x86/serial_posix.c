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
#include "serial_posix.h"

extern void signal_handler_IO (int status);    //definition of signal handler

serial_t *serial_open(const char *device)
{
	serial_t *h = calloc(sizeof(serial_t), 1);
	h->fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (h->fd < 0) {
		free(h);
		return NULL;
	}
	fcntl(h->fd, F_SETFL, 0);

	tcgetattr(h->fd, &h->oldtio);
	tcgetattr(h->fd, &h->newtio);

	return h;
}

void serial_flush(const serial_t *h)
{
	tcflush(h->fd, TCIFLUSH);
}

void serial_close(serial_t *h)
{
	serial_flush(h);
	tcsetattr(h->fd, TCSANOW, &h->oldtio);
	close(h->fd);
	free(h);
}

int serial_setup(serial_t *h,speed_t port_baud,tcflag_t port_bits,tcflag_t port_parity,tcflag_t port_stop)
{

	struct termios settings;
	cfmakeraw(&h->newtio);	
	h->newtio.c_iflag &=~(IXON | IXOFF | IXANY);
	h->newtio.c_iflag &=~(INLCR | IGNCR | ICRNL);

	h->newtio.c_cflag &= ~PARENB;
	h->newtio.c_cflag &= ~CSTOPB;
	h->newtio.c_cflag &= ~CSIZE;
	h->newtio.c_cflag &= ~HUPCL; 
	h->newtio.c_oflag &= ~OPOST;

	h->newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/* setup the new settings */
	cfsetispeed(&h->newtio, port_baud);
	cfsetospeed(&h->newtio, port_baud);
	h->newtio.c_cflag |= port_bits;
	h->newtio.c_cflag |= CLOCAL;
	h->newtio.c_cflag |= CREAD;

	h->newtio.c_cc[VMIN] = 0;
	h->newtio.c_cc[VTIME] = 5;	/* in units of 0.1 s */

	/* set the settings */
	serial_flush(h);
	if (tcsetattr(h->fd, TCSANOW, &h->newtio) != 0)
		return -1;

	/* confirm they were set */
	tcgetattr(h->fd, &settings);
	if (settings.c_iflag != h->newtio.c_iflag ||
			settings.c_oflag != h->newtio.c_oflag ||
			settings.c_cflag != h->newtio.c_cflag ||
			settings.c_lflag != h->newtio.c_lflag)
		return -1;

	return 0;
}

void serial_it_config(serial_t *h){
	struct sigaction saio;  
	//install the serial handler before making the device asynchronous
	saio.sa_handler = signal_handler_IO;
	sigemptyset(&saio.sa_mask);   //saio.sa_mask = 0;
	saio.sa_flags = 0;
	saio.sa_restorer = NULL;
	sigaction(SIGIO,&saio,NULL);

	// allow the process to receive SIGIO
	fcntl(h->fd, F_SETOWN, getpid());
	// Make the file descriptor asynchronous (the manual page says only
	// O_APPEND and O_NONBLOCK, will work with F_SETFL...)
	fcntl(h->fd, F_SETFL, FASYNC);

}
int serial_read(const serial_t *h, void *buf,size_t nbyte)
{
	ssize_t r;
	uint8_t *pos = (uint8_t *)buf;
	int cnt = 0;

	if (h == NULL)
		return -1;

	while (nbyte) {
		r = read(h->fd, pos, nbyte);
		if (r == 0)
			return 0;
		if (r < 0)
			return -1;

		nbyte -= r;
		pos += r;
		cnt += r;
	}
	return cnt;
}

int serial_write(const serial_t *h,void *buf,size_t nbyte)
{
	ssize_t r;
	const uint8_t *pos = (const uint8_t *)buf;
	int cnt=0;

	if (h == NULL)
		return -1;
	while (nbyte) {
		r = write(h->fd, pos, nbyte);
		if (r < 1)
			return -1;

		nbyte -= r;
		pos += r;
		cnt += r;

	}
	return cnt;
}
