/*
 *  serial.c - implementing serial port communication functions
 *  	copyright (c) 2012 Kai (ba5ag.kai@gmail.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>

#include "serial.h"

#define TIMEDOUT	2

/**
 * open serial port
 * @return file descriptor of the serial port, or
 * 	SER_ERR,if failed
 */
int serialOpen(const char *device, unsigned int baudrate, unsigned int bits, char parity, unsigned int stop)
{
	int fd=-1;
	struct termios optnew;
	speed_t speed = B19200;

	/* Open serial device */
	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if ( fd >0 ) {
		/* Read with blocking behavior */
		fcntl(fd, F_SETFL, 0);

		/* Get current option */
		tcgetattr(fd, &optnew);

		/* initialize new option to raw input/output */
//		memset(&optnew, 0, sizeof(optnew));
		cfmakeraw(&optnew);
		optnew.c_cc[VMIN ] = 0;
		optnew.c_cc[VTIME] = TIMEDOUT*10;

		/* set baudrate */
		switch (baudrate) {
			case   1200: speed =   B1200;  break;
			case   1800: speed =   B1800;  break;
			case   4800: speed =   B4800;  break;
			case   9600: speed =   B9600;  break;
			case  19200: speed =  B19200;  break;
			case  38400: speed =  B38400;  break;
			case  57600: speed =  B57600;  break;
			case 115200: speed = B115200;  break;
			default:    speed = B19200; break;
		}
		cfsetispeed(&optnew, speed);
		cfsetospeed(&optnew, speed);

		/* Set data bits */
		optnew.c_cflag &= ~CSIZE;
		optnew.c_cflag &= ~CRTSCTS;
		optnew.c_iflag &= ~(ICRNL|IXON);
		optnew.c_cflag |= CLOCAL | CREAD;
		optnew.c_oflag &= ~OPOST;

		switch (bits) {
			case 5: optnew.c_cflag |= CS5; break;
			case 6: optnew.c_cflag |= CS6; break;
			case 7: optnew.c_cflag |= CS7; break;
			default :
				optnew.c_cflag |= CS8; break;
		}

		/* Set parity checking */
		optnew.c_cflag |= PARENB;
		switch (parity) {
			case 'e':
			case 'E': optnew.c_cflag &= ~PARODD; break;
			case 'o':
			case 'O': optnew.c_cflag &=  PARODD; break;
			default :
				optnew.c_cflag &= ~PARENB; break;
		}

		/* Set stop bit(s) */
		if (stop == 2)
			optnew.c_cflag &=  CSTOPB;
		else
			optnew.c_cflag &= ~CSTOPB;
	
		optnew.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );

		/* Apply new option */
		tcsetattr(fd, TCSANOW, &optnew);
	}
	return fd;
}

void serialFlush(int fd)
{
	tcflush(fd, TCIOFLUSH);
}

int serialClose(int fd)
{
	serialFlush(fd);
	close(fd);
	return SER_OK;
}

/**
 * write to serial
 * @param fd the file description for the serial port
 * @param buffer the data to be written
 * @param len the length of the data
 * @return SER_OK or SER_ERR
 */
int serialWrite(int fd, unsigned char *buffer, int len)
{
	int w;
	unsigned char *p;
	int ret = SER_OK;

	p = buffer;

	while (len > 0) {
		w = write(fd, p, len);
		if ( w < 1 ) {
			ret = SER_ERR;
			break;
		}
		
		len -= w;
		p   += w;
	}
	return ret;
}

/**
 * read data with specified length from the serial port
 * @param fd the file descriptor of the serial port
 * @param buffer the buffer for the bytes read
 * @param len the required number of bytes to be read
 * @return  SER_OK if expected number of bytes read, or
 * 			SER_TO if timed-out during the read, or
 * 			SER_ERR if read failed
 */
int serialRead(int fd, unsigned char *buffer, int len)
{
	int ret = SER_OK;
	int r;
#ifdef __APPLE__
	struct 
#endif
	fd_set fdr;
	struct timeval timeout = {3,0};
	int cnt = 0;

	while (cnt < len ) {
		FD_ZERO(&fdr);
		FD_SET(fd, &fdr);
		timeout.tv_sec = TIMEDOUT;
		timeout.tv_usec = 0;
		r = select(fd+1, &fdr, NULL, NULL, &timeout);
		if ( r == -1 ) {	//	failed
			ret = SER_ERR;
			break;
		} else if ( r == 0 ) {	//	timed-out
			ret = SER_TO;
			break;
		} else if ( r != 0 ) {
			if ( FD_ISSET(fd, &fdr) ) {
				r = read(fd , buffer+cnt , len-cnt);
				if ( r >= 0 ) {
					cnt += r;
				}
			}
		}
	}

	return ret;
}

//	TIOCM_RTS
void serialSetPin(int fd, int pin) {
	int set;
	ioctl (fd, TIOCMGET, &set);
	set |= pin;
	ioctl (fd, TIOCMSET, &set);
}

void serialResetPin(int fd, int pin) {
	int set;
	ioctl (fd, TIOCMGET, &set);
	set &= ~pin;
	ioctl (fd, TIOCMSET, &set);
}
