/*
 *  serial.h - implementing serial port communication functions
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

#ifndef __SERIAL_H
#define __SERIAL_H

enum {
	SER_TO  = -2,
	SER_ERR = -1,
	SER_OK  =  0,
};

int serialOpen(const char *device, unsigned int baudrate, unsigned int bits, char parity, unsigned int stop);
int serialClose(int fd);

int serialRead(int fd, unsigned char *buffer, int len);
int serialWrite(int fd,unsigned char *buffer, int len);
void serialFlush(int fd);

void serialSetPin(int fd, int pin);
void serialResetPin(int fd, int pin);

#endif /* __SERIAL_H */
