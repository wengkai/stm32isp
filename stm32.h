/*
 *  stm32.h - implementing STM32F10x-ISP functions
 *	part of STM32ISP
 *  	copyright (c) 2011 Kai (ba5ag.kai@gmail.com)
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

#ifndef __STM32_H
#define __STM32_H

#include <stdint.h>
#include "memmap.h"

enum {
	STM_ERR = -1,
	STM_OK  = 0
};

enum {
	DTR = 1,
	RTS = 2,
};

int stm32Init(const char* port, unsigned int baudrate, int bootp, int reset);
void stm32Close();

void stm32Reboot(int fd, int reset);
int stm32Reboot2isp();
int stm32Sync();

int stm32CmdGet();
int stm32GetVersion();
int stm32GetID();
int stm32Read(unsigned char *data, unsigned int len, unsigned int addr);
int stm32Verify(MemMap* mm);
int stm32Erase();
int stm32Write(MemMap* mp);
int stm32Go(unsigned int addr);

#endif
