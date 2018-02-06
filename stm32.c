/*
 *  stm32.c - implementing STM32F10x-ISP functions
 *	part of STM32ISP
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
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <assert.h>
#include <stdlib.h>
#include "serial.h"
#include "stm32.h"
#include "memmap.h"

#define STM32_ACK		0x79
#define STM32_NACK		0x1F

/**
 * maxim length of block for read/write
 */
#define MAXLEN		256
#define MAXTRY		4
#define TM_BOOTP	2000	//	delay after change bootp
#define TM_RESET	200000	//	delay after RESET down

typedef struct
{
	int serial_fd;
//	int error;
	uint8_t version;
	uint8_t cmd[16];
	uint8_t pid[8];
//	int flash;
//	int ram;
	int readprotect;
	int writeprotect;
	int bootp;
	int reset;
}Stm32Info;

static Stm32Info board;

static int sendCommand(unsigned char byte);
static int waitAck();
static unsigned char *addrPacket(unsigned int addr, unsigned char *buffer);
static unsigned char calcChecksum(unsigned char* data, unsigned int len);
static int stm32WriteBlock(unsigned char *data, unsigned int len, unsigned int addr);

/**
 * Init serial port and sync with the board
 * @param port the serial port, for example: /dev/ttyS0 or COM1
 * @param baudrate the bardrate used during the burning
 * @return  STM_OK if the port is inited and the board is connected;
 * 			STM_ERR if the port can not be opened, or the board can not be connected
 */
int stm32Init(const char* port, unsigned int baudrate, int bootp, int reset)
{
	int ret = STM_ERR;
	board.bootp = bootp;
	board.reset = reset;
	ret = serialOpen(port, baudrate, 8, 'E', 1);
	if ( ret>0 ) {
		board.serial_fd = ret;
		printf("Serial port %s open.\n", port);
		ret = stm32Reboot2isp();
//		ret = stm32Sync();
		if ( ret == STM_ERR ) {
			serialClose(board.serial_fd);
		}
	} else {
		printf("Serial port %s open failed.\n", port);
	}
	return ret;
}

/**
 * close serial port
 */
void stm32Close()
{
	serialClose(board.serial_fd);
}

int stm32Reboot2isp()
{
	int ret = STM_ERR;
	int i;

	for ( i=0; ret == STM_ERR && i< MAXTRY; i++ ) {
		if ( board.bootp == RTS ) {
			serialSetPin(board.serial_fd, TIOCM_RTS);	//RTS->bootp
		} else if ( board.bootp == -RTS ) {
			serialResetPin(board.serial_fd, TIOCM_RTS);	//-RTS->bootp
		} if ( board.bootp == DTR ) {
			serialSetPin(board.serial_fd, TIOCM_DTR);	//DTR->bootp
		} else if ( board.bootp == -DTR ) {
			serialResetPin(board.serial_fd, TIOCM_DTR);	//-DTR->bootp
		}
		usleep(TM_BOOTP);
		printf("Reboot board to isp. ");
		stm32Reboot(board.serial_fd, board.reset);
		ret = stm32Sync();
//		printf("ret=%d\n",ret);
	}
	return ret;
}

void stm32Reboot(int fd, int reset)
{
	printf("Booting...\n");
	if ( reset == RTS ) {
		serialSetPin(fd, TIOCM_RTS);	//RTS
	} else if ( reset == -RTS ) {
		serialResetPin(fd, TIOCM_RTS);	//-RTS
	} if ( reset == DTR ) {
		serialSetPin(fd, TIOCM_DTR);	//DTR
	} else if ( reset == -DTR ) {
		serialResetPin(fd, TIOCM_DTR);	//-DTR
	}
//	serialSetPin(board.serial_fd, TIOCM_DTR);
	usleep(TM_RESET);
	if ( reset == RTS ) {
		serialResetPin(fd, TIOCM_RTS);	//RTS
	} else if ( reset == -RTS ) {
		serialSetPin(fd, TIOCM_RTS);	//-RTS
	} if (reset == DTR ) {
		serialResetPin(fd, TIOCM_DTR);	//DTR
	} else if ( reset == -DTR ) {
		serialSetPin(fd, TIOCM_DTR);	//-DTR
	}
//	serialResetPin(board.serial_fd, TIOCM_DTR);
	usleep(TM_RESET);
}

int stm32Sync()
{
	int ret = STM_ERR;
	static unsigned char STM32_INIT = 0x7F;
	unsigned char r;
	int rr;
	int i;

	printf("Sync");
	fflush(stdout);
	for ( i=0; i<MAXTRY; i++ ) {
		printf(".");
		fflush(stdout);
		rr = serialWrite(board.serial_fd, &STM32_INIT, 1);
		if ( rr == SER_OK ) {
			serialFlush(board.serial_fd);
			rr = serialRead(board.serial_fd, &r, sizeof(r));
			if ( rr == SER_OK ) {
				if ( r==STM32_ACK || r==STM32_NACK) {
					ret = STM_OK;
					break;
				}
			}
		}
	}
	printf("\n");

	if ( ret == STM_OK ) {
		printf("Connected to board.\n");
	} else {
		printf("Can not connect to board, ");
		if ( rr == SER_ERR ) {
			printf(" failed.\n");
		} else {
			printf(" timed-out.\n");
		}
	}

	return ret;
}

int stm32CmdGet()
{
	int ret = STM_ERR;

	if ( sendCommand(0x00)==STM_OK ) {
		int i;
		unsigned char num;

		serialRead(board.serial_fd, &num, 1);	//	number of bytes
		serialRead(board.serial_fd, &board.version, 1);
		serialRead(board.serial_fd, board.cmd, num);

		if ( waitAck()==STM_OK ) {
			printf("Bootloader version: %x.%x\nCommand set-[", board.version >> 4, board.version & 0x0F);
			for ( i=0; i < num; i++ )
				printf(" %02X", board.cmd[i]);
			printf("]\n");
			ret = STM_OK;
		}
	}

	if ( ret == STM_ERR ) {
		printf("STM32-Get: command error!\n");
	}

	return ret;
}


int stm32GetVersion()
{
	int ret = STM_ERR;

	if ( sendCommand(0x01) == STM_OK ) {
		unsigned char buf[3];

		serialRead(board.serial_fd, buf, 3);

		printf("Bootloader version: %x.%x\n", buf[0] >> 4, buf[0] & 0x0F);
		printf("%02X-%02X\n", buf[1], buf[2]);

		if ( waitAck() == STM_OK ) {
			ret = STM_OK;
		}
	}

	if ( ret == STM_ERR ) {
			printf("STM32-GetVersion: command error!\n");
	}
	return ret;
}

int stm32GetID()
{
	int ret = STM_ERR;

	if (sendCommand(0x02) == STM_OK ) {
		unsigned char num;
		int i;

		serialRead(board.serial_fd, &num, 1);
		serialRead(board.serial_fd, board.pid, num+1);

		printf("Product ID=");
		for ( i=0; i<num+1; i++ ) {
			printf("%02X", board.pid[i]);
		}
		printf("\n");

		if ( waitAck() == STM_OK ) {
			ret = STM_OK;
		}
	}

	if ( ret == STM_ERR ) {
		printf("Command-getID: command error!\n");
	}

	return ret;
}

/**
 * Read from the chip.
 * It read data from anywhere in the chip: falsh or ram. The max length can be read is 256-byte.
 */
int stm32Read(unsigned char *data, unsigned int len, unsigned int addr)
{
	int ret = STM_ERR;

	unsigned char addr_buffer[5];

//	printf("STM32-Read: %08X(%d)\n", addr, len);

	if ( sendCommand(0x11) == STM_OK ) {
		serialWrite(board.serial_fd, addrPacket(addr, addr_buffer), 5);
		if ( waitAck() == STM_OK ) {
			if ( sendCommand(len - 1) == STM_OK ) {
				if ( serialRead(board.serial_fd, data, len) == STM_OK) {
					ret = STM_OK;
				} else {
					printf("STM32-Read: data error!\n");
				}
			} else {
				printf("STM32-Read: length error!\n");
			}
		} else {
			printf("STM32-Read: address error!\n");
		}
	} else {
		printf("STM32-Read: command error!\n");
	}

	return ret;
}

int stm32Verify(MemMap* mm)
{
	int ret = STM_ERR;
	MemBlock* pBlock;

	printf("STM-32 Verifying\n");
	for ( pBlock = mm->head; pBlock; pBlock = pBlock->next ) {
		unsigned int address = pBlock->address;
		unsigned int loc = 0;
		unsigned char buf[MAXLEN];
		printf("%08X:", address);
		fflush(stdout);
		while ( address+loc < pBlock->lastValid ) {
			int size = pBlock->lastValid - (address + loc);
//			printf("address=%08X, loc=%d\n", address, loc);
			if ( size > MAXLEN)
				size = MAXLEN;
			if ( stm32Read(buf, size, address+loc) == STM_OK ) {
				int i;
				for ( i=0; i<size; i++ ) {
					if ( pBlock->data[loc] != buf[i] ) {
						printf("\nSTM32-Verify: not match at %08X:data=%02X,chip=%02X.\n", address+loc, pBlock->data[loc], buf[i]);
						goto EXIT;
					}
					loc++;
				}
				printf(".");fflush(stdout);
			} else {
				printf("\nSTM32-Verify: can not read!\n");
				goto EXIT;
			}
		}
		printf("\n");
	}
	printf("STM32 Verity success.\n");
EXIT:
	return ret;
}

int stm32Erase()
{
	int ret = STM_ERR;
	unsigned char buffer[] = {0xFF, 0xFF, 0x00};

	printf("\nSTM32-Erase: %X\n", board.cmd[6]);
	if ( board.cmd[6] == 0x43 ) {
		if ( sendCommand(0x43) == STM_OK ) {
			if ( sendCommand(0xFF) == STM_OK ) {
				ret = STM_OK;
			} else {
				printf("STM32-GlobalErase: fail\n");
			}
		} else {
			printf("STM32-Erase: fail\n");
		}
	} else {
		if ( sendCommand(0x44) == STM_OK ) {
			serialWrite(board.serial_fd, buffer, sizeof(buffer));
			if ( waitAck() == STM_OK ) {
				ret = STM_OK;
			} else {
				printf("STM32-GlobalExErase: fail\n");
			}
		} else {
			printf("STM32-ExErase: fail\n");
		}
	}

	if ( ret == STM_OK ) {
		printf("STM32-Erase: OK\n");
	}

	return ret;
}

int stm32Write(MemMap* mp)
{
	int ret = STM_ERR;
	MemBlock *pBlock;
	for ( pBlock = mp->head; pBlock; pBlock = pBlock->next ) {
		unsigned int loc = 0;
		printf("STM32Wirte: %08X", pBlock->address+loc);
		while ( pBlock->address+loc < pBlock->lastValid ) {
			int size = pBlock->lastValid - (pBlock->address + loc);
			if ( size > MAXLEN) {
				size = MAXLEN;
			}
			if ( size %4 ) {
				size += 4-size%4;
			}
			printf(".");
			fflush(stdout);
			stm32WriteBlock(pBlock->data+loc, size, pBlock->address+loc);
			loc+=size;
		}
		printf("...Done.\n");
	}
	printf("STM32Write: success.\n");
	return ret;
}

int stm32Go(unsigned int addr)
{
	int ret = STM_ERR;
	unsigned char addr_buffer[5];

	printf("STM32-Go: %08X\n", addr);
	if ( sendCommand(0x21) == STM_OK ) {
		serialWrite(board.serial_fd, addrPacket(addr, addr_buffer), 5);
		if (waitAck() == STM_OK ) {
			printf("STM32-Go: success\n");
			ret = STM_OK;
		} else {
			printf("STM32-Go: address error!\n");
		}

	} else {
		printf("STM32-Go: command error!\n");
	}

	return ret;
}

static int stm32WriteBlock(unsigned char *data, unsigned int len, unsigned int addr)
{
	int ret = STM_ERR;
	unsigned char addr_buffer[5];
	unsigned char slen;
	unsigned char cs;

	assert(len%4 == 0);
	assert(len<=MAXLEN);

	slen = (unsigned char)(len-1);
	cs = calcChecksum(data, len);
	if ( sendCommand(0x31) == STM_OK ) {
		serialWrite(board.serial_fd, addrPacket(addr, addr_buffer), 5);
		if ( waitAck() == STM_OK ) {
			serialWrite(board.serial_fd, &slen, 1);
			serialWrite(board.serial_fd, data, len);
			serialWrite(board.serial_fd, &cs, 1);
			if ( waitAck() == STM_OK ) {
				ret = STM_OK;
			} else {
				printf("STM32-Write: data error!\n");
			}
		} else {
			printf("STM32-Write: address error!\n");
		}
	} else {
		printf("STM32-Write: command error!\n");
	}

	return ret;
}

//---------------------------------------------------------------------
/**
 * send a byte to the board and get the ACK
 * @return STM_OK or STM_ERR if NAK received or communication failed
 */
static int sendCommand(unsigned char byte)
{
	unsigned char xorbyte;
	xorbyte = byte ^ 0xFF;

//	printf("byte=%02X\n",byte);
	serialFlush(board.serial_fd);
	serialWrite(board.serial_fd,&byte, 1);
	serialWrite(board.serial_fd,&xorbyte, 1);

	return waitAck();
}

/**
 * read and check one byte if it is ACK
 * @return STM_OK or STM_ERR if NAK received or communication failed
 */
static int waitAck()
{
	int ret = STM_ERR;
	int rr;

	unsigned char r;
	rr = serialRead(board.serial_fd, &r, sizeof(r));
//	printf("rr=%d, r=%d\n", rr, r);
	if ( rr == SER_OK ) {
		if ( r == STM32_ACK )
			ret = STM_OK;
	}

	return ret;
}

static unsigned char *addrPacket(unsigned int addr, unsigned char *buffer)
{
	buffer[0] = (addr >> 24) & 0xFF;
	buffer[1] = (addr >> 16) & 0xFF;
	buffer[2] = (addr >> 8)  & 0xFF;
	buffer[3] = (addr >> 0)  & 0xFF;
	buffer[4] = buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3];

	return buffer;
}

static unsigned char calcChecksum(unsigned char* data, unsigned int len)
{
	int i;
	unsigned char cs = (unsigned char)(len-1);
	for ( i=0; i<len; i++ )
		cs ^= data[i];
	return cs;
}

#if 0



int stm32_readprotect(int x)
{
	uint8_t command;

	(x > 0)? (command = 0x82): (command = 0x92);
 
	if (sendCommand(command))
	{
		printf("STM32-%x: command fail!\n", command);
		return 1;
	}

	if (waitAck())
		return 2;

	stm32f10x.readprotect = x;
	return 0;
}

int write_unprotect(void)
{
	if (sendCommand(0x73))
	{
		printf("STM32-WriteUnprotect: command fail!\n");
		return 1;
	}

	if (waitAck())
	{
		printf("STM32-WriteUnprotect: unprotect fail!\n");
		return 2;
	}


	stm32f10x.writeprotect = 0;
	return 0;
}

int write_protect(void)
{
	uint8_t buffer[MAXLEN+2];
	unsigned int i;

	if (sendCommand(0x63))
	{
		printf("STM32-WriteProtect: command fail!\n");
		return 1;
	}

	buffer[0] = MAXLEN - 1;
	buffer[MAXLEN+1] = buffer[0];
	for (i=1; i<=MAXLEN; i++)
	{
		buffer[i] = i - 1;
		buffer[MAXLEN+1] ^= buffer[i];
	}

	serialWrite(buffer, MAXLEN+2);

	if (waitAck())
	{
		printf("STM32-WriteProtect: protect fail!\n");
		return 2;
	}

	stm32f10x.writeprotect = 0;
	return 0;
}

int stm32_writeprotect(int x)
{
	int ret;
	if (x > 0)
		ret = write_protect();
	else
		ret = write_unprotect();

	return ret;
}




#endif

