/*
 * term.c
 *
 *  Created on: 2012-1-2
 *      Author: wengkai
 */
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "term.h"
#include "serial.h"
#include "stm32.h"

#define ESC_CHAR	29

static void setBootp(int fd, int bootp)
{
	if ( bootp == RTS ) {
		serialResetPin(fd, TIOCM_RTS);	//RTS->bootp
	} else if ( bootp == -RTS ) {
		serialSetPin(fd, TIOCM_RTS);	//-RTS->bootp
	} if ( bootp == DTR ) {
		serialResetPin(fd, TIOCM_DTR);	//DTR->bootp
	} else if ( bootp == -DTR ) {
		serialSetPin(fd, TIOCM_DTR);	//-DTR->bootp
	}
}

int term(char* portName, int baud, int bootp, int reset, int isEcho, int isLine)
{
	int ret = SER_ERR;
	int fd;
	struct termios oldt;
	struct termios newt;
	char ch = 0;
#ifdef __APPLE__
	struct 
#endif
	fd_set fdr;
	int r;
	fd = serialOpen(portName, baud, 8, 'N', 1);
	if ( fd >0 ) {
		setBootp(fd, bootp);
		stm32Reboot(fd, reset);
		printf("Board rebooted.\n");
		printf("Ctrl-] to quit\n");
		tcgetattr( STDIN_FILENO, &oldt );
		newt = oldt;
		if ( !isEcho )
			newt.c_lflag &= ~( ECHO );
		if ( !isLine)
			newt.c_lflag &= ~( ICANON );
		tcsetattr( STDIN_FILENO, TCSANOW, &newt );
		FD_ZERO(&fdr);
		FD_SET(STDIN_FILENO, &fdr);
		FD_SET(fd, &fdr);
		while ( 1 ) {
			r = select(fd+1, &fdr, NULL, NULL, NULL);
			if ( r == -1 ) {	//	failed
				printf("SerialTerm: read failed\n");
				ret = SER_ERR;
				break;
			} else if ( r == 0 ) {	//	timed-out
				printf("SerialTerm: timed-out\n");
				ret = SER_TO;
				break;
			} else if ( r != 0 ) {
				if ( FD_ISSET(STDIN_FILENO, &fdr) ) {
					ch = getchar();
					if ( ch == ESC_CHAR )
						break;
					write(fd, &ch, 1);
				}
				if ( FD_ISSET(fd, &fdr) ) {
					read(fd, &ch, 1);
					putchar(ch);
				}
			}
			FD_ZERO(&fdr);
			FD_SET(STDIN_FILENO, &fdr);
			FD_SET(fd, &fdr);
		}
		tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
		serialClose(fd);
	} else {
		printf("SerialTerm: Can not open serial port:%s.\n", portName);
	}
	return ret;
}
