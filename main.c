/*
 * STM32ISP - CLI tool for ST STM32F10x In-System Programming
 *
 *  	copyright (c) 2011 BA5AG (ba5ag.kai@gmail.com)
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
#include <stdlib.h>
#include <string.h>
#include "stm32.h"
#include "hexfile.h"
#include "memmap.h"
#include "term.h"

typedef struct {
	char* 	portName;
	int 	baudrate;
	char* 	fileName;
	int 	isBin;
	int		reset;
	int		bootp;
	int		termBaud;
	char*	logFile;
	int		isDetect;
	int		isTerm;
	int		isEcho;
	int		isLine;
	int		isVerbose;
} Config;

static void prompt()
{
	printf(
"Syntax:  stm32isp [Options] <file> <comport> <baudrate>\n"
"\n"
"Example: stm32isp test.hex /dev/ttyS0 115200\n"
"\n"
"Options: -bin              for uploading binary file\n"
"         -reset [-]DTR|RTS reset control line, default DTR\n"
"         -bootp [-]DTR|RTS bootp control line, default RTS\n"
"         -term <baud>      for starting terminal after upload with buadrate, default 9600\n"
"         -detectonly       detect chip set only\n"
"         -termonly         for starting terminal without an upload\n"
"         -noreset          do not reset the board in termonly mode\n"
"         -echo             local echo in term\n"
"         -line             local line edit in term\n"
"         -verbose          for more information\n"
"         -log <file name>  for enabling logging of terminal output to <file name>\n"
	);
}

static int parsePin(char* s)
{
	int ret = 0;
	if ( strstr(s, "DTR") ) {
		ret = DTR;
	} else if ( strstr(s, "RTS") ) {
		ret = RTS;
	}
	if ( s[0] == '-' )
		ret = -ret;
	return ret;
}

static void dumpOptions(Config *p)
{
	printf("Options:\n");
	printf("\tfile name:%s\n", p->fileName);
	printf("\tfile type:%s\n", p->isBin?"bin":"hex");
	printf("\tserial port:%s\n", p->portName);
	printf("\tbaud rate:%d\n", p->baudrate);
	if ( p->reset >0 ) {
		printf("\treset use:%s%s\n", p->reset<0?"-":"", abs(p->reset)==DTR?"DTR":"RTS");
	}
	if ( p->bootp >0 ) {
		printf("\tbootp use:%s%s\n", p->bootp<0?"-":"", abs(p->bootp)==DTR?"DTR":"RTS");
	}
	if ( p->termBaud >0 ) {
		printf("\tterm baud:%d\n", p->termBaud);
	} else {
		printf("\tterm no\n");
	}
	if ( p->isDetect ) {
		printf("\tdetect chip set only\n");
	}
	if ( p->isTerm ) {
		printf("\tterm only\n");
	}
	if ( p->isEcho ) {
		printf("\tlocal echo in term\n");
	}
	if ( p->isLine ) {
		printf("\tlocal line edit in term\n");
	}
	if ( p->isVerbose ) {
		printf("\tin verbose mode\n");
	}
}

static int verifyCondig(Config *p)
{
	int ret = 1;
	if ( p->baudrate <1200 || p->baudrate >115200 ) {
		printf("Not valid baudrate:%d\n", p->baudrate);
		ret = 0;
	} else if ( p->termBaud >115200 ) {
		printf("Not valid baudrate:%d for term\n", p->termBaud);
		ret = 0;
	} else if ( p->isDetect && p->isTerm ) {
		ret = 0;
	}
	return ret;
}

static int parseOptions(int argc, char* argv[], Config* p)
{
	int i;
	int argidx = 0;
	int ret = 0;

	memset(p, 0, sizeof(Config));	//	init the config

	for ( i=1; i< argc; i++ ) {
		if ( argv[i][0] == '-' ) {	//	an option
			if ( strcmp(argv[i], "-bin") ==0 ) {
				p->isBin = 1;
			} else if ( strcmp(argv[i], "-reset") ==0 ) {
				p->reset = parsePin(argv[i+1]);
				i++;
			} else if ( strcmp(argv[i], "-bootp") ==0 ) {
				p->bootp = parsePin(argv[i+1]);
				i++;
			} else if ( strcmp(argv[i], "-term") ==0 ) {
				sscanf(argv[i+1], "%d", &(p->termBaud));
				i++;
			} else if ( strcmp(argv[i], "-detectonly") ==0 ) {
				p->isDetect = 1;
			} else if ( strcmp(argv[i], "-termonly") ==0 ) {
				p->isTerm = 1;
			} else if ( strcmp(argv[i], "-echo") ==0 ) {
				p->isEcho = 1;
			} else if ( strcmp(argv[i], "-line") ==0 ) {
				p->isLine = 1;
			} else if ( strcmp(argv[i], "-verbose") ==0 ) {
				p->isVerbose = 1;
			} else if ( strcmp(argv[i], "-log") ==0 ) {
				p->logFile = argv[i+1];
				i++;
			} else {
				printf("Unknown option: %s\n", argv[i]);
				ret = 0;
				break;
			}
		} else {
			switch ( argidx ) {
			case 0:
				p->fileName = argv[i];
				break;
			case 1:
				p->portName = argv[i];
				break;
			case 2:
				sscanf(argv[i], "%d", &(p->baudrate));
				break;
			}
			argidx ++;
		}
	}

	if ( argidx == 3 ) {
		if ( p->isVerbose ) {
			dumpOptions(p);
		}
		ret = 1;
	}

	return ret;
}

int main(int argc, char *argv[])
{
	Config config;

	memset((void*)&config, 0, sizeof(config));
	config.baudrate = 115200;
	config.termBaud = 9600;
	config.bootp = RTS;
	config.reset = DTR;

	printf(
	"Portable command line ISP for STM32F10x family\n"
	"Version 0.1 compiled for Apple MacOS X: "__DATE__ " " __TIME__ "\n"
	"Copyright (C) by Weng Kai, 2011-2015, Email: ba5ag.kai@gmail.com\n"
	"\n");

	if ( parseOptions(argc, argv, &config) ==0 || verifyCondig(&config) ==0 ) {
		prompt();
		exit(-1);
	}

//	term(config.portName,19200,config.bootp, config.reset, config.isEcho, config.isLine);

	if ( !config.isTerm ) {
		if ( stm32Init(config.portName, config.baudrate, config.bootp, config.reset) == STM_OK ) {
			stm32CmdGet();
	//		stm32GetVersion();
			stm32GetID();
			if ( !config.isDetect ) {
				MemMap mp;
				if ( readHexFile(config.fileName, &mp) ) {
					stm32Erase();
					stm32Write(&mp);
					stm32Verify(&mp);
					stm32Go(0x08000000);
					stm32Close();
					mmapDelete(&mp);
					printf("STM32isp finished.\n");
				} else {
					printf("Can not open file %s.\n", config.fileName);
				}
			}
		} else {
			printf("Can not connect to board.\n");
		}
	}
	if ( config.termBaud >0 ) {
		printf("Starting term.\n");
		term(config.portName,config.termBaud,config.bootp, config.reset, config.isEcho, config.isLine);
	}

	return 0;
}

