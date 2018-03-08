/*
 * hexfile.c
 *
 *  Created on: 2011-12-24
 *      Author: wengkai
 */
#include <stdio.h>
#include <stdlib.h>
#include "hexfile.h"

#define MARK ':'

typedef struct {
	unsigned char len;
	unsigned short offset;
	unsigned char type;
	unsigned char *data;
} HexRecord;

static int readRecord(FILE *fp, HexRecord *pRec);

int readHexFile(char* fileName, MemMap *mp)
{
	int ret = 0;

	HexRecord rec;
	unsigned int addrBase;

	mmapInit(mp);

	FILE *fp = fopen(fileName, "r");
	if ( fp ) {
		while ( readRecord(fp, &rec) ) {
			if ( rec.type == 0x04 ) {
				addrBase = rec.data[0] *256 + rec.data[1];
				addrBase <<= 16;
			} else if ( rec.type == 0x00 && rec.len >0 ) {
				mmapPut(mp, addrBase + rec.offset, rec.len, rec.data);
				// printf("%X\n", addrBase + rec.offset);
			}
			free(rec.data);
		}
		fclose(fp);
		ret = 1;
	}

	return ret;
}

static int readRecord(FILE *fp, HexRecord *pRec)
{
	int ret = 0;
	unsigned char ch;
	int i;
	unsigned char chksum ;
	// unsigned char *p = ((unsigned char*)pRec) + 1;
	int r;

	ch = fgetc(fp);
	if ( ch ==  MARK ) {
		fscanf(fp, "%02X", &r);
		pRec->len = r;
		chksum = r;
		pRec->data = (unsigned char*)malloc(r);
		fscanf(fp, "%04X", &r);
		pRec->offset = r;
		chksum += (r&0x0FF) +(r>>8);
		fscanf(fp, "%02X", &r);
		pRec->type = r;
		chksum += r;
		for ( i=0; i<pRec->len; i++ ) {
			fscanf(fp, "%02X", &r);
			pRec->data[i] = r;
			chksum += r;
		}
		fscanf(fp, "%02X\n", &r);
		chksum += r;
//		printf("len=%02X, offset=%04X, type=%02X:", pRec->len, pRec->offset, pRec->type);
//		for ( i=0; i< pRec->len; i++ ) {
//			printf("%02X", pRec->data[i]);
//		}
//		printf("cs=%d\n", chksum);
		if ( chksum == 0 )
			ret = 1;
	}
	return ret;
}
