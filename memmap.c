/*
 * memmap.c
 *
 *  Created on: 2011-12-24
 *      Author: wengkai
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "memmap.h"

void mmapInit(MemMap* mp)
{
	mp->head = 0;
}

void mmapPut(MemMap* mp, unsigned int address, unsigned int len, unsigned char *data)
{
	//	search for valid block
	MemBlock *pBlock;
//	printf("add=%08X,len=%d", address, len);
	for ( pBlock = mp->head; pBlock; pBlock = pBlock->next ) {
		if ( address >= pBlock->address && address < pBlock->address + MMAP_BLOCK_SIZE ) {
			break;
		}
	}
	//	find the block
	if ( pBlock ) {
		unsigned int nlen = len;
//		printf("bk found\n");
		if ( address + len > pBlock->address + MMAP_BLOCK_SIZE ) {
			nlen = pBlock->address + MMAP_BLOCK_SIZE - address;
		}
		memcpy(pBlock->data+(address-pBlock->address), data, nlen);
		pBlock->lastValid = address+nlen;
		if ( nlen < len ) {
//			printf("recusive, len=%d, nlen=%d\n", len, nlen);
			mmapPut(mp, address+nlen, len-nlen, data+nlen);
		}
	} else {
		//	create new block
//		printf("create new bk\n");
		pBlock = (MemBlock*)malloc(sizeof(MemBlock));
		pBlock->next = 0;
		// pBlock->address = address;
		pBlock->address = address/MMAP_BLOCK_SIZE*MMAP_BLOCK_SIZE;
		// printf("create new bk %X at %X\n", address, pBlock->address);
		pBlock->lastValid = address;
		memset(pBlock->data, 0xFF, MMAP_BLOCK_SIZE);	//	None used bytes as 0xFF
		//	add new block
		if ( mp->head ) {
			MemBlock *p;
			for ( p=mp->head; p->next; p=p->next ) {
			}
			p->next = pBlock;
		} else {
			mp->head = pBlock;
		}
		mmapPut(mp, address, len, data);
	}
}

void mmapDelete(MemMap* mp)
{
	MemBlock *p, *q;
	for ( p=mp->head; p; p=q ) {
		q = p->next;
		free(p);
	}
}

void mmapDump(MemMap* mp)
{
	MemBlock *p;
	int i;
	for ( p=mp->head; p; p=p->next ) {
		printf("Address:%08X(%08X)\n  ", p->address, p->lastValid);
		for ( i=0; i<p->lastValid - p->address; i++ ) {
			printf("%02X", p->data[i]);
			if ( (i+1)%16 == 0 ) {
				printf("\n  ");
			}
		}
		printf("\n");
	}
}
