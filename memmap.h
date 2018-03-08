/*
 * memmap.h
 *
 *  Created on: 2011-12-24
 *      Author: wengkai
 */

#ifndef MEMMAP_H_
#define MEMMAP_H_

#define MMAP_BLOCK_SIZE (4*1024)

typedef struct _mem_block {
	unsigned int address;
	unsigned int lastValid;
	unsigned char data[MMAP_BLOCK_SIZE];
	struct _mem_block *next;
} MemBlock;

typedef struct {
	MemBlock* head;
} MemMap;

void mmapInit(MemMap* mp);
void mmapPut(MemMap* mp, unsigned int address, unsigned int len, unsigned char *data);
void mmapDelete(MemMap* mp);
void mmapDump(MemMap* mp);

#endif /* MEMMAP_H_ */
