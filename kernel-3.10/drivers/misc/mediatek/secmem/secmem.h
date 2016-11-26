#ifndef SECMEM_H
#define SECMEM_H

#include "secmem_plat.h"

#define SECMEM_NAME     "secmem"

#define MAX_NAME_SIZE   32

struct secmem_param {
	u32 alignment;		/* IN */
	u32 size;		/* IN */
	u32 refcount;		/* INOUT */
	u32 sec_handle;		/* OUT */
#ifdef SECMEM_DEBUG_DUMP
	uint32_t id;
	uint8_t owner[MAX_NAME_SIZE];
	uint32_t owner_len;
#endif
};

#define SECMEM_IOC_MAGIC      'T'
#define SECMEM_MEM_ALLOC      _IOWR(SECMEM_IOC_MAGIC, 1, struct secmem_param)
#define SECMEM_MEM_REF        _IOWR(SECMEM_IOC_MAGIC, 2, struct secmem_param)
#define SECMEM_MEM_UNREF      _IOWR(SECMEM_IOC_MAGIC, 3, struct secmem_param)
#define SECMEM_MEM_ALLOC_TBL  _IOWR(SECMEM_IOC_MAGIC, 4, struct secmem_param)
#define SECMEM_MEM_UNREF_TBL  _IOWR(SECMEM_IOC_MAGIC, 5, struct secmem_param)
#define SECMEM_MEM_USAGE_DUMP _IOWR(SECMEM_IOC_MAGIC, 6, struct secmem_param)
#ifdef SECMEM_DEBUG_DUMP
#define SECMEM_MEM_DUMP_INFO  _IOWR(SECMEM_IOC_MAGIC, 7, struct secmem_param)
#endif

#define SECMEM_IOC_MAXNR      (10)

#endif				/* end of SECMEM_H */
