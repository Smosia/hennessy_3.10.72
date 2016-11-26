/*
 *  linux/include/asm/setup.h
 *
 *  Copyright (C) 1997-1999 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Structure passed to kernel to tell it about the
 *  hardware it's running on.  See Documentation/arm/Setup
 *  for more info.
 */
#ifndef _UAPI__ASMARM_SETUP_H
#define _UAPI__ASMARM_SETUP_H

#include <linux/types.h>
#include <mach/dfo_boot.h>
#include <mach/mt_devinfo.h>
#define COMMAND_LINE_SIZE 1024

/* The list ends with an ATAG_NONE node. */
#define ATAG_NONE	0x00000000

struct tag_header {
	__u32 size;
	__u32 tag;
};

/* The list must start with an ATAG_CORE node */
#define ATAG_CORE	0x54410001

struct tag_core {
	__u32 flags;		/* bit 0 = read-only */
	__u32 pagesize;
	__u32 rootdev;
};

/* it is allowed to have multiple ATAG_MEM nodes */
#define ATAG_MEM	0x54410002

struct tag_mem32 {
	__u32	size;
	__u32	start;	/* physical start address */
};

/* it is allowed to have multiple ATAG_MEM nodes */
#define ATAG_MEM64	0x54420002

struct tag_mem64 {
	__u64	size;
	__u64	start;	/* physical start address */
};

/* VGA text type displays */
#define ATAG_VIDEOTEXT	0x54410003

struct tag_videotext {
	__u8		x;
	__u8		y;
	__u16		video_page;
	__u8		video_mode;
	__u8		video_cols;
	__u16		video_ega_bx;
	__u8		video_lines;
	__u8		video_isvga;
	__u16		video_points;
};

/* describes how the ramdisk will be used in kernel */
#define ATAG_RAMDISK	0x54410004

struct tag_ramdisk {
	__u32 flags;	/* bit 0 = load, bit 1 = prompt */
	__u32 size;	/* decompressed ramdisk size in _kilo_ bytes */
	__u32 start;	/* starting block of floppy-based RAM disk image */
};

/* describes where the compressed ramdisk image lives (virtual address) */
/*
 * this one accidentally used virtual addresses - as such,
 * it's deprecated.
 */
#define ATAG_INITRD	0x54410005

/* describes where the compressed ramdisk image lives (physical address) */
#define ATAG_INITRD2	0x54420005

struct tag_initrd {
	__u32 start;	/* physical start address */
	__u32 size;	/* size of compressed ramdisk image in bytes */
};

/* board serial number. "64 bits should be enough for everybody" */
#define ATAG_SERIAL	0x54410006

struct tag_serialnr {
	__u32 low;
	__u32 high;
};

/* board revision */
#define ATAG_REVISION	0x54410007

struct tag_revision {
	__u32 rev;
};

/* initial values for vesafb-type framebuffers. see struct screen_info
 * in include/linux/tty.h
 */
#define ATAG_VIDEOLFB	0x54410008

struct tag_videolfb {
	__u16		lfb_width;
	__u16		lfb_height;
	__u16		lfb_depth;
	__u16		lfb_linelength;
	__u32		lfb_base;
	__u32		lfb_size;
	__u8		red_size;
	__u8		red_pos;
	__u8		green_size;
	__u8		green_pos;
	__u8		blue_size;
	__u8		blue_pos;
	__u8		rsvd_size;
	__u8		rsvd_pos;
};

#ifdef CONFIG_TRUSTY
#define ATAG_MEM_TEE_DESC 0x54410042
/*
mem_desc_t;
*/
#endif

/* command line: \0 terminated string */
#define ATAG_CMDLINE	0x54410009

struct tag_cmdline {
	char	cmdline[1];	/* this is the minimum size */
};

/* acorn RiscPC specific information */
#define ATAG_ACORN	0x41000101

struct tag_acorn {
	__u32 memc_control_reg;
	__u32 vram_pages;
	__u8 sounddefault;
	__u8 adfsdrives;
};

/* footbridge memory clock, see arch/arm/mach-footbridge/arch.c */
#define ATAG_MEMCLK	0x41000402

struct tag_memclk {
	__u32 fmemclk;
};

/* boot information */
#define ATAG_BOOT       0x41000802

struct tag_boot {
	u32 bootmode;
};

/*META com port information*/
#define ATAG_META_COM 0x41000803

struct tag_meta_com {
	u32 meta_com_type; /* identify meta via uart or usb */
	u32 meta_com_id;  /* multiple meta need to know com port id */
};


/* MDINFO */
#define ATAG_MDINFO_DATA 0x41000806
struct tag_mdinfo_data{
	u8 md_type[4];
};

#define ATAG_MASP_DATA         0x41000866
#define NUM_SBC_PUBK_HASH           8
#define NUM_CRYPTO_SEED          16
#define NUM_RID 4

struct tag_masp_data{
	unsigned int rom_info_sbc_attr;
	unsigned int rom_info_sdl_attr;
	unsigned int hw_sbcen;
	unsigned int lock_state;
	unsigned int rid[NUM_RID];
	/*rom_info.m_SEC_KEY.crypto_seed*/
	unsigned char crypto_seed[NUM_CRYPTO_SEED];
	unsigned int sbc_pubk_hash[NUM_SBC_PUBK_HASH];
};

#define ATAG_TEE_DATA 0x41000808

/* general memory descriptor */
typedef struct {
    u64 start;
    u64 size;
} mem_desc_t;

/* mblock is used by CPU */
typedef struct {
	u64 start;
	u64 size;
	u32 rank;	/* rank the mblock belongs to */
} mblock_t;

typedef struct {
	u32 mblock_num;
	mblock_t mblock[4];
} mblock_info_t;

typedef struct {
	u32 rank_num;
	mem_desc_t rank_info[4];
} dram_info_t;

struct tag {
	struct tag_header hdr;
	union {
		struct tag_core		core;
		struct tag_mem32	mem;
		struct tag_mem64	mem64;
		struct tag_videotext	videotext;
		struct tag_ramdisk	ramdisk;
		struct tag_initrd	initrd;
		struct tag_serialnr	serialnr;
		struct tag_revision	revision;
		struct tag_videolfb	videolfb;
		struct tag_cmdline	cmdline;

		/*
		 * Acorn specific
		 */
		struct tag_acorn	acorn;

		/*
		 * DC21285 specific
		 */
		struct tag_memclk	memclk;
		struct tag_boot		boot;
		struct tag_meta_com	meta_com;
		struct tag_devinfo_data	devinfo_data;
		struct tag_masp_data	masp_data;
        tag_dfo_boot     dfo_data;
        struct tag_mdinfo_data mdinfo_data;
		mem_desc_t tee_reserved_mem;
#ifdef PT_ABTC_ATAG
		struct tag_pt_info tag_pt_info;
#endif
#ifdef NAND_ABTC_ATAG
		struct tag_nand_number tag_nand_number;
		flashdev_info_t gen_FlashTable_p;
#endif
	} u;
};

struct tagtable {
	__u32 tag;
	int (*parse)(const struct tag *);
};

#define tag_member_present(tag,member)				\
	((unsigned long)(&((struct tag *)0L)->member + 1)	\
		<= (tag)->hdr.size * 4)

#define tag_next(t)	((struct tag *)((__u32 *)(t) + (t)->hdr.size))
#define tag_size(type)	((sizeof(struct tag_header) + sizeof(struct type)) >> 2)

#define for_each_tag(t,base)		\
	for (t = base; t->hdr.size; t = tag_next(t))


#endif /* _UAPI__ASMARM_SETUP_H */
