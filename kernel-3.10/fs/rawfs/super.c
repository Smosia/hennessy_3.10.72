/*
  RAWFS: Raw file system for NAND flash
  Copyright (C) 2012-2015  Perry Hsu <perry.hsu@mediatek.com>"

  This program can be distributed under the terms of the GNU GPL.
  See the file COPYING.
*/

#if defined(CONFIG_MT_ENG_BUILD)
#define DEBUG 1
#endif

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/version.h>
#include <linux/nls.h>
#include <linux/proc_fs.h>
#include <linux/mtd/mtd.h>
#include <linux/statfs.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uidgid.h>
#include <linux/cred.h>
#include <linux/seq_file.h>
#include "rawfs.h"

/* To identify RAW FS from other file systems. */
#define RAWFS_MAGIC	 0x4D524653 /* "MRFS" */

unsigned int rawfs_parm = 0;
module_param(rawfs_parm, int, 0);
MODULE_PARM_DESC(rawfs_parm, "");

/* Debug message level */
int rawfs_debug_msg_mask = RAWFS_DEBUG_MSG_DEFAULT;

/* File System Statistics */

static struct list_head *rawfs_context;
static struct mutex rawfs_context_lock;

static void rawfs_context_init(void)
{
	rawfs_context = kzalloc(sizeof(struct list_head), GFP_NOFS);

	if (rawfs_context)	{
		INIT_LIST_HEAD(rawfs_context);
		mutex_init(&rawfs_context_lock);
	}
}

static void rawfs_context_add(struct super_block *sb)
{
	struct rawfs_sb_info *rawfs_sb;

	rawfs_sb = RAWFS_SB(sb);
	rawfs_sb->super = sb;

	mutex_lock(&rawfs_context_lock);
	INIT_LIST_HEAD(&(rawfs_sb->fs_context));
	list_add_tail(&(rawfs_sb->fs_context), rawfs_context);
	mutex_unlock(&rawfs_context_lock);
}

static void rawfs_context_remove(struct super_block *sb)
{
	struct rawfs_sb_info *rawfs_sb;
	rawfs_sb = RAWFS_SB(sb);

	mutex_lock(&rawfs_context_lock);
	list_del(&(rawfs_sb->fs_context));
	mutex_unlock(&rawfs_context_lock);
}


/* External Variables */
/* extern struct inode_operations rawfs_dir_inode_ops;
extern struct inode_operations rawfs_file_inode_ops;
extern struct file_operations rawfs_dir_operations;
*/

/* Local Macros */
#define rawfs_devname(sb, buf)	bdevname(sb->s_bdev, buf)

/* Process Files */
static struct proc_dir_entry *rawfs_proc_root;

static int rawfs_proc_init(void)
{
	rawfs_proc_root = proc_mkdir("fs/rawfs", NULL);

	if (rawfs_proc_root == NULL)
		return -ENOMEM;
	return 0;
}

static void rawfs_proc_clean(void)
{
	if (rawfs_proc_root == NULL)
		return;

	remove_proc_entry("DebugData", rawfs_proc_root);
	remove_proc_entry("fs/rawfs", NULL);
}

static const char * const rawfs_block_stat_str[] = {
	"INV",
	"EMP",
	"IDT",
	"DAT",
	""
};


static const char * const rawfs_page_stat_str[] = {
	"EMP",
	"DEL",
	"VAL",
	"BKH",
	"GCM",
	"UNC",
	"INV",
	""
};

static inline const char *rawfs_block_stat(int i)
{
	if (i < 0 || i > 4)
		i = 4;
	return rawfs_block_stat_str[i];
}

static inline const char *rawfs_page_stat(int i)
{
	if (i < 0 || i > 7)
		i = 7;
	return rawfs_page_stat_str[i];
}

static struct {
	char *dbg_name;
	int   dbg_bit;
} rawfs_dbg_flags[] = {
	{"init", RAWFS_DBG_INIT},
	{"super", RAWFS_DBG_SUPER},
	{"inode", RAWFS_DBG_INODE},
	{"file",  RAWFS_DBG_FILE},
	{"dir", RAWFS_DBG_DIR},
	{"device", RAWFS_DBG_DEVICE},
	{"dentry", RAWFS_DBG_DENTRY},
	{"gc", RAWFS_DBG_GC},
	{"all", -1},
	{"none", 0},
	{NULL, 0},
};

#define MAX_DBG_NAME_LENGTH 30


static int rawfs_proc_write(struct file *filp, const char __user *buf,
	size_t len, loff_t *ppos)
/* static int rawfs_proc_write(struct file *file, const char *buf, */
/* unsigned long count, void *data) */
{
	int result = 0, dbg_bit;
	char *end;
	char *dbg_name;
	const char *x;
	char substring[MAX_DBG_NAME_LENGTH + 1];
	int i;
	int done = 0;
	int add, slen = 0;
	int pos = 0;

	RAWFS_PRINT(RAWFS_DBG_SUPER, "rawfs_proc_write, current mask %X, %s\n",
		rawfs_debug_msg_mask, buf);

	result = rawfs_debug_msg_mask;

	while (!done && (pos < len)) {
		done = 1;
		while ((pos < len) && isspace(buf[pos]))
			pos++;

		switch (buf[pos]) {
		case '+':
		case '-':
		case '=':
			add = buf[pos];
			pos++;
			break;

		default:
			add = ' ';
			break;
		}
		dbg_name = NULL;

		dbg_bit = simple_strtoul(buf + pos, &end, 0);

		if (end > buf + pos) {
			dbg_name = "numeral";
			slen = end - (buf + pos);
			pos += slen;
			done = 0;
		} else {
			for (x = buf + pos, i = 0;
				 (*x == '_' || (*x >= 'a' && *x <= 'z')) &&
				 i < MAX_DBG_NAME_LENGTH; x++, i++, pos++)
				substring[i] = *x;
			substring[i] = '\0';

			for (i = 0; rawfs_dbg_flags[i].dbg_name != NULL; i++) {
				if (strcmp(substring, rawfs_dbg_flags[i].dbg_name)
					== 0) {
					dbg_name = rawfs_dbg_flags[i].dbg_name;
					dbg_bit =
						rawfs_dbg_flags[i].dbg_bit;
					done = 0;
					break;
				}
			}
		}

		if (dbg_name  != NULL) {
			done = 0;
			switch (add) {
			case '-':
				result &= ~dbg_bit;
				break;
			case '+':
				result |= dbg_bit;
				break;
			case '=':
				result = dbg_bit;
				break;
			default:
				result |= dbg_bit;
				break;
			}
		}
	}

	rawfs_debug_msg_mask = result;

	RAWFS_PRINT(RAWFS_DBG_SUPER, "rawfs_proc_write: new trace filter = %X\n",
		rawfs_debug_msg_mask);

	if (result) {
		for (i = 0; rawfs_dbg_flags[i].dbg_name != NULL; i++) {
			char flag;
			flag = ((result & rawfs_dbg_flags[i].dbg_bit) ==
					 rawfs_dbg_flags[i].dbg_bit) ? '+' : '-';
			RAWFS_PRINT(RAWFS_DBG_SUPER, " %c%s\n", flag,
				rawfs_dbg_flags[i].dbg_name);
		}
	}

	return len;
}


static void *rawfs_seq_debug_start(struct seq_file *seq, loff_t *pos)
{
	struct super_block *sb = seq->private;
	unsigned int block_no;

	if (*pos < 0 || *pos >= RAWFS_NAND_BLOCKS(RAWFS_SB(sb)))
		return NULL;
	block_no = *pos + 1;
	return (void *) ((unsigned long) block_no);
}

static void *rawfs_seq_debug_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct super_block *sb = seq->private;
	unsigned int block_no;

	++*pos;
	if (*pos < 0 || *pos >= RAWFS_NAND_BLOCKS(RAWFS_SB(sb)))
		return NULL;
	block_no = *pos + 1;
	return (void *) ((unsigned long) block_no);
}

static int rawfs_seq_debug_show(struct seq_file *seq, void *v)
{
	struct super_block *sb = seq->private;
	struct rawfs_sb_info *sbi;
	unsigned int block_no = (unsigned int) ((unsigned long) v);
	struct rawfs_block_header bh;
	struct rawfs_file_info fi;
	int i = 0, page, stat_blk;
	char devname_buf[BDEVNAME_SIZE + 1];

	RAWFS_PRINT(RAWFS_DBG_SUPER, "rawfs_seq_debug_show()\n");

	block_no--;

	if (!sb)
		return 0;

	sbi = RAWFS_SB(sb);

	mutex_lock(&sbi->rawfs_lock);

	if (block_no == 0)	{
		seq_printf(seq,
				"-----------------------------\n"
				"[%d] %s @ %d.%d\n"
				" <geometry>\n"
				" -total blocks %d\n"
				" -block size %d\n"
				" -page size %d\n"
				" -pages per block %d\n"
				" -sectors per page %d\n"
				" -data per page %d\n",
				i, rawfs_devname(sbi->super, devname_buf),
					MAJOR(sbi->super->s_dev), MINOR(sbi->super->s_dev),
				sbi->total_blocks,
				sbi->block_size,
				sbi->page_size,
				sbi->pages_per_block,
				sbi->sectors_per_page,
				sbi->page_data_size);
		seq_printf(seq,
				" <management>\n"
				" -data block %d\n"
				" -  free index %d\n"
				" -  gc index %d\n"
				" -empty block %d\n"
				" -max ec %d\n"
				" -seq.no %d\n",
				sbi->data_block,
				sbi->data_block_free_page_index,
				sbi->data_block_gcmarker_page_index,
				sbi->empty_block,
				sbi->erase_count_max,
				sbi->sequence_number);
	}

	stat_blk = rawfs_block_is_valid(sbi->super, block_no, &bh, NULL);

	seq_printf(seq,
		"<block %d> %s, ver=%d, seq_no=%d, last_seq_no=%d, ec=%d, "
		"crc=%X\n",
		block_no,
		rawfs_block_stat(stat_blk),
		bh.i_rawfs_version,
		bh.i_sequence_number,
		bh.i_sequence_number_last,
		bh.i_erase_count,
		bh.i_crc);

	for (page = 0; page < sbi->pages_per_block; page++) {
		int stat_pg;

		stat_pg = rawfs_page_get(sbi->super, block_no, page, &fi,
			NULL);

		if ((stat_pg == RAWFS_PAGE_STAT_VALID) ||
			(stat_pg == RAWFS_PAGE_STAT_DELETED)) {
				seq_printf(seq,
					" -%d %s %s (%d/%d)@%X id=%X %dB\n",
					page,
					rawfs_page_stat(stat_pg),
					fi.i_name,
					fi.i_chunk_index,
					fi.i_chunk_total,
					fi.i_parent_folder_id,
					fi.i_id,
					(unsigned)fi.i_size);
		} else if (stat_pg == RAWFS_PAGE_STAT_EMPTY) {
			continue;
		} else {
				seq_printf(seq,
					" -%d %s\n",
					page,
					rawfs_page_stat(stat_pg));
		}
	}


	mutex_unlock(&sbi->rawfs_lock);

	return 0;
}

static void rawfs_seq_debug_stop(struct seq_file *seq, void *v)
{
}

static const struct seq_operations rawfs_seq_debug_ops = {
	.start  = rawfs_seq_debug_start,
	.next   = rawfs_seq_debug_next,
	.stop   = rawfs_seq_debug_stop,
	.show   = rawfs_seq_debug_show,
};

static int rawfs_seq_debug_open(struct inode *inode, struct file *file)
{
	struct super_block *sb = PDE_DATA(inode);
	int rc;

	rc = seq_open(file, &rawfs_seq_debug_ops);
	if (rc == 0) {
		struct seq_file *m = file->private_data;
		m->private = sb;
	}
	return rc;

}

static const struct file_operations rawfs_seq_debug_fops = {
	.owner		= THIS_MODULE,
	.open		= rawfs_seq_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
	.write		= rawfs_proc_write,
};


static unsigned rawfs_calc_shifts(unsigned a)
{
	unsigned bits = 0;

	if (!a)
		return 0;

	while (!(a & 1)) {
		a >>= 1;
		bits++;
	}

	return bits;
}


static void
rawfs_put_super(struct super_block *sb)
{
	struct rawfs_sb_info *rawfs_sb;

	rawfs_sb = RAWFS_SB(sb);
	if (rawfs_sb == NULL)  /* Empty superblock info passed to unmount */
		return;

	unload_nls(rawfs_sb->local_nls);

	/* FS-FILLIN your fs specific umount logic here */
	if (rawfs_sb->driver_context)
		put_mtd_device(rawfs_sb->driver_context);

	rawfs_context_remove(sb); /* remove from fs context list */

	/* Process File */
	if (rawfs_sb->s_proc) {
		remove_proc_entry("debug", rawfs_sb->s_proc);
		remove_proc_entry(sb->s_id, rawfs_proc_root);
	}

	kfree(rawfs_sb);

	return;
}

/* This will be called when using "df" command */
int rawfs_stat_fs(struct dentry *dentry, struct kstatfs *buf)
{
	unsigned int entry_count, used_blocks, free_blocks;
	int result = 0;

	struct super_block *sb = dentry->d_inode->i_sb;
	struct rawfs_sb_info *rawfs_sb = RAWFS_SB(sb);

	RAWFS_PRINT(RAWFS_DBG_SUPER, "rawfs_stat_fs\n");

	if (!buf)
		result = -EFAULT;
	else
		result = rawfs_file_list_count(sb, &entry_count, &used_blocks,
			&free_blocks);

	if (result)
		goto out;

	buf->f_type = dentry->d_sb->s_magic;	   /* RAWFS_MAGIC */
	buf->f_bsize = rawfs_sb->page_size;		/* PAGE_CACHE_SIZE; */
	buf->f_namelen = RAWFS_MAX_FILENAME_LEN;   /* NAME_MAX */
	buf->f_blocks = rawfs_sb->pages_per_block * rawfs_sb->total_blocks;
	buf->f_ffree = free_blocks;
	buf->f_bavail = free_blocks;
	buf->f_files = entry_count;
	buf->f_ffree = free_blocks;

	RAWFS_PRINT(RAWFS_DBG_SUPER,
		"rawfs_stat_fs: f_type %X, f_blocks %d, f_ffree %d, f_bavail %d, f_namelen %d, f_bsize %d\n",
		(unsigned)buf->f_type,
		(unsigned)buf->f_blocks,
		(unsigned)buf->f_ffree,
		(unsigned)buf->f_bavail,
		(unsigned)buf->f_bsize,
		(unsigned)buf->f_namelen
		);

out:
	return result;
}

/* RAWFS: To be compeleted */
/* Reference: fat_sops */
struct super_operations rawfs_super_ops = {
	.alloc_inode	= rawfs_alloc_inode,
	.destroy_inode	= rawfs_destroy_inode,
/* .write_inode	= rawfs_write_inode,	 // fat_write_inode */
/* .evict_inode	= rawfs_evict_inode,	 // fat_evict_inode */
/* .sync_fs		= rawfs_sync_fs,		 // fat_sync_fs */
	.statfs		 = rawfs_stat_fs,		   /* simple_statfs, */
	.drop_inode	 = generic_delete_inode, /* Not needed, is the default */
	.put_super	  = rawfs_put_super,
};

static void
rawfs_parse_mount_options(char *options, struct rawfs_sb_info *rawfs_sb)
{
	char *value;
	char *data;
	int size;

	/* Default is MTD device */
	rawfs_sb->flags |= RAWFS_MNT_MTD;

	if (!options)
		return;


	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_parse_mount_options: %s\n", options);

	while ((data = strsep(&options, ",")) != NULL) {
		if (!*data)
			continue;
		value = strchr(data, '=');
		if (value != NULL)
			*value++ = '\0';
		if (strnicmp(data, "rsize", 5) == 0) {
			if (value && *value) {
				size = simple_strtoul(value, &value, 0);
				if (size > 0) {
					rawfs_sb->rsize = size;
					RAWFS_PRINT(RAWFS_DBG_INIT, "+ rsize %d\n", size);
				}
			}
		} else if (strnicmp(data, "wsize", 5) == 0) {
			if (value && *value) {
				size = simple_strtoul(value, &value, 0);
				if (size > 0) {
					rawfs_sb->wsize = size;
					RAWFS_PRINT(RAWFS_DBG_INIT, "+ wsize %d\n", size);
				}
			}
		} else if ((strnicmp(data, "nocase", 6) == 0) ||
			   (strnicmp(data, "ignorecase", 10)  == 0)) {
			rawfs_sb->flags |= RAWFS_MNT_CASE;
			RAWFS_PRINT(RAWFS_DBG_INIT, "+ ignore case\n");
		} else if (strnicmp(data, "ram", 3) == 0) {
			rawfs_sb->flags |= RAWFS_MNT_RAM;
			rawfs_sb->flags &= ~RAWFS_MNT_MTD;
			RAWFS_PRINT(RAWFS_DBG_INIT, "+ ram disk\n");
		} else if (strnicmp(data, "mtd", 3) == 0) {
			rawfs_sb->flags |= RAWFS_MNT_MTD;
			rawfs_sb->flags &= ~RAWFS_MNT_RAM;
			RAWFS_PRINT(RAWFS_DBG_INIT, "+ nand mtd\n");
		} else if (strnicmp(data, "firstboot", 9) == 0) {
			rawfs_sb->flags |= RAWFS_MNT_FIRSTBOOT;
			RAWFS_PRINT(RAWFS_DBG_INIT, "+ first boot\n");
		} else if (strnicmp(data, "blockfile", 9) == 0) {
			rawfs_sb->flags |= RAWFS_MNT_BLOCKFILE;
			RAWFS_PRINT(RAWFS_DBG_INIT, "+ block file\n");
		} else {
			RAWFS_PRINT(RAWFS_DBG_INIT, "+ bad mount option %s\n",
				data);
		}
	}
}

int rawfs_ci_hash(const struct dentry *dentry, const struct inode *inode,
	struct qstr *q)
{
		struct nls_table *codepage = RAWFS_SB(dentry->d_inode->i_sb)->local_nls;
		unsigned long hash;
		int i;

		hash = init_name_hash();
		for (i = 0; i < q->len; i++)
				hash = partial_name_hash(nls_tolower(codepage, q->name[i]),
										 hash);
		q->hash = end_name_hash(hash);

		return 0;
}
EXPORT_SYMBOL_GPL(rawfs_ci_hash);

int rawfs_compare_dentry(const struct dentry *parent,
	const struct inode *pinode, const struct dentry *dentry,
	const struct inode *inode, unsigned int len, const char *str,
	const struct qstr *name)
{
	struct nls_table *codepage = RAWFS_SB(dentry->d_inode->i_sb)->local_nls;

	if (len != name->len)
		return 1;

	return nls_strnicmp(codepage, str, name->name, len);

}
EXPORT_SYMBOL_GPL(rawfs_compare_dentry);

/* No sense hanging on to negative dentries as they are only
in memory - we are not saving anything as we would for network
or disk filesystem */

int rawfs_delete_dentry(const struct dentry *dentry)
{
		RAWFS_PRINT(RAWFS_DBG_DENTRY, "delete entry %s\n", dentry->d_name.name);
		return 1;
}
EXPORT_SYMBOL_GPL(rawfs_delete_dentry);

uint32_t rawfs_div(uint64_t n, uint32_t base)
{
	uint64_t result = n;
	do_div(result, base);
	return (uint32_t) result;
}
EXPORT_SYMBOL_GPL(rawfs_div);

#define RAWFS_DEV_RAM_PAGE_SIZE	   1024
#define RAWFS_DEV_RAM_PAGES_PER_BLOCK 64
#define RAWFS_DEV_RAM_BLOCKS		  2

static void rawfs_dbg_sb_info(struct super_block *sb)
{
	struct rawfs_sb_info *rawfs_sb = RAWFS_SB(sb);

	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  page_size %d\n",
		rawfs_sb->page_size);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  pages_per_block %d\n",
		rawfs_sb->pages_per_block);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  total_blocks %d\n",
		rawfs_sb->total_blocks);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  block_size %d\n",
		rawfs_sb->block_size);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  sectors_per_page %d\n",
		rawfs_sb->sectors_per_page);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  page_data_size %d\n",
		rawfs_sb->page_data_size);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  dev.erase_block %p\n",
		rawfs_sb->dev.erase_block);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  dev.read_page %p\n",
		rawfs_sb->dev.read_page);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  dev.read_page_user %p\n",
		rawfs_sb->dev.read_page_user);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  dev.write_page %p\n",
		rawfs_sb->dev.write_page);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_dbg_sb_info:  driver_context %p\n",
		rawfs_sb->driver_context);
}

#ifdef RAWFS_RAM_DISK
/*
char rawfs_ram_disk[RAWFS_DEV_RAM_PAGE_SIZE*
					RAWFS_DEV_RAM_PAGES_PER_BLOCK*
					RAWFS_DEV_RAM_BLOCKS];
*/

static void rawfs_ram_init(void *data, struct rawfs_sb_info *rawfs_sb)
{
	rawfs_sb->page_size = RAWFS_DEV_RAM_PAGE_SIZE;
	rawfs_sb->pages_per_block = RAWFS_DEV_RAM_PAGES_PER_BLOCK;
	rawfs_sb->total_blocks = RAWFS_DEV_RAM_BLOCKS;

	rawfs_sb->block_size = rawfs_sb->pages_per_block * rawfs_sb->page_size;
	rawfs_sb->sectors_per_page = RAWFS_DEV_RAM_PAGE_SIZE >> 9;
	rawfs_sb->page_data_size = RAWFS_DEV_RAM_PAGE_SIZE - 512;

	rawfs_sb->dev.erase_block = rawfs_dev_ram_erase_block;
	rawfs_sb->dev.read_page = rawfs_dev_ram_read_page;
	rawfs_sb->dev.read_page_user = rawfs_dev_ram_read_page_user;
	rawfs_sb->dev.write_page = rawfs_dev_ram_write_page;

	rawfs_sb->driver_context = NULL;
}
#endif

#ifdef MTK_NAND_PL_TEST
extern void mtk_nand_register_pl_test(u32 base, u32 length, u32 flag);
#endif

static int rawfs_mtd_init(struct super_block *sb, void *data, int silent)
{
	char devname_buf[BDEVNAME_SIZE + 1];
	struct mtd_info *mtd;
	struct rawfs_sb_info *rawfs_sb;

	if (!sb)
		RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init: sb is NULL\n");
	else if (!sb->s_dev)
		RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init: sb->s_dev is NULL\n");
	else if (!rawfs_devname(sb, devname_buf))
		RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init: devname is NULL\n");
	else
		RAWFS_PRINT(RAWFS_DBG_INIT,
			"rawfs_mtd_init: dev is %d name is \"%s\"\n",
			sb->s_dev, rawfs_devname(sb, devname_buf));

	RAWFS_PRINT(RAWFS_DBG_INIT,
		"rawfs_mtd_init: Attempting MTD mount of %u.%u,\"%s\"\n",
		MAJOR(sb->s_dev), MINOR(sb->s_dev), rawfs_devname(sb, devname_buf));

	/* Check it's an mtd device..... */
	if (MAJOR(sb->s_dev) != MTD_BLOCK_MAJOR) {
		RAWFS_PRINT(RAWFS_DBG_INIT,
			"rawfs_mtd_init: Major %d is not a MTD block device\n", MAJOR(sb->s_dev));
		return -1;	/* This isn't an mtd device */
	}

	/* Get the device */
	mtd = get_mtd_device(NULL, MINOR(sb->s_dev));
	if (!mtd) {
		RAWFS_PRINT(RAWFS_DBG_INIT,
			"rawfs_mtd_init:  MTD device #%u doesn't appear to exist",
			MINOR(sb->s_dev));
		return -1;
	}
	/* Check it's NAND */
	if (mtd->type != MTD_NANDFLASH) {
		RAWFS_PRINT(RAWFS_DBG_INIT,
			"rawfs_mtd_init:  MTD device is not NAND it's type %d",
			mtd->type);
		return -1;
	}

	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  erase %p", mtd->_erase);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  read %p", mtd->_read);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  write %p", mtd->_write);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  readoob %p", mtd->_read_oob);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  writeoob %p",
		mtd->_write_oob);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  block_isbad %p",
		mtd->_block_isbad);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  block_markbad %p",
		mtd->_block_markbad);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  write (page) size %d",
		mtd->writesize);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  oobsize %d", mtd->oobsize);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  erase (block) size %d",
		mtd->erasesize);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  size %lld", mtd->size);

	rawfs_sb = RAWFS_SB(sb);

	rawfs_sb->block_size = mtd->erasesize;
	rawfs_sb->page_size = mtd->writesize;

	rawfs_sb->pages_per_block = rawfs_div(rawfs_sb->block_size,
		rawfs_sb->page_size);
	rawfs_sb->total_blocks = rawfs_div(mtd->size, mtd->erasesize);

	rawfs_sb->sectors_per_page = rawfs_sb->page_size >> 9;
	rawfs_sb->page_data_size = rawfs_sb->page_size - 512;

	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  total blocks %d",
		rawfs_sb->total_blocks);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  pages per block %d",
		rawfs_sb->pages_per_block);

	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  sectors per page %d",
		rawfs_sb->sectors_per_page);
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mtd_init:  data storage per page %d",
		rawfs_sb->page_data_size);

	rawfs_sb->driver_context = mtd;

	rawfs_sb->dev.erase_block = rawfs_dev_mtd_erase_block;
	rawfs_sb->dev.read_page = rawfs_dev_mtd_read_page;
	rawfs_sb->dev.read_page_user = rawfs_dev_mtd_read_page_user;
	rawfs_sb->dev.write_page = rawfs_dev_mtd_write_page;

#ifdef MTK_NAND_PL_TEST
	mtk_nand_register_pl_test(0x480000 >> mtd->writesize_shift,
							  0x80000 >> mtd->writesize_shift, 0);
#endif

	return 0;
}

static int rawfs_fill_super(struct super_block *sb, void *data, int silent)
{
	int result;
	struct inode *inode;
	struct rawfs_sb_info *rawfs_sb;

	sb->s_magic = RAWFS_MAGIC;
	sb->s_op = &rawfs_super_ops;
	sb->s_time_gran = 1; /* 1 nanosecond time granularity */

	RAWFS_PRINT(RAWFS_DBG_INIT,
		"fill super, rawfs_file_info=%d, rawfs_page_header=%d\n",
		sizeof(struct rawfs_file_info),
		sizeof(struct rawfs_page));

	/* Allocate SB Info */
	sb->s_fs_info = kzalloc(sizeof(struct rawfs_sb_info), GFP_KERNEL);
	rawfs_sb = RAWFS_SB(sb);
	if (!rawfs_sb)
		return -ENOMEM;

	/* Parse mount options */
	rawfs_parse_mount_options(data, rawfs_sb);

	#ifdef RAWFS_BLOCK_FILE
	rawfs_sb->flags |= RAWFS_MNT_BLOCKFILE;
	#endif

	/* init inode hash tables */
	rawfs_hash_init(sb);

	/* init file list */
	rawfs_file_list_init(sb);

	/* read device information */

	/* init locks & mutex */
	mutex_init(&rawfs_sb->rawfs_lock);
	mutex_init(&rawfs_sb->file_list_lock);

	/* Mount on RAM */
#ifdef RAWFS_RAM_DISK
	if (rawfs_sb->flags & RAWFS_MNT_RAM) {
		rawfs_ram_init(data, rawfs_sb);
		rawfs_sb->fake_block = kzalloc(rawfs_sb->total_blocks *
			rawfs_sb->block_size, GFP_NOFS);

		/* rawfs_sb->fake_block = &rawfs_ram_disk[0]; */

		if (!rawfs_sb->fake_block)
			return -ENOMEM;
	}
#endif

	/* Mount on MTD */
	if (rawfs_sb->flags & RAWFS_MNT_MTD)
		rawfs_mtd_init(sb, data, silent);

	rawfs_dbg_sb_info(sb);

	sb->s_maxbytes = (rawfs_sb->pages_per_block-3) * rawfs_sb->page_data_size;
	sb->s_blocksize = rawfs_sb->page_size;
	sb->s_blocksize_bits = rawfs_calc_shifts(rawfs_sb->page_size);

	inode = new_inode(sb);

	if (!inode) {
		kfree(rawfs_sb);
		return -ENOMEM;
	}

	/* fill root inode: S_IFDIR, NULL, -1 */
	rawfs_fill_inode(inode, NULL, -1, -1, S_IFDIR | 0755, 0);

	sb->s_root = d_make_root(inode);
	if (!sb->s_root) {
		iput(inode);
		kfree(rawfs_sb);
		return -ENOMEM;
	}

	/* below not needed for many fs - but an example of per fs sb data */
	rawfs_sb->local_nls = load_nls_default();

	/* FS-FILLIN your filesystem specific mount logic/checks here */
	result = rawfs_block_level_analysis(sb);  /* Build block file inodes */
	if (result < 0)
		goto end;

	/* Adding block file */
	if (rawfs_sb->flags & RAWFS_MNT_BLOCKFILE) {
		struct rawfs_file_info fi;

		memset(&fi, 0, sizeof(struct rawfs_file_info));

		fi.i_atime = fi.i_mtime = fi.i_ctime = CURRENT_TIME_SEC;
		fi.i_uid = current_fsuid();
		fi.i_gid = current_fsgid(); /* current->fsgid; */
		fi.i_parent_folder_id = RAWFS_ROOT_DIR_ID;
		fi.i_mode = S_IFREG | S_IRUSR | S_IRGRP | S_IROTH;
		fi.i_id = RAWFS_BLOCK0_INO;
		strncpy(fi.i_name, ".block0", RAWFS_MAX_FILENAME_LEN+4);
		rawfs_file_list_add(sb, &fi, 0, -1);
		fi.i_id = RAWFS_BLOCK1_INO;
		strncpy(fi.i_name, ".block1", RAWFS_MAX_FILENAME_LEN+4);
		rawfs_file_list_add(sb, &fi, 1, -1);
	}

	result = rawfs_page_level_analysis(sb);  /* Build regular file inodes */
	if (result < 0)
		goto end;

	rawfs_file_level_analysis(sb);

	if (rawfs_context == NULL)
		rawfs_context_init();

	rawfs_context_add(sb); /* Add to fs context list */


	/* Process File */
	if (rawfs_proc_root)
		rawfs_sb->s_proc = proc_mkdir(sb->s_id, rawfs_proc_root);

	if (rawfs_sb->s_proc)
		proc_create_data("debug", S_IRUGO, rawfs_sb->s_proc,
			 &rawfs_seq_debug_fops, sb);

end:
	return result;
}


static struct dentry *rawfs_mount(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
	struct rawfs_sb_info peek_sb;

	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mount");

	/* peek mount options */
	rawfs_parse_mount_options(data, &peek_sb);

	if (peek_sb.flags & RAWFS_MNT_MTD)	{
		RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mount: mount on %s", dev_name);
		return mount_bdev(fs_type, flags, dev_name, data, rawfs_fill_super);
	} else {
		RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_mount: mount on RAM");
		return mount_nodev(fs_type, flags, data, rawfs_fill_super);
	}
}

static void rawfs_kill_super(struct super_block *sb)
{
	RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_kill_super: unmount\n");
	rawfs_file_list_destroy(sb);
	kill_litter_super(sb);
}

static struct file_system_type rawfs_fs_type = {
	.owner = THIS_MODULE,
	.name = "rawfs",
	.mount = rawfs_mount,
/* .get_sb = rawfs_get_sb, */
	.kill_sb = rawfs_kill_super,
	/*  .fs_flags */
};

/* ------------------------------------------------------------------------------ */
static int __init init_rawfs_fs(void)
{
	int err;
	RAWFS_PRINT(RAWFS_DBG_INIT, "Module Init\n");

#ifdef CONFIG_PROC_FS
	err = rawfs_proc_init();
#endif

	/* filesystem pass optional parms at load time */
	if (rawfs_parm > 1024) {
		RAWFS_PRINT(RAWFS_DBG_INIT, "rawfs_parm %d exceeds limit\n",
			rawfs_parm);
		rawfs_parm = 128;
	}
	err = rawfs_init_inodecache();

	if (err)
		goto out;

	err = register_filesystem(&rawfs_fs_type);

out:
	return err;
}

static void __exit exit_rawfs_fs(void)
{
	RAWFS_PRINT(RAWFS_DBG_INIT, "Module Exit\n");

#ifdef CONFIG_PROC_FS
	rawfs_proc_clean();
#endif

	rawfs_destroy_inodecache();
	unregister_filesystem(&rawfs_fs_type);
}

module_init(init_rawfs_fs)
module_exit(exit_rawfs_fs)

MODULE_AUTHOR("Perry Hsu <perry.hsu@mediatek.com>");
MODULE_DESCRIPTION("RAW file system for NAND flash");
MODULE_LICENSE("GPL");
