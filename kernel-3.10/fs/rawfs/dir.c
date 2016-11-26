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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/crc32.h>
#include "rawfs.h"

int rawfs_readdir(struct file *filp, void *dirent, filldir_t filldir)
{
	struct inode *inode = filp->f_path.dentry->d_inode;
	struct dentry *dentry = filp->f_path.dentry;
	struct super_block *sb = inode->i_sb;
	struct rawfs_sb_info *sbi = RAWFS_SB(sb);
	int total_cnt = 0;  /* Total entries we found in hlist */

	loff_t cpos;
	unsigned long ino;

	cpos = filp->f_pos;


	/* Read dir will execute twice. */
	if (inode->i_ino == RAWFS_ROOT_INO)
		RAWFS_PRINT(RAWFS_DBG_DIR, "rawfs_readdir, root, pos %lld\n", cpos);
	else
		RAWFS_PRINT(RAWFS_DBG_DIR, "rawfs_readdir, %s, i_id %X, i_ino %X, pos %lld\n",
		dentry->d_name.name, RAWFS_I(inode)->i_id, (unsigned)inode->i_ino, cpos);

	switch (cpos) {
	case 0:
		ino = dentry->d_inode->i_ino;
		if (filldir(dirent, ".", 1, cpos, ino, DT_DIR) < 0)
			break;
		filp->f_pos++;
		cpos++;
		/* fallthrough */
	case 1:
		ino = parent_ino(dentry);
		if (filldir(dirent, "..", 2, cpos, ino, DT_DIR) < 0)
			break;
		filp->f_pos++;
		cpos++;
		/* fallthrough */
	default:
		{
			struct rawfs_file_list_entry *entry;
			struct list_head *lists[2];
			int i;

			lists[0] =  &sbi->folder_list;
			lists[1] =  &sbi->file_list;

			mutex_lock(&sbi->file_list_lock);

			for (i = 0; i < 2; i++)	{
				list_for_each_entry(entry, lists[i], list)
				{
					 int name_len;

					 /* Matching sub-directory */
					 if (entry->file_info.i_parent_folder_id !=
						RAWFS_I(dentry->d_inode)->i_id) {
						RAWFS_PRINT(RAWFS_DBG_DIR,
							"readdir: skip %s, parent folder id %X, target folder %X\n",
						   entry->file_info.i_name,
						   entry->file_info.i_parent_folder_id,
						   RAWFS_I(dentry->d_inode)->i_id);
						continue;
					 }

					 total_cnt++;

					 if ((total_cnt+2) <= cpos) { /* skip first N + 2 (. & ..)
						entiries, if cpos doesn't start from zero */
						RAWFS_PRINT(RAWFS_DBG_DIR,
						"readdir: cpos=%lld, total cnt=%d, %s\n", cpos, total_cnt,
						entry->file_info.i_name);
						continue;
					 }

					 name_len = strlen(entry->file_info.i_name);

					 if (filldir(dirent,
						entry->file_info.i_name,
						name_len,
						cpos,
						entry->file_info.i_id,
						(S_ISDIR(entry->file_info.i_mode)?DT_DIR:DT_REG)) < 0)
						goto out;

					 filp->f_pos++;
					 cpos++;
				}
			}
		}

out:
			mutex_unlock(&sbi->file_list_lock);

			break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rawfs_readdir);

int rawfs_file_sync(struct file *file, loff_t start, loff_t end,
	int datasync)
{
	struct inode *inode = file->f_path.dentry->d_inode;

	RAWFS_PRINT(RAWFS_DBG_DIR, "fsync, i_ino %ld\n", inode->i_ino);
	return 0;
}
EXPORT_SYMBOL_GPL(rawfs_file_sync);

MODULE_AUTHOR("Perry Hsu <perry.hsu@mediatek.com>");
MODULE_DESCRIPTION("RAW file system for NAND flash");
MODULE_LICENSE("GPL");
