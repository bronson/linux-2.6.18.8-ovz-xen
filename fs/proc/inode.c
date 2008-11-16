/*
 *  linux/fs/proc/inode.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/file.h>
#include <linux/limits.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/smp_lock.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include "internal.h"

/*
 * Decrements the use count and checks for deferred deletion.
 */
void de_put(struct proc_dir_entry *de)
{
	if (de) {
		if (unlikely(!atomic_read(&de->count)))
			goto out_bad;

		if (atomic_dec_and_test(&de->count)) {
			if (unlikely(!de->deleted))
				goto out_bad;

			free_proc_entry(de);
		}
	}
	return;

out_bad:
	printk("de_put: bad dentry %s count:%d deleted:%d\n",
			de->name, atomic_read(&de->count), de->deleted);
}

/*
 * Decrement the use count of the proc_dir_entry.
 */
static void proc_delete_inode(struct inode *inode)
{
	struct proc_dir_entry *de;

	truncate_inode_pages(&inode->i_data, 0);

	/* Stop tracking associated processes */
	put_pid(PROC_I(inode)->pid);

	/* Let go of any associated proc directory entry */
	de = LPDE(inode);
	if (de) {
		if (de->owner)
			module_put(de->owner);
		de_put(de);
	}
#ifdef CONFIG_VE
	de = GPDE(inode);
	if (de) {
		module_put(de->owner);
		de_put(de);
	}
#endif
	clear_inode(inode);
}

struct vfsmount *proc_mnt;

static void proc_read_inode(struct inode * inode)
{
	inode->i_mtime = inode->i_atime = inode->i_ctime = CURRENT_TIME;
}

static kmem_cache_t * proc_inode_cachep;

static struct inode *proc_alloc_inode(struct super_block *sb)
{
	struct proc_inode *ei;
	struct inode *inode;

	ei = (struct proc_inode *)kmem_cache_alloc(proc_inode_cachep, SLAB_KERNEL);
	if (!ei)
		return NULL;
	ei->pid = NULL;
	ei->fd = 0;
	ei->op.proc_get_link = NULL;
	ei->pde = NULL;
	inode = &ei->vfs_inode;
	inode->i_mtime = inode->i_atime = inode->i_ctime = CURRENT_TIME;
#ifdef CONFIG_VE
	GPDE(inode) = NULL;
#endif
	return inode;
}

static void proc_destroy_inode(struct inode *inode)
{
	kmem_cache_free(proc_inode_cachep, PROC_I(inode));
}

static void init_once(void * foo, kmem_cache_t * cachep, unsigned long flags)
{
	struct proc_inode *ei = (struct proc_inode *) foo;

	if ((flags & (SLAB_CTOR_VERIFY|SLAB_CTOR_CONSTRUCTOR)) ==
	    SLAB_CTOR_CONSTRUCTOR)
		inode_init_once(&ei->vfs_inode);
}
 
int __init proc_init_inodecache(void)
{
	proc_inode_cachep = kmem_cache_create("proc_inode_cache",
					     sizeof(struct proc_inode),
					     0, (SLAB_RECLAIM_ACCOUNT|
						SLAB_MEM_SPREAD),
					     init_once, NULL);
	if (proc_inode_cachep == NULL)
		return -ENOMEM;
	return 0;
}

static int proc_remount(struct super_block *sb, int *flags, char *data)
{
	*flags |= MS_NODIRATIME;
	return 0;
}

static struct super_operations proc_sops = { 
	.alloc_inode	= proc_alloc_inode,
	.destroy_inode	= proc_destroy_inode,
	.read_inode	= proc_read_inode,
	.drop_inode	= generic_delete_inode,
	.delete_inode	= proc_delete_inode,
	.statfs		= simple_statfs,
	.remount_fs	= proc_remount,
};

struct inode *proc_get_inode(struct super_block *sb, unsigned int ino,
				struct proc_dir_entry *de)
{
	struct inode * inode;

	/*
	 * Increment the use count so the dir entry can't disappear.
	 */
	de_get(de);

	inode = iget(sb, ino);
	if (!inode)
		goto out_mod;

	PROC_I(inode)->pde = de;
	if (de) {
		if (de->mode) {
			inode->i_mode = de->mode;
			inode->i_uid = de->uid;
			inode->i_gid = de->gid;
		}
		if (de->size)
			inode->i_size = de->size;
		if (de->nlink)
			inode->i_nlink = de->nlink;
		if (de->proc_iops)
			inode->i_op = de->proc_iops;
		if (de->proc_fops)
			inode->i_fop = de->proc_fops;
	}

	return inode;

out_mod:
	de_put(de);
	return NULL;
}			

int proc_fill_super(struct super_block *s, void *data, int silent)
{
	struct inode * root_inode;

	s->s_flags |= MS_NODIRATIME | MS_NOSUID | MS_NOEXEC;
	s->s_blocksize = 1024;
	s->s_blocksize_bits = 10;
	s->s_magic = PROC_SUPER_MAGIC;
	s->s_op = &proc_sops;
	s->s_time_gran = 1;

	/* proc_root.owner == NULL, just a formal call */
	__module_get(proc_root.owner);
	root_inode = proc_get_inode(s, PROC_ROOT_INO, &proc_root);
	if (!root_inode)
		goto out_no_root;
	root_inode->i_uid = 0;
	root_inode->i_gid = 0;
	s->s_root = d_alloc_root(root_inode);
	if (!s->s_root)
		goto out_no_root;
#ifdef CONFIG_VE
	LPDE(root_inode) = de_get(get_exec_env()->proc_root);
	GPDE(root_inode) = &proc_root;
#else
	LPDE(root_inode) = &proc_root;
#endif
	return 0;

out_no_root:
	printk("proc_read_super: get root inode failed\n");
	iput(root_inode);
	return -ENOMEM;
}
MODULE_LICENSE("GPL");
