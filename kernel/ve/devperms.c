/*
 *  linux/kernel/ve/devperms.c
 *
 *  Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 * Devices permissions routines,
 * character and block devices separately
 *
 */

#include <linux/sched.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/list.h>
#include <linux/ve.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/rcupdate.h>
#include <linux/vzcalluser.h>
#include <linux/kdev_t.h>
#include <linux/major.h>

/*
 * Rules applied in the following order:
 *  MAJOR!=0, MINOR!=0
 *  MAJOR!=0, MINOR==0
 *  MAJOR==0, MINOR==0
 */

struct devperms_struct {
	dev_t   	dev;	/* device id */
	unsigned char	mask;
	unsigned 	type;
	envid_t	 	veid;

	struct hlist_node	hash;
	struct rcu_head		rcu;
};

static struct devperms_struct default_major_perms[] = {
	{
		MKDEV(UNIX98_PTY_MASTER_MAJOR, 0),
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(UNIX98_PTY_SLAVE_MAJOR, 0),
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(PTY_MASTER_MAJOR, 0),
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(PTY_SLAVE_MAJOR, 0),
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
};

static struct devperms_struct default_minor_perms[] = {
	{
		MKDEV(MEM_MAJOR, 3),	/* null */
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(MEM_MAJOR, 5),	/* zero */
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(MEM_MAJOR, 7),	/* full */
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(TTYAUX_MAJOR, 0),	/* tty */
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(TTYAUX_MAJOR, 2),	/* ptmx */
		S_IROTH | S_IWOTH,
		S_IFCHR,
	},
	{
		MKDEV(MEM_MAJOR, 8),	/* random */
		S_IROTH,
		S_IFCHR,
	},
	{
		MKDEV(MEM_MAJOR, 9),	/* urandom */
		S_IROTH,
		S_IFCHR
	},
};

static struct devperms_struct default_deny_perms = {
	MKDEV(0, 0),
	0,
	S_IFCHR,
};

static inline struct devperms_struct *find_default_devperms(int type, dev_t dev)
{
	int i;

	/* XXX all defaults perms are S_IFCHR */
	if (type != S_IFCHR)
		return &default_deny_perms;

	for (i = 0; i < ARRAY_SIZE(default_minor_perms); i++)
		if (MAJOR(dev) == MAJOR(default_minor_perms[i].dev) &&
				MINOR(dev) == MINOR(default_minor_perms[i].dev))
			return &default_minor_perms[i];

	for (i = 0; i < ARRAY_SIZE(default_major_perms); i++)
		if (MAJOR(dev) == MAJOR(default_major_perms[i].dev))
			return &default_major_perms[i];

	return &default_deny_perms;
}

#define DEVPERMS_HASH_SZ 512
#define devperms_hashfn(id, dev) \
	( (id << 5) ^ (id >> 5) ^ (MAJOR(dev)) ^ MINOR(dev) ) & \
						(DEVPERMS_HASH_SZ - 1)

static DEFINE_SPINLOCK(devperms_hash_lock);
static struct hlist_head devperms_hash[DEVPERMS_HASH_SZ];

static inline struct devperms_struct *find_devperms(envid_t veid,
						    int type,
						    dev_t dev)
{
	struct hlist_head *table;
	struct devperms_struct *perms;
	struct hlist_node *h;

	table = &devperms_hash[devperms_hashfn(veid, dev)];
	hlist_for_each_entry_rcu (perms, h, table, hash)
		if (perms->type == type && perms->veid == veid &&
				MAJOR(perms->dev) == MAJOR(dev) &&
				MINOR(perms->dev) == MINOR(dev))
			return perms;

	return NULL;
}

static void free_devperms(struct rcu_head *rcu)
{
	struct devperms_struct *perms;

	perms = container_of(rcu, struct devperms_struct, rcu);
	kfree(perms);
}

/* API calls */

void clean_device_perms_ve(envid_t veid)
{
	int i;
	struct devperms_struct *p;
	struct hlist_node *n, *tmp;

	spin_lock(&devperms_hash_lock);
	for (i = 0; i < DEVPERMS_HASH_SZ; i++)
		hlist_for_each_entry_safe (p, n, tmp, &devperms_hash[i], hash)
			if (p->veid == veid) {
				hlist_del_rcu(&p->hash);
				call_rcu(&p->rcu, free_devperms);
			}
	spin_unlock(&devperms_hash_lock);
}

EXPORT_SYMBOL(clean_device_perms_ve);

/*
 * Mode is a mask of
 *	FMODE_READ	for read access (configurable by S_IROTH)
 *	FMODE_WRITE	for write access (configurable by S_IWOTH)
 *	FMODE_QUOTACTL	for quotactl access (configurable by S_IXGRP)
 */

int get_device_perms_ve(int dev_type, dev_t dev, int access_mode)
{
	struct devperms_struct *p;
	struct ve_struct *ve;
	envid_t veid;
	char mask;

	ve = get_exec_env();
	veid = ve->veid;
	rcu_read_lock();

	p = find_devperms(veid, dev_type | VE_USE_MINOR, dev);
	if (p != NULL)
		goto end;

	p = find_devperms(veid, dev_type | VE_USE_MAJOR, MKDEV(MAJOR(dev),0));
	if (p != NULL)
		goto end;

	p = find_devperms(veid, dev_type, MKDEV(0,0));
	if (p != NULL)
		goto end;

	if (ve->features & VE_FEATURE_DEF_PERMS) {
		p = find_default_devperms(dev_type, dev);
		if (p != NULL)
			goto end;
	}

	rcu_read_unlock();
	return -ENODEV;

end:
	mask = p->mask;
	rcu_read_unlock();

	access_mode = "\000\004\002\006\010\014\012\016"[access_mode];
	return ((mask & access_mode) == access_mode) ? 0 : -EACCES;
}

EXPORT_SYMBOL(get_device_perms_ve);

int set_device_perms_ve(envid_t veid, unsigned type, dev_t dev, unsigned mask)
{
	struct devperms_struct *perms, *new_perms;
	struct hlist_head *htable;

	new_perms = kmalloc(sizeof(struct devperms_struct), GFP_KERNEL);

	spin_lock(&devperms_hash_lock);
	perms = find_devperms(veid, type, dev);
	if (perms != NULL) {
		kfree(new_perms);
		perms->mask = mask & S_IALLUGO;
	} else {
		switch (type & VE_USE_MASK) {
		case 0:
			dev = 0;
			break;
		case VE_USE_MAJOR:
			dev = MKDEV(MAJOR(dev),0);
			break;
		}

		new_perms->veid = veid;
		new_perms->dev = dev;
		new_perms->type = type;
		new_perms->mask = mask & S_IALLUGO;

		htable = &devperms_hash[devperms_hashfn(new_perms->veid,
				new_perms->dev)];
		hlist_add_head_rcu(&new_perms->hash, htable);
	}
	spin_unlock(&devperms_hash_lock);
	return 0;
}

EXPORT_SYMBOL(set_device_perms_ve);

#ifdef CONFIG_PROC_FS
static int devperms_seq_show(struct seq_file *m, void *v)
{
	struct devperms_struct *dp;
	char dev_s[32], type_c;
	unsigned use, type;
	dev_t dev;

	dp = (struct devperms_struct *)v;
	if (dp == (struct devperms_struct *)1L) {
		seq_printf(m, "Version: 2.7\n");
		return 0;
	}

	use = dp->type & VE_USE_MASK;
	type = dp->type & S_IFMT;
	dev = dp->dev;

	if ((use | VE_USE_MINOR) == use)
		snprintf(dev_s, sizeof(dev_s), "%d:%d", MAJOR(dev), MINOR(dev));
	else if ((use | VE_USE_MAJOR) == use)
		snprintf(dev_s, sizeof(dev_s), "%d:*", MAJOR(dp->dev));
	else
		snprintf(dev_s, sizeof(dev_s), "*:*");

	if (type == S_IFCHR)
		type_c = 'c';
	else if (type == S_IFBLK)
		type_c = 'b';
	else
		type_c = '?';

	seq_printf(m, "%10u %c %03o %s\n", dp->veid, type_c, dp->mask, dev_s);
	return 0;
}

static void *devperms_seq_start(struct seq_file *m, loff_t *pos)
{
	loff_t cpos;
	long slot;
	struct devperms_struct *dp;
	struct hlist_node *h;

	cpos = *pos;
	rcu_read_lock();

	if (cpos-- == 0)
		return (void *)1L;

	for (slot = 0; slot < DEVPERMS_HASH_SZ; slot++)
		hlist_for_each_entry_rcu (dp, h, &devperms_hash[slot], hash)
			if (cpos-- == 0) {
				m->private = (void *)slot;
				return dp;
			}
	return NULL;
}

static void *devperms_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	long slot;
	struct hlist_node *next;
	struct devperms_struct *dp;

	dp = (struct devperms_struct *)v;

	if (unlikely(dp == (struct devperms_struct *)1L))
		slot = 0;
	else {
		next = rcu_dereference(dp->hash.next);
		if (next != NULL)
			goto out;

		slot = (long)m->private + 1;
	}

	for (; slot < DEVPERMS_HASH_SZ; slot++) {
		next = rcu_dereference(devperms_hash[slot].first);
		if (next == NULL)
			continue;

		m->private = (void *)slot;
		goto out;
	}
	return NULL;

out:
	(*pos)++;
	return hlist_entry(next, struct devperms_struct, hash);
}

static void devperms_seq_stop(struct seq_file *m, void *v)
{
	rcu_read_unlock();
}

static struct seq_operations devperms_seq_op = {
	.start	= devperms_seq_start,
	.next	= devperms_seq_next,
	.stop	= devperms_seq_stop,
	.show	= devperms_seq_show,
};

static int devperms_open(struct inode *inode, struct file *file)
{
        return seq_open(file, &devperms_seq_op);
}

struct file_operations proc_devperms_ops = {
	.open		= devperms_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

EXPORT_SYMBOL(proc_devperms_ops);
#endif

/* Initialisation */

static struct devperms_struct original_perms[] =
{
	{
		MKDEV(0,0),
		S_IROTH | S_IWOTH,
		S_IFCHR,
		0,
	},
	{
		MKDEV(0,0),
		S_IXGRP | S_IROTH | S_IWOTH,
		S_IFBLK,
		0,
	},
};

static int __init init_devperms_hash(void)
{
	hlist_add_head(&original_perms[0].hash,
			&devperms_hash[devperms_hashfn(0,
				original_perms[0].dev)]);
	hlist_add_head(&original_perms[1].hash,
			&devperms_hash[devperms_hashfn(0,
				original_perms[1].dev)]);
	return 0;
}

core_initcall(init_devperms_hash);
