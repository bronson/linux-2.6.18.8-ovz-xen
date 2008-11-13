/*
 *  kernel/ub/ub_dcache.c
 *
 *  Copyright (C) 2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#include <linux/config.h>
#include <linux/dcache.h>
#include <linux/slab.h>
#include <linux/kmem_cache.h>
#include <linux/fs.h>
#include <linux/kmem_slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sysctl.h>
#include <linux/swap.h>
#include <linux/stop_machine.h>
#include <linux/cpumask.h>
#include <linux/nmi.h>
#include <linux/rwsem.h>
#include <linux/rcupdate.h>
#include <asm/bitops.h>

#include <ub/beancounter.h>
#include <ub/ub_mem.h>
#include <ub/ub_dcache.h>
#include <ub/ub_dcache_op.h>

/*
 * Locking
 *                          traverse  dcache_lock  d_lock
 *        ub_dentry_charge   +         -            +
 *      ub_dentry_uncharge   +         +            -
 * ub_dentry_charge_nofail   +         +            -
 *
 * d_inuse changes are atomic, with special handling of "not in use" <->
 * "in use" (-1 <-> 0) transitions.  We have two sources of non-atomicity
 * here: (1) in many operations we need to change d_inuse of both dentry and
 * its parent, and (2) on state transitions we need to adjust the account.
 *
 * Regarding (1): we do not have (and do not want) a single lock covering all
 * operations, so in general it's impossible to get a consistent view of
 * a tree with respect to d_inuse counters (except by swsuspend).  It also
 * means if a dentry with d_inuse of 0 gets one new in-use child and loses
 * one, it's d_inuse counter will go either 0 -> 1 -> 0 path or 0 -> -1 -> 0,
 * and we can't say which way.
 * Note that path -1 -> 0 -> -1 can't turn into -1 -> -2 -> -1, since
 * uncharge can be done only after return from charge (with d_genocide being
 * the only apparent exception).
 * Regarding (2): there is a similar uncertainty with the dcache account.
 * If the account is equal to the limit, one more dentry is started to be
 * used and one is put, the account will either hit the limit (and an error
 * will be returned), or decrement will happen before increment.
 *
 * These races do not really matter.
 * The only things we want are:
 *  - if a system is suspenede with no in-use dentries, all d_inuse counters
 *    should be correct (-1);
 *  - d_inuse counters should always be >= -1.
 * This holds if ->parent references are accessed and maintained properly.
 * In subtle moments (like d_move) dentries exchanging their parents should
 * both be in-use.  At d_genocide time, lookups and charges are assumed to be
 * impossible.
 */

/*
 * Hierarchical accounting
 * UB argument must NOT be NULL
 */

static int do_charge_dcache(struct user_beancounter *ub, unsigned long size, 
		enum ub_severity sv)
{
	unsigned long flags;

	spin_lock_irqsave(&ub->ub_lock, flags);
	if (__charge_beancounter_locked(ub, UB_KMEMSIZE, CHARGE_SIZE(size), sv))
		goto out_mem;
	if (__charge_beancounter_locked(ub, UB_DCACHESIZE, size, sv))
		goto out_dcache;
	spin_unlock_irqrestore(&ub->ub_lock, flags);
	return 0;

out_dcache:
	__uncharge_beancounter_locked(ub, UB_KMEMSIZE, CHARGE_SIZE(size));
out_mem:
	spin_unlock_irqrestore(&ub->ub_lock, flags);
	return -ENOMEM;
}

static void do_uncharge_dcache(struct user_beancounter *ub, 
		unsigned long size)
{
	unsigned long flags;

	spin_lock_irqsave(&ub->ub_lock, flags);
	__uncharge_beancounter_locked(ub, UB_KMEMSIZE, CHARGE_SIZE(size));
	__uncharge_beancounter_locked(ub, UB_DCACHESIZE, size);
	spin_unlock_irqrestore(&ub->ub_lock, flags);
}

static int charge_dcache(struct user_beancounter *ub, unsigned long size, 
		enum ub_severity sv)
{
	struct user_beancounter *p, *q;

	for (p = ub; p != NULL; p = p->parent) {
		if (do_charge_dcache(p, size, sv))
			goto unroll;
	}
	return 0;

unroll:
	for (q = ub; q != p; q = q->parent)
		do_uncharge_dcache(q, size);
	return -ENOMEM;
}

void uncharge_dcache(struct user_beancounter *ub, unsigned long size)
{
	for (; ub != NULL; ub = ub->parent)
		do_uncharge_dcache(ub, size);
}

/*
 * Simple helpers to do maintain account and d_ub field.
 */

static inline int d_charge(struct dentry_beancounter *d_bc)
{
	struct user_beancounter *ub;

	ub = get_beancounter(get_exec_ub());
	if (charge_dcache(ub, d_bc->d_ubsize, UB_SOFT)) {
		put_beancounter(ub);
		return -1;
	}
	d_bc->d_ub = ub;
	return 0;
}

static inline void d_forced_charge(struct dentry_beancounter *d_bc)
{
	struct user_beancounter *ub;

	ub = get_beancounter(get_exec_ub());
	charge_dcache(ub, d_bc->d_ubsize, UB_FORCE);
	d_bc->d_ub = ub;
}

/*
 * Minor helpers
 */

extern kmem_cache_t *dentry_cache; 
extern kmem_cache_t *inode_cachep;
static struct rw_semaphore ub_dentry_alloc_sem;

static inline unsigned int dentry_memusage(void)
{
	return dentry_cache->objuse;
}

static inline unsigned int inode_memusage(void)
{
	return inode_cachep->objuse;
}

static inline unsigned long d_charge_size(struct dentry *dentry)
{
	/* dentry's d_name is already set to appropriate value (see d_alloc) */
	return inode_cachep->objuse + dentry_cache->objuse +
		(dname_external(dentry) ?
		 kmem_obj_memusage((void *)dentry->d_name.name) : 0);
}

/*
 * Entry points from dcache.c
 */

/* 
 * Set initial d_inuse on d_alloc.
 * Called with no locks, preemption disabled.
 */
int __ub_dentry_alloc(struct dentry *dentry)
{
	struct dentry_beancounter *d_bc;

	d_bc = &dentry->dentry_bc;
	d_bc->d_ub = get_beancounter(get_exec_ub());
	atomic_set(&d_bc->d_inuse, INUSE_INIT); /* see comment in ub_dcache.h */
	d_bc->d_ubsize = d_charge_size(dentry);

	if (charge_dcache(d_bc->d_ub, d_bc->d_ubsize, UB_HARD))
		goto failure;
	return 0;

failure:
	put_beancounter(d_bc->d_ub);
	d_bc->d_ub = NULL;
	return -ENOMEM;
}
void __ub_dentry_alloc_start(void)
{
	down_read(&ub_dentry_alloc_sem);
	current->task_bc.dentry_alloc = 1;
}

void __ub_dentry_alloc_end(void)
{
	current->task_bc.dentry_alloc = 0;
	up_read(&ub_dentry_alloc_sem);
}

/*
 * It is assumed that parent is already in use, so traverse upwards is
 * limited to one ancestor only.
 * Called under d_lock and rcu_read_lock.
 */
int __ub_dentry_charge(struct dentry *dentry)
{
	struct dentry_beancounter *d_bc;
	struct dentry *parent;
	int ret;

	if (ub_dget_testone(dentry)) {
		d_bc = &dentry->dentry_bc;
		/* state transition -1 => 0 */
		if (d_charge(d_bc))
			goto failure;

		if (dentry != dentry->d_parent) {
			parent = dentry->d_parent;
			if (ub_dget_testone(parent))
				BUG();
		}
	}
	return 0;

failure:
	/*
	 * Here we would like to fail the lookup.
	 * It is not easy: if d_lookup fails, callers expect that a dentry
	 * with the given name doesn't exist, and create a new one.
	 * So, first we forcedly charge for this dentry.
	 * Then try to remove it from cache safely.  If it turns out to be
	 * possible, we can return error.
	 */
	d_forced_charge(d_bc);

	if (dentry != dentry->d_parent) {
		parent = dentry->d_parent;
		if (ub_dget_testone(parent))
			BUG();
	}

	ret = 0;
	if (spin_trylock(&dcache_lock)) {
		if (!list_empty(&dentry->d_subdirs)) {
			spin_unlock(&dentry->d_lock);
			spin_unlock(&dcache_lock);
			rcu_read_unlock();
			shrink_dcache_parent(dentry);
			rcu_read_lock();
			spin_lock(&dcache_lock);
			spin_lock(&dentry->d_lock);
		}
		if (atomic_read(&dentry->d_count) == 1) {
			__d_drop(dentry);
			ret = -1;
		}
		spin_unlock(&dcache_lock);
	}

	return ret;
}

/*
 * Go up in the tree decreasing d_inuse.
 * Called under dcache_lock.
 */
void __ub_dentry_uncharge(struct dentry *dentry)
{
	struct dentry *parent;
	struct user_beancounter *ub;
	unsigned long size;

	/* go up until state doesn't change or and root is reached */
	size = dentry->dentry_bc.d_ubsize;
	ub = dentry->dentry_bc.d_ub;
	while (ub_dput_testzero(dentry)) {
		/* state transition 0 => -1 */
		uncharge_dcache(ub, size);
		put_beancounter(ub);

		parent = dentry->d_parent;
		if (dentry == parent)
			break;

		dentry = parent;
		size = dentry->dentry_bc.d_ubsize;
		ub = dentry->dentry_bc.d_ub;
	}
}

/* 
 * Forced charge for __dget_locked, where API doesn't allow to return error.
 * Called under dcache_lock.
 */
void __ub_dentry_charge_nofail(struct dentry *dentry)
{
	struct dentry *parent;

	while (ub_dget_testone(dentry)) {
		/* state transition -1 => 0 */
		d_forced_charge(&dentry->dentry_bc);

		parent = dentry->d_parent;
		if (dentry == parent)
			break;
		dentry = parent;
	}
}

/*
 * Adaptive accounting
 */

int ub_dentry_on;
int ub_dentry_alloc_barrier;
EXPORT_SYMBOL(ub_dentry_on);

static DEFINE_PER_CPU(int, checkcnt);
static unsigned long checklowat = 0;
static unsigned long checkhiwat = ULONG_MAX;

static int sysctl_ub_dentry_chk = 10;
#define sysctl_ub_lowat	sysctl_ub_watermark[0]
#define sysctl_ub_hiwat sysctl_ub_watermark[1]
static DECLARE_RWSEM(ub_dentry_alloc_sem);
/* 1024th of lowmem size */
static unsigned int sysctl_ub_watermark[2] = {0, 100};


static int ub_dentry_acctinit(struct dentry *dentry)
{
	struct dentry_beancounter *d_bc;

	d_bc = &dentry->dentry_bc;
	d_bc->d_ub = NULL;
	atomic_set(&d_bc->d_inuse, -1);
	if (dname_external(dentry)) {
		struct page *page;
		page = virt_to_page(dentry->d_name.name);
		if (!PageSlab(page) || page_get_cache(page) == NULL) {
			printk("Problem with name, dentry %p, parent %p, "
					"name %p len %d\n",
					dentry, dentry->d_parent,
					dentry->d_name.name,
					dentry->d_name.len);
			printk("   de %p name %.10s\n",
					dentry, dentry->d_name.name);
			d_bc->d_ubsize = 0;
			return 0;
		}
	}
	d_bc->d_ubsize = d_charge_size(dentry);
	return 0;
}

static int ub_dentry_acctcount(struct dentry *dentry)
{
	struct dentry_beancounter *d_bc;
	struct dentry *child;
	int count;

	count = 0;
	list_for_each_entry(child, &dentry->d_subdirs, d_u.d_child)
		count++;

	d_bc = &dentry->dentry_bc;
	count = atomic_read(&dentry->d_count) - count;
	if (count) {
		__ub_dentry_charge_nofail(dentry);
		if (count > 1)
			atomic_add(count - 1, &d_bc->d_inuse);
	}

	return 0;
}

static int ub_dentry_acctdrop(struct dentry *dentry)
{
	struct dentry_beancounter *d_bc;

	d_bc = &dentry->dentry_bc;
	if (atomic_read(&d_bc->d_inuse) < 0)
		return 0;
	atomic_set(&d_bc->d_inuse, -1);
	uncharge_dcache(d_bc->d_ub, d_bc->d_ubsize);
	put_beancounter(d_bc->d_ub);
	return 0;
}

extern void kmem_cache_free_block(kmem_cache_t *cachep,
		struct kmem_list3 *l3, void **objpp,
		int nr_objects, int node);

static int ub_dentry_walk_node(int (*fun)(struct dentry *), int node)
{
	kmem_cache_t *cachep;
	struct array_cache *ac;
	struct slab *slabp;
	char *objp;
	int cpu, i, sz, r, n;
	struct kmem_list3 *l3;
	unsigned long map[PAGE_SIZE / sizeof(struct dentry)
					/ BITS_PER_LONG + 1];

	cachep = dentry_cache;
	if (cachep->num >= sizeof(map) * 8)
		return -E2BIG;

	l3 = cachep->nodelists[node];
	/* drain all CPU caches to have up-to-date free map */

#ifdef CONFIG_NUMA
	/* walk through all nodes and drain alien caches */
	for_each_online_node (n) {
		if (!cachep->nodelists[n]->alien)
			continue;
		ac = cachep->nodelists[n]->alien[node];
		if (!ac)
			continue;
		kmem_cache_free_block(cachep, cachep->nodelists[node],
				ac->entry, ac->avail, node);
		ac->avail = 0;
	}
#endif

	ac = l3->shared;
	kmem_cache_free_block(cachep, l3, ac->entry, ac->avail, node);
	ac->avail = 0;
	for_each_online_cpu(cpu) {
		ac = cachep->array[cpu];
		n = cpu_to_node(cpu);
		kmem_cache_free_block(cachep, cachep->nodelists[n],
				ac->entry, ac->avail, n);
		ac->avail = 0;
	}

	list_for_each_entry(slabp, &l3->slabs_full, list) {
		touch_nmi_watchdog();
		for (i = 0, objp = slabp->s_mem;
		     i < cachep->num;
		     i++, objp += cachep->buffer_size) {
#if SLAB_DEBUG
			r = (*fun)((struct dentry *)
					(objp + cachep->obj_offset));
#else
			r = (*fun)((struct dentry *)objp);
#endif
			if (r)
				return r;
		}
	}

	list_for_each_entry(slabp, &l3->slabs_partial, list) {
		touch_nmi_watchdog();
		memset(map, 0xff, sizeof(map));
		for (i = slabp->free, r = 0;
		     i != BUFCTL_END;
		     i = slab_bufctl(slabp)[i], r++) {
			if (r > cachep->num)
				return -1;
			__clear_bit(i, map);
		}
		sz = sizeof(map) * BITS_PER_LONG;
		for (i = find_first_bit(map, sz);
		     i < cachep->num;
		     i = find_next_bit(map, sz, i + 1)) {
			objp = slabp->s_mem + i * cachep->buffer_size;
#if SLAB_DEBUG
			r = (*fun)((struct dentry *)
					(objp + cachep->obj_offset));
#else
			r = (*fun)((struct dentry *)objp);
#endif
			if (r)
				return r;
		}
	}

	return 0;
}

static int ub_dentry_walk(int (*fun)(struct dentry *))
{
	int node;
	int err;

	for_each_online_node (node) {
		if ((err = ub_dentry_walk_node(fun, node)) != 0)
			return err;
	}
	return 0;
}

static int ub_dentry_accton(void *data)
{
	struct user_beancounter *ub;
	int err;

	ub = get_exec_ub();
	set_exec_ub(get_ub0());
	err = ub_dentry_walk(&ub_dentry_acctinit);
	if (!err)
		err = ub_dentry_walk(&ub_dentry_acctcount);
	set_exec_ub(ub);
	if (err == 0)
		ub_dentry_on = 1;
	return err;
}

static int ub_dentry_acctoff(void *data)
{
	int ret;
	ret = ub_dentry_walk(&ub_dentry_acctdrop);
	if (ret == 0)
		ub_dentry_on = 0;
	return ret;
}

/*
 * Main function turning dcache accounting on and off.
 * Called with preemption disabled (for caller's convenience).
 */
static void ub_dentry_switch(int onoff, unsigned long pages, int (*fun)(void *))
{
	static char *s[] = { "off", "on" };
	unsigned long start_jiffies;
	int err, tm;

	start_jiffies = jiffies;
	preempt_enable();
	ub_dentry_alloc_barrier = 1;
	/* ensure ub_dentry_alloc_barrier is visible on all CPUs */
	mb();
	synchronize_rcu();
	down_write(&ub_dentry_alloc_sem);
	if (ub_dentry_on == onoff)
		goto done;

	printk("UBC: preparing to turn dcache accounting %s, "
			"size %lu pages, watermarks %lu %lu\n",
			s[onoff], pages, checklowat, checkhiwat);
	err = stop_machine_run(fun, NULL, NR_CPUS);
	if (err) {
		printk(KERN_ERR "UBC: ERROR: dcache accounting switch %d\n",
				err);
		preempt_disable();
		checklowat = 0;
		checkhiwat = ULONG_MAX;
		sysctl_ub_dentry_chk = INT_MAX;
		preempt_enable();
	} else {
		tm = jiffies_to_msecs(jiffies - start_jiffies);
		printk("UBC: turning dcache accounting %s succeeded, "
				"usage %lu, time %u.%03u\n",
				s[onoff],
				get_ub0()->ub_parms[UB_DCACHESIZE].held,
				tm / 1000, tm % 1000);
	}

done:
	ub_dentry_alloc_barrier = 0;
	up_write(&ub_dentry_alloc_sem);
	preempt_disable();
}

void ub_dentry_checkup(void)
{
	int *p;
	unsigned long pages;

	preempt_disable();
	p = &__get_cpu_var(checkcnt);
	if (++*p > sysctl_ub_dentry_chk) {
		*p = 0;
		pages = dentry_cache->grown
			- dentry_cache->reaped
			- dentry_cache->shrunk;
		pages <<= dentry_cache->gfporder;
		if (ub_dentry_on) {
			if (pages < checklowat)
				ub_dentry_switch(0, pages, &ub_dentry_acctoff);
		} else {
			if (pages >= checkhiwat)
				ub_dentry_switch(1, pages, &ub_dentry_accton);
		}
	}
	preempt_enable();
}

static void ub_dentry_set_limits(unsigned long pages, unsigned long cap)
{
	down_write(&ub_dentry_alloc_sem);
	preempt_disable();
	checklowat = (pages >> 10) * sysctl_ub_lowat;
	checkhiwat = (pages >> 10) * sysctl_ub_hiwat;
	if (checkhiwat > cap) {
		checkhiwat = cap;
		checklowat = cap / sysctl_ub_hiwat * sysctl_ub_lowat;
	}
	preempt_enable();
	up_write(&ub_dentry_alloc_sem);
}

static int ub_dentry_proc_handler(ctl_table *ctl, int write, struct file *filp,
			  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int r;

	r = proc_dointvec(ctl, write, filp, buffer, lenp, ppos);
	if (!r && write)
		ub_dentry_set_limits(totalram_pages - totalhigh_pages,
				ULONG_MAX);
	return r;
}

static ctl_table ub_dentry_sysctl_table[] = {
	{
		.ctl_name	= 1000,
		.procname	= "dentry_check",
		.data		= &sysctl_ub_dentry_chk,
		.maxlen		= sizeof(sysctl_ub_dentry_chk),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= 1001,
		.procname	= "dentry_watermark",
		.data		= &sysctl_ub_lowat,
		.maxlen		= sizeof(sysctl_ub_lowat) * 2,
		.mode		= 0644,
		.proc_handler	= &ub_dentry_proc_handler,
	},
	{ .ctl_name = 0 }
};
static ctl_table ub_dentry_sysctl_root[] = {
	{
		.ctl_name	= 23681,
		.procname	= "ubc",
		.mode		= 0555,
		.child		= ub_dentry_sysctl_table,
	},
	{ .ctl_name = 0 }
};

static int __init ub_dentry_init(void)
{
	/*
	 * Initial watermarks are limited, to limit walk time.
	 * 384MB translates into 0.8 sec on PIII 866MHz.
	 */
	ub_dentry_set_limits(totalram_pages - totalhigh_pages,
			384 * 1024 * 1024 / PAGE_SIZE);
	if (register_sysctl_table(ub_dentry_sysctl_root, 0) == NULL)
		return -ENOMEM;
	return 0;
}
__initcall(ub_dentry_init);
