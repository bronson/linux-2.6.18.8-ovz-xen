/*
 * Fair Scheduler
 *
 * Copyright (C) 2000-2005  SWsoft
 * All rights reserved.
 * 
 * Licensing governed by "linux/COPYING.SWsoft" file.
 *
 * Start-tag scheduling follows the theory presented in
 * http://www.cs.utexas.edu/users/dmcl/papers/ps/SIGCOMM96.ps
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <xen/interface/arch-x86/xen.h>
#include <asm/timex.h>
#include <asm/atomic.h>
#include <linux/spinlock.h>
#include <asm/semaphore.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/dcache.h>
#include <linux/sysctl.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/console.h>
#include <linux/fairsched.h>
#include <linux/vsched.h>

/* we need it for vsched routines in sched.c */
spinlock_t fairsched_lock = SPIN_LOCK_UNLOCKED;

#ifdef CONFIG_FAIRSCHED

#define FAIRSHED_DEBUG		" debug"


/*********************************************************************/
/*
 * Special arithmetics
 */
/*********************************************************************/

#define CYCLES_SHIFT (8)
#define SCYCLES_TIME(time) \
        ((scycles_t) {((time) + (1 << CYCLES_SHIFT) - 1)  >> CYCLES_SHIFT})

#define CYCLES_ZERO (0)
static inline int CYCLES_BEFORE(cycles_t x, cycles_t y)
{
        return (__s64)(x-y) < 0;
}
static inline int CYCLES_AFTER(cycles_t x, cycles_t y)
{
        return (__s64)(y-x) < 0;
}
static inline void CYCLES_DADD(cycles_t *x, fschdur_t y) {*x+=y.d;}

/*
 * fairsched_schedule() can be called rarely than on each timer tick 
 * due to main scheduler optimizations, so new abstract timeslice must
 * be introduced. It can have arbitrary number ot cycles, but main
 * scheduler mustn't exceed this value and call fairsched scheduler
 * before this timeslice is expired on a node.
 */
static cycles_t cycles_per_timeslice;
#define FSCHDUR_ZERO (0)
#define TICK_DUR ((fschdur_t){cycles_per_timeslice})
static inline fschdur_t FSCHDURATION(cycles_t x, cycles_t y)
{
	return (fschdur_t){x - y};
}
static inline int FSCHDUR_CMP(fschdur_t x, fschdur_t y)
{
	if (x.d < y.d) return -1;
	if (x.d > y.d) return 1;
	return 0;
}
static inline fschdur_t FSCHDUR_SUB(fschdur_t x, fschdur_t y)
{
	return (fschdur_t){x.d - y.d};
}

#define FSCHTAG_ZERO ((fschtag_t){0})
static inline int FSCHTAG_CMP(fschtag_t x, fschtag_t y)
{
	if (x.t < y.t) return -1;
	if (x.t > y.t) return 1;
	return 0;
}
static inline fschtag_t FSCHTAG_MAX(fschtag_t x, fschtag_t y)
{
	return x.t >= y.t ? x : y;
}
static inline int FSCHTAG_DADD(fschtag_t *tag, fschdur_t dur, unsigned w)
{
	cycles_t new_tag;
	new_tag = tag->t + (cycles_t)dur.d * w;
	if (new_tag < tag->t)
		return -1;
	/* DEBUG */
	if (new_tag >= (1ULL << 48))
		return -1;
	tag->t = new_tag;
	return 0;
}
static inline int FSCHTAG_ADD(fschtag_t *tag, fschtag_t y)
{
	cycles_t new_tag;
	new_tag = tag->t + y.t;
	if (new_tag < tag->t)
		return -1;
	tag->t = new_tag;
	return 0;
}
static inline fschtag_t FSCHTAG_SUB(fschtag_t x, fschtag_t y)
{
	return (fschtag_t){x.t - y.t};
}

#define FSCHVALUE_FMT "%Lu"
#define FSCHVALUE_PRINT(x) ((x).v)
#define FSCHVALUE_ZERO ((fschvalue_t){0})
#define TICK_VALUE ((fschvalue_t)	\
	{(cycles_t)cycles_per_timeslice << FSCHRATE_SHIFT})
static inline fschvalue_t FSCHVALUE(unsigned long t)
{
	return (fschvalue_t){(cycles_t)t << FSCHRATE_SHIFT};
}
static inline int FSCHVALUE_CMP(fschvalue_t x, fschvalue_t y)
{
	if (x.v < y.v) return -1;
	if (x.v > y.v) return 1;
	return 0;
}
static inline void FSCHVALUE_DADD(fschvalue_t *val, fschdur_t dur,
		unsigned rate)
{
	val->v += (cycles_t)dur.d * rate;
}
static inline fschvalue_t FSCHVALUE_SUB(fschvalue_t x, fschvalue_t y)
{
	return (fschvalue_t){x.v - y.v};
}
static inline cycles_t FSCHVALUE_TO_DELAY(fschvalue_t val, unsigned rate)
{
	unsigned long t;
	/*
	 * Here we lose precision to make the division 32-bit on IA-32.
	 * The value is not greater than TICK_VALUE.
	 * (TICK_VALUE >> FSCHRATE_SHIFT) fits unsigned long.
	 */
	t = (val.v + (1 << FSCHRATE_SHIFT) - 1) >> FSCHRATE_SHIFT;
	return (cycles_t)((t + rate - 1) / rate) << FSCHRATE_SHIFT;
}


/*********************************************************************/
/*
 * Global data
 */
/*********************************************************************/

/*
 * Assertions.
 * Called with preemption disabled.
 */

#define fsch_assert(x)							\
	do {								\
		static int count;					\
		if (x)							\
			break;						\
		if (count++ > 10)					\
			break;						\
		__printk_no_wake++;					\
		printk("fsch_assert " #x " failed\n");			\
		__printk_no_wake--;					\
	} while (0)

#define fsch_validate(x, fmt...)					\
	do {								\
		static int count;					\
		if (x)							\
			break;						\
		if (count++ > 10)					\
			break;						\
		__printk_no_wake++;					\
		printk("fsch_assert " #x " failed\n");			\
		printk("fsch_assert: " fmt);				\
		__printk_no_wake--;					\
	} while (0)

/*
 * Configurable parameters
 */
unsigned fairsched_max_latency = 25; /* jiffies */

/*
 * Parameters initialized at startup
 */
/* Number of online CPUs */
unsigned fairsched_nr_cpus;
/* Token Bucket depth (burst size) */
static fschvalue_t max_value;

struct fairsched_node fairsched_init_node = {
	.id		= INT_MAX,
#ifdef CONFIG_VE
	.owner_env	= get_ve0(),
#endif
	.weight		= 1,
};
EXPORT_SYMBOL(fairsched_init_node);

struct fairsched_node fairsched_idle_node = {
	.id =			-1,
};

static int fairsched_nr_nodes;
static LIST_HEAD(fairsched_node_head);
static LIST_HEAD(fairsched_running_head);
static LIST_HEAD(fairsched_delayed_head);

DEFINE_PER_CPU(cycles_t, prev_schedule);
static fschtag_t max_latency;

static DEFINE_MUTEX(fairsched_mutex);

/*********************************************************************/
/*
 * Small helper routines
 */
/*********************************************************************/

/* this didn't proved to be very valuable statistics... */
#define fairsched_inc_ve_strv(node, cycles)  do {} while(0)
#define fairsched_dec_ve_strv(node, cycles)  do {} while(0)

/*********************************************************************/
/*
 * Runlist management
 */
/*********************************************************************/

/*
 * Returns the start_tag of the first runnable node, or 0.
 */
static inline fschtag_t virtual_time(void)
{
	struct fairsched_node *p;

	if (!list_empty(&fairsched_running_head)) {
		p = list_first_entry(&fairsched_running_head,
				struct fairsched_node, runlist);
		return p->start_tag;
	}
	return FSCHTAG_ZERO;
}

static void fairsched_recompute_max_latency(void)
{
	struct fairsched_node *p;
	unsigned w;
	fschtag_t tag;

	w = FSCHWEIGHT_MAX;
	for_each_fairsched_node(p) {
		if (p->weight < w)
			w = p->weight;
	}
	tag = FSCHTAG_ZERO;
	(void) FSCHTAG_DADD(&tag, TICK_DUR,
				fairsched_nr_cpus * fairsched_max_latency * w);
	max_latency = tag;
}

static void fairsched_reset_start_tags(void)
{
	struct fairsched_node *cnode;
	fschtag_t min_tag;

	min_tag = virtual_time();
	for_each_fairsched_node(cnode) {
		if (FSCHTAG_CMP(cnode->start_tag, min_tag) > 0)
			cnode->start_tag = FSCHTAG_SUB(cnode->start_tag,
						       min_tag);
		else
			cnode->start_tag = FSCHTAG_ZERO;
	}
}

static void fairsched_running_insert(struct fairsched_node *node)
{
	struct list_head *tmp;
	struct fairsched_node *p;
	fschtag_t start_tag_max;

	if (!list_empty(&fairsched_running_head)) {
		start_tag_max = virtual_time();
		if (!FSCHTAG_ADD(&start_tag_max, max_latency) &&
		    FSCHTAG_CMP(start_tag_max, node->start_tag) < 0)
			node->start_tag = start_tag_max;
	}

	list_for_each(tmp, &fairsched_running_head) {
		p = list_entry(tmp, struct fairsched_node, runlist);
		if (FSCHTAG_CMP(node->start_tag, p->start_tag) <= 0)
			break;
	}
	/* insert node just before tmp */
	list_add_tail(&node->runlist, tmp);
}

static inline void fairsched_running_insert_fromsleep(
		struct fairsched_node *node)
{
	node->start_tag = FSCHTAG_MAX(node->start_tag, virtual_time());
	fairsched_running_insert(node);
}


/*********************************************************************/
/*
 * CPU limiting helper functions
 *
 * These functions compute rates, delays and manipulate with sleep
 * lists and so on.
 */
/*********************************************************************/

/*
 * Insert a node into the list of nodes removed from scheduling,
 * sorted by the time at which the the node is allowed to run,
 * historically called `delay'.
 */
static void fairsched_delayed_insert(struct fairsched_node *node)
{
	struct fairsched_node *p;
	struct list_head *tmp;

	list_for_each(tmp, &fairsched_delayed_head) {
		p = list_entry(tmp, struct fairsched_node,
				   runlist);
		if (CYCLES_AFTER(p->delay, node->delay))
			break;
	}
        /* insert node just before tmp */
	list_add_tail(&node->runlist, tmp);
}

static inline void nodevalue_add(struct fairsched_node *node,
		fschdur_t duration, unsigned rate)
{
	FSCHVALUE_DADD(&node->value, duration, rate);
	if (FSCHVALUE_CMP(node->value, max_value) > 0)
		node->value = max_value;
}

/*
 * The node has been selected to run.
 * This function accounts in advance for the time that the node will run.
 * The advance not used by the node will be credited back.
 */
static void fairsched_ratelimit_charge_advance(
		struct fairsched_node *node,
		cycles_t time)
{
	fsch_assert(!node->delayed);
	fsch_validate(FSCHVALUE_CMP(node->value, TICK_VALUE) >= 0,
			"charge, value " FSCHVALUE_FMT
			", tick " FSCHVALUE_FMT
			", delay %Lu, time %Lu"
			", lastupd %Lu, rate %u\n",
			FSCHVALUE_PRINT(node->value),
			FSCHVALUE_PRINT(TICK_VALUE),
			node->delay, time,
			node->last_updated_at, node->rate);

	/*
	 * Account for the time passed since last update.
	 * It might be needed if the node has become runnable because of
	 * a wakeup, but hasn't gone through other functions updating
	 * the bucket value.
	 */
	if (CYCLES_AFTER(time, node->last_updated_at)) {
		nodevalue_add(node, FSCHDURATION(time, node->last_updated_at),
			      node->rate);
		node->last_updated_at = time;
	}

	/* charge for the full tick the node might be running */
	node->value = FSCHVALUE_SUB(node->value, TICK_VALUE);
	if (FSCHVALUE_CMP(node->value, TICK_VALUE) < 0) {
		list_del(&node->runlist);
		node->delayed = 1;
		node->delay = node->last_updated_at + FSCHVALUE_TO_DELAY(
					FSCHVALUE_SUB(TICK_VALUE, node->value),
					node->rate);
		node->nr_ready = 0;
		fairsched_delayed_insert(node);
	}
}

static void fairsched_ratelimit_credit_unused(
		struct fairsched_node *node,
		cycles_t time, fschdur_t duration)
{
	/* account for the time passed since last update */
	if (CYCLES_AFTER(time, node->last_updated_at)) {
		nodevalue_add(node, FSCHDURATION(time, node->last_updated_at),
			      node->rate);
		node->last_updated_at = time;
	}

	/*
	 * When the node was given this CPU, it was charged for 1 tick.
	 * Credit back the unused time.
	 */
	if (FSCHDUR_CMP(duration, TICK_DUR) < 0)
		nodevalue_add(node, FSCHDUR_SUB(TICK_DUR, duration),
			      1 << FSCHRATE_SHIFT);

	/* check if the node is allowed to run */
	if (FSCHVALUE_CMP(node->value, TICK_VALUE) < 0) {
		/*
		 * The node was delayed and remain such.
		 * But since the bucket value has been updated,
		 * update the delay time and move the node in the list.
		 */
		fsch_assert(node->delayed);
		node->delay = node->last_updated_at + FSCHVALUE_TO_DELAY(
					FSCHVALUE_SUB(TICK_VALUE, node->value),
					node->rate);
	} else if (node->delayed) {
		/*
		 * The node was delayed, but now it is allowed to run.
		 * We do not manipulate with lists, it will be done by the
		 * caller.
		 */
		node->nr_ready = node->nr_runnable;
		node->delayed = 0;
	}
}

static void fairsched_delayed_wake(cycles_t time)
{
	struct fairsched_node *p;

	while (!list_empty(&fairsched_delayed_head)) {
		p = list_entry(fairsched_delayed_head.next,
				  struct fairsched_node,
				  runlist);
		if (CYCLES_AFTER(p->delay, time))
			break;

		/* ok, the delay period is completed */
		/* account for the time passed since last update */
		if (CYCLES_AFTER(time, p->last_updated_at)) {
			nodevalue_add(p, FSCHDURATION(time, p->last_updated_at),
					p->rate);
			p->last_updated_at = time;
		}

		fsch_validate(FSCHVALUE_CMP(p->value, TICK_VALUE) >= 0,
				"wake, value " FSCHVALUE_FMT
				", tick " FSCHVALUE_FMT
				", delay %Lu, time %Lu"
				", lastupd %Lu, rate %u\n",
				FSCHVALUE_PRINT(p->value),
				FSCHVALUE_PRINT(TICK_VALUE),
				p->delay, time,
				p->last_updated_at, p->rate);
		p->nr_ready = p->nr_runnable;
		p->delayed = 0;
		list_del_init(&p->runlist);
		if (p->nr_ready)
			fairsched_running_insert_fromsleep(p);
	}
}

static struct fairsched_node *fairsched_find(unsigned int id);

void fairsched_cpu_online_map(int id, cpumask_t *mask)
{
	struct fairsched_node *node;

	mutex_lock(&fairsched_mutex);
	node = fairsched_find(id);
	if (node == NULL)
		*mask = CPU_MASK_NONE;
	else
		vsched_cpu_online_map(node->vsched, mask);
	mutex_unlock(&fairsched_mutex);
}

/*********************************************************************/
/*
 * The heart of the algorithm:
 * fairsched_incrun, fairsched_decrun, fairsched_schedule
 *
 * Note: old property nr_ready >= nr_pcpu doesn't hold anymore.
 * However, nr_runnable, nr_ready and delayed are maintained in sync.
 */
/*********************************************************************/

/*
 * Called on a wakeup inside the node.
 */
void fairsched_incrun(struct fairsched_node *node)
{
	if (!node->delayed && !node->nr_ready++)
		/* the node wasn't on the running list, insert */
		fairsched_running_insert_fromsleep(node);
	node->nr_runnable++;
}

/*
 * Called from inside schedule() when a sleeping state is entered.
 */
void fairsched_decrun(struct fairsched_node *node)
{
	if (!node->delayed && !--node->nr_ready)
		/* nr_ready changed 1->0, remove from the running list */
		list_del_init(&node->runlist);
	--node->nr_runnable;
}

void fairsched_inccpu(struct fairsched_node *node)
{
	node->nr_pcpu++;
	fairsched_dec_ve_strv(node, cycles);
}

static inline void __fairsched_deccpu(struct fairsched_node *node)
{
	node->nr_pcpu--;
	fairsched_inc_ve_strv(node, cycles);
}

void fairsched_deccpu(struct fairsched_node *node)
{
	if (node == &fairsched_idle_node)
		return;

	__fairsched_deccpu(node);
}

static void fairsched_account(struct fairsched_node *node,
		cycles_t time)
{
	fschdur_t duration;

	duration = FSCHDURATION(time, __get_cpu_var(prev_schedule));
#ifdef CONFIG_VE
	CYCLES_DADD(&node->owner_env->cpu_used_ve, duration);
#endif

	/*
	 * The duration is not greater than TICK_DUR since
	 * task->need_resched is always 1.
	 */
	if (FSCHTAG_DADD(&node->start_tag, duration, node->weight)) {
		fairsched_reset_start_tags();
		(void) FSCHTAG_DADD(&node->start_tag, duration,
					node->weight);
	}

	list_del_init(&node->runlist);
	if (node->rate_limited)
		fairsched_ratelimit_credit_unused(node, time, duration);
	if (!node->delayed) {
		if (node->nr_ready)
			fairsched_running_insert(node);
	} else
		fairsched_delayed_insert(node);
}

/*
 * Scheduling decision
 *
 * Updates CPU usage for the node releasing the CPU and selects a new node.
 */
struct fairsched_node *fairsched_schedule(
		struct fairsched_node *prev_node,
		struct fairsched_node *cur_node,
		int cur_node_active,
		cycles_t time)
{
	struct fairsched_node *p;

	if (prev_node != &fairsched_idle_node)
		fairsched_account(prev_node, time);
	__get_cpu_var(prev_schedule) = time;

	fairsched_delayed_wake(time);

	list_for_each_entry(p, &fairsched_running_head, runlist) {
		if (p->nr_pcpu < p->nr_ready ||
		    (cur_node_active && p == cur_node)) {
			if (p->rate_limited)
				fairsched_ratelimit_charge_advance(p, time);
			return p;
		}
	}
	return NULL;
}


/*********************************************************************/
/*
 * System calls 
 *
 * All do_xxx functions are called under fairsched semaphore and after
 * capability check.
 *
 * The binary interfaces follow some other Fair Scheduler implementations
 * (although some system call arguments are not needed for our implementation).
 */
/*********************************************************************/

static struct fairsched_node *fairsched_find(unsigned int id)
{
	struct fairsched_node *p;

	for_each_fairsched_node(p) {
		if (p->id == id)
			return p;
	}
	return NULL;
}

static int do_fairsched_mknod(unsigned int parent, unsigned int weight,
		unsigned int newid)
{
	struct fairsched_node *node;
	int retval;

	retval = -EINVAL;
	if (weight < 1 || weight > FSCHWEIGHT_MAX)
		goto out;
	if (newid < 0 || newid > INT_MAX)
		goto out;

	retval = -EBUSY;
	if (fairsched_find(newid) != NULL)
		goto out;

	retval = -ENOMEM;
	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (node == NULL)
		goto out;

	memset(node, 0, sizeof(*node));
	node->weight = weight;
	INIT_LIST_HEAD(&node->runlist);
	node->id = newid;
	node->vcpus = 0;
#ifdef CONFIG_VE
	node->owner_env = get_exec_env();
#endif

	spin_lock_irq(&fairsched_lock);
	list_add(&node->nodelist, &fairsched_node_head);
	fairsched_nr_nodes++;
	fairsched_recompute_max_latency();
	spin_unlock_irq(&fairsched_lock);

	retval = newid;
out:
	return retval;
}

asmlinkage int sys_fairsched_mknod(unsigned int parent, unsigned int weight,
				    unsigned int newid)
{
	int retval;

	if (!capable(CAP_SETVEID))
		return -EPERM;

	mutex_lock(&fairsched_mutex);
	retval = do_fairsched_mknod(parent, weight, newid);
	mutex_unlock(&fairsched_mutex);

	return retval;
}
EXPORT_SYMBOL(sys_fairsched_mknod);

static int do_fairsched_rmnod(unsigned int id)
{
	struct fairsched_node *node;
	int retval;

	retval = -EINVAL;
	node = fairsched_find(id);
	if (node == NULL)
		goto out;
	if (node == &fairsched_init_node)
		goto out;

	/* check if node is empty */
	retval = -EBUSY;
	if (vsched_taskcount(node->vsched))
		goto out;

	retval = vsched_destroy(node->vsched);
	if (retval)
		goto out;

	spin_lock_irq(&fairsched_lock);
	list_del(&node->runlist); /* required for delayed nodes */
	list_del(&node->nodelist);
	fairsched_nr_nodes--;
	fairsched_recompute_max_latency();
	spin_unlock_irq(&fairsched_lock);

	kfree(node);
	retval = 0;
out:
	return retval;
}

asmlinkage int sys_fairsched_rmnod(unsigned int id)
{
	int retval;

	if (!capable(CAP_SETVEID))
		return -EPERM;

	mutex_lock(&fairsched_mutex);
	retval = do_fairsched_rmnod(id);
	mutex_unlock(&fairsched_mutex);

	return retval;
}
EXPORT_SYMBOL(sys_fairsched_rmnod);

int do_fairsched_chwt(unsigned int id, unsigned weight)
{
	struct fairsched_node *node;

	if (id == 0)
		return -EINVAL;
	if (weight < 1 || weight > FSCHWEIGHT_MAX)
		return -EINVAL;

	node = fairsched_find(id);
	if (node == NULL)
		return -ENOENT;

	spin_lock_irq(&fairsched_lock);
	node->weight = weight;
	fairsched_recompute_max_latency();
	spin_unlock_irq(&fairsched_lock);

	return 0;
}

asmlinkage int sys_fairsched_chwt(unsigned int id, unsigned weight)
{
	int retval;

	if (!capable(CAP_SETVEID))
		return -EPERM;

	mutex_lock(&fairsched_mutex);
	retval = do_fairsched_chwt(id, weight);
	mutex_unlock(&fairsched_mutex);

	return retval;
}

int do_fairsched_vcpus(unsigned int id, unsigned int vcpus)
{
	struct fairsched_node *node;
	int ret = 0;

	if (id == 0)
		return -EINVAL;

	node = fairsched_find(id);
	if (node == NULL)
		return -ENOENT;

	if (vcpus < 1 || vcpus > num_online_cpus())
		vcpus = num_online_cpus();

	node->vcpus = vcpus;
	if (node->vsched != NULL) {
		ret = vsched_set_vcpus(node->vsched, vcpus);
		/* FIXME: adjust rate ... */
	}

	return ret;
}

asmlinkage int sys_fairsched_vcpus(unsigned int id, unsigned int vcpus)
{
	int retval;

	if (!capable(CAP_SETVEID))
		return -EPERM;

	mutex_lock(&fairsched_mutex);
	retval = do_fairsched_vcpus(id, vcpus);
	mutex_unlock(&fairsched_mutex);

	return retval;
}
EXPORT_SYMBOL(sys_fairsched_vcpus);

int do_fairsched_rate(unsigned int id, int op, unsigned rate)
{
	struct fairsched_node *node;
	cycles_t time;
	int retval;

	if (id == 0)
		return -EINVAL;
	if (op == FAIRSCHED_SET_RATE && (rate < 1 || rate >= (1UL << 31)))
		return -EINVAL;

	node = fairsched_find(id);
	if (node == NULL)
		return -ENOENT;

	retval = -EINVAL;
	spin_lock_irq(&fairsched_lock);
	time = get_cycles();
	switch (op) {
		case FAIRSCHED_SET_RATE:
			node->rate = rate;
			if (node->rate > (fairsched_nr_cpus << FSCHRATE_SHIFT))
				node->rate =
					fairsched_nr_cpus << FSCHRATE_SHIFT;
			node->rate_limited = 1;
			node->value = max_value;
			if (node->delayed) {
				list_del(&node->runlist);
				node->delay = time;
				fairsched_delayed_insert(node);
				node->last_updated_at = time;
				fairsched_delayed_wake(time);
			}
			retval = node->rate;
			break;
		case FAIRSCHED_DROP_RATE:
			node->rate = 0; /* This assignment is not needed
					   for the kernel code, and it should
					   not rely on rate being 0 when it's
					   unset.  This is a band-aid for some
					   existing tools (don't know which one
					   exactly).  --SAW */
			node->rate_limited = 0;
			node->value = max_value;
			if (node->delayed) {
				list_del(&node->runlist);
				node->delay = time;
				fairsched_delayed_insert(node);
				node->last_updated_at = time;
				fairsched_delayed_wake(time);
			}
			retval = 0;
			break;
		case FAIRSCHED_GET_RATE:
			if (node->rate_limited)
				retval = node->rate;
			else
				retval = -ENODATA;
			break;
	}
	spin_unlock_irq(&fairsched_lock);

	return retval;
}

asmlinkage int sys_fairsched_rate(unsigned int id, int op, unsigned rate)
{
	int retval;

	if (!capable(CAP_SETVEID))
		return -EPERM;

	mutex_lock(&fairsched_mutex);
	retval = do_fairsched_rate(id, op, rate);
	mutex_unlock(&fairsched_mutex);

	return retval;
}

/*
 * Called under fairsched_mutex.
 */
static int __do_fairsched_mvpr(struct task_struct *p,
		struct fairsched_node *node)
{
	int retval;

	if (node->vsched == NULL) {
		retval = vsched_create(node->id, node);
		if (retval < 0)
			return retval;
	}

	/* no need to destroy vsched in case of mvpr failure */
	return vsched_mvpr(p, node->vsched);
}

int do_fairsched_mvpr(pid_t pid, unsigned int nodeid)
{
	struct task_struct *p;
	struct fairsched_node *node;
	int retval;

	retval = -ENOENT;
	node = fairsched_find(nodeid);
	if (node == NULL)
		goto out;

	read_lock(&tasklist_lock);
	retval = -ESRCH;
	p = find_task_by_pid_all(pid);
	if (p == NULL)
		goto out_unlock;
	get_task_struct(p);
	read_unlock(&tasklist_lock);

	retval = __do_fairsched_mvpr(p, node);
	put_task_struct(p);
	return retval;

out_unlock:
	read_unlock(&tasklist_lock);
out:
	return retval;
}

asmlinkage int sys_fairsched_mvpr(pid_t pid, unsigned int nodeid)
{
	int retval;

	if (!capable(CAP_SETVEID))
		return -EPERM;

	mutex_lock(&fairsched_mutex);
	retval = do_fairsched_mvpr(pid, nodeid);
	mutex_unlock(&fairsched_mutex);

	return retval;
}
EXPORT_SYMBOL(sys_fairsched_mvpr);


/*********************************************************************/
/*
 * proc interface
 */
/*********************************************************************/

struct fairsched_node_dump {
#ifdef CONFIG_VE
	envid_t veid;
#endif
	int id;
	unsigned weight;
	unsigned rate;
	unsigned rate_limited : 1,
		 delayed : 1;
	fschtag_t start_tag;
	fschvalue_t value;
	cycles_t delay;
	int nr_ready;
	int nr_runnable;
	int nr_pcpu;
	int nr_tasks, nr_runtasks;
};

struct fairsched_dump {
	int len, compat;
	struct fairsched_node_dump nodes[0];
};

static struct fairsched_dump *fairsched_do_dump(int compat)
{
	int nr_nodes;
	int len, i;
	struct fairsched_dump *dump;
	struct fairsched_node *node;
	struct fairsched_node_dump *p;
	unsigned long flags;

start:
	nr_nodes = (ve_is_super(get_exec_env()) ? fairsched_nr_nodes + 16 : 1);
	len = sizeof(*dump) + nr_nodes * sizeof(dump->nodes[0]);
	dump = ub_vmalloc(len);
	if (dump == NULL)
		goto out;

	spin_lock_irqsave(&fairsched_lock, flags);
	if (ve_is_super(get_exec_env()) && nr_nodes < fairsched_nr_nodes)
		goto repeat;
	p = dump->nodes;
	list_for_each_entry_reverse(node, &fairsched_node_head, nodelist) {
		if ((char *)p - (char *)dump >= len)
			break;
		p->nr_tasks = 0;
		p->nr_runtasks = 0;
#ifdef CONFIG_VE
		if (!ve_accessible(node->owner_env, get_exec_env()))
			continue;
		p->veid = node->owner_env->veid;
		if (compat) {
			p->nr_tasks = atomic_read(&node->owner_env->pcounter);
			for_each_online_cpu(i)
				p->nr_runtasks +=
					VE_CPU_STATS(node->owner_env, i)
								->nr_running;
			if (p->nr_runtasks < 0)
				p->nr_runtasks = 0;
		}
#endif
		p->id = node->id;
		p->weight = node->weight;
		p->rate = node->rate;
		p->rate_limited = node->rate_limited;
		p->delayed = node->delayed;
		p->start_tag = node->start_tag;
		p->value = node->value;
		p->delay = node->delay;
		p->nr_ready = node->nr_ready;
		p->nr_runnable = node->nr_runnable;
		p->nr_pcpu = node->nr_pcpu;
		p++;
	}
	dump->len = p - dump->nodes;
	dump->compat = compat;
	spin_unlock_irqrestore(&fairsched_lock, flags);

out:
	return dump;

repeat:
	spin_unlock_irqrestore(&fairsched_lock, flags);
	vfree(dump);
	goto start;
}

#define FAIRSCHED_PROC_HEADLINES 2

#if defined(CONFIG_VE)
/*
 * File format is dictated by compatibility reasons.
 */
static int fairsched_seq_show(struct seq_file *m, void *v)
{
	struct fairsched_dump *dump;
	struct fairsched_node_dump *p;
	unsigned vid, nid, pid, r;

	dump = m->private;
	p = (struct fairsched_node_dump *)((unsigned long)v & ~3UL);
	if (p - dump->nodes < FAIRSCHED_PROC_HEADLINES) {
		if (p == dump->nodes)
			seq_printf(m, "Version: 2.6 debug\n");
		else if (p == dump->nodes + 1)
			seq_printf(m,
				       "      veid "
				       "        id "
				       "    parent "
				       "weight "
				       " rate "
  				       "tasks "
				       "  run "
				       "cpus"
				       " "
				       "flg "
				       "ready "
				       "           start_tag "
				       "               value "
				       "               delay"
				       "\n");
	} else {
		p -= FAIRSCHED_PROC_HEADLINES;
		vid = nid = pid = 0;
		r = (unsigned long)v & 3;
		if (p == dump->nodes) {
			if (r == 2)
				nid = p->id;
		} else {
			if (!r)
				nid = p->id;
			else if (r == 1)
				vid = pid = p->id;
			else
				vid = p->id, nid = 1;
		}
		seq_printf(m,
			       "%10u "
			       "%10u %10u %6u %5u %5u %5u %4u"
			       " "
			       " %c%c %5u %20Lu %20Lu %20Lu"
			       "\n",
			       vid,
			       nid,
			       pid,
			       p->weight,
			       p->rate,
			       p->nr_tasks,
			       p->nr_runtasks,
			       p->nr_pcpu,
			       p->rate_limited ? 'L' : '.',
			       p->delayed ? 'D' : '.',
			       p->nr_ready,
			       (unsigned long long)p->start_tag.t,
			       (unsigned long long)p->value.v,
			       (unsigned long long)p->delay
			       );
	}

	return 0;
}

static void *fairsched_seq_start(struct seq_file *m, loff_t *pos)
{
	struct fairsched_dump *dump;
	unsigned long l;

	dump = m->private;
	if (*pos >= dump->len * 3 - 1 + FAIRSCHED_PROC_HEADLINES)
		return NULL;
	if (*pos < FAIRSCHED_PROC_HEADLINES)
		return dump->nodes + *pos;
	/* guess why... */
	l = (unsigned long)(dump->nodes +
		((unsigned long)*pos + FAIRSCHED_PROC_HEADLINES * 2 + 1) / 3);
	l |= ((unsigned long)*pos + FAIRSCHED_PROC_HEADLINES * 2 + 1) % 3;
	return (void *)l;
}
static void *fairsched_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return fairsched_seq_start(m, pos);
}
#endif

static int fairsched2_seq_show(struct seq_file *m, void *v)
{
	struct fairsched_dump *dump;
	struct fairsched_node_dump *p;

	dump = m->private;
	p = v;
	if (p - dump->nodes < FAIRSCHED_PROC_HEADLINES) {
		if (p == dump->nodes)
			seq_printf(m, "Version: 2.7" FAIRSHED_DEBUG "\n");
		else if (p == dump->nodes + 1)
			seq_printf(m,
				       "        id "
				       "weight "
				       " rate "
				       "  run "
				       "cpus"
#ifdef FAIRSHED_DEBUG
				       " "
				       "flg "
				       "ready "
				       "           start_tag "
				       "               value "
				       "               delay"
#endif
				       "\n");
	} else {
		p -= FAIRSCHED_PROC_HEADLINES;
		seq_printf(m,
			       "%10u %6u %5u %5u %4u"
#ifdef FAIRSHED_DEBUG
			       " "
			       " %c%c %5u %20Lu %20Lu %20Lu"
#endif
			       "\n",
			       p->id,
			       p->weight,
			       p->rate,
			       p->nr_runnable,
			       p->nr_pcpu
#ifdef FAIRSHED_DEBUG
			       ,
			       p->rate_limited ? 'L' : '.',
			       p->delayed ? 'D' : '.',
			       p->nr_ready,
			       (unsigned long long)p->start_tag.t,
			       (unsigned long long)p->value.v,
			       (unsigned long long)p->delay
#endif
			       );
	}

	return 0;
}

static void *fairsched2_seq_start(struct seq_file *m, loff_t *pos)
{
	struct fairsched_dump *dump;

	dump = m->private;
	if (*pos >= dump->len + FAIRSCHED_PROC_HEADLINES)
		return NULL;
	return dump->nodes + *pos;
}
static void *fairsched2_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return fairsched2_seq_start(m, pos);
}
static void fairsched2_seq_stop(struct seq_file *m, void *v)
{
}

#ifdef CONFIG_VE
static struct seq_operations fairsched_seq_op = {
	.start		= fairsched_seq_start,
	.next		= fairsched_seq_next,
	.stop		= fairsched2_seq_stop,
	.show		= fairsched_seq_show
};
#endif
static struct seq_operations fairsched2_seq_op = {
	.start		= fairsched2_seq_start,
	.next		= fairsched2_seq_next,
	.stop		= fairsched2_seq_stop,
	.show		= fairsched2_seq_show
};
static int fairsched_seq_open(struct inode *inode, struct file *file)
{
	int ret;
	struct seq_file *m;
	int compat;

#ifdef CONFIG_VE
	compat = (file->f_dentry->d_name.len == sizeof("fairsched") - 1);
	ret = seq_open(file, compat ? &fairsched_seq_op : &fairsched2_seq_op);
#else
	compat = 0;
	ret = seq_open(file, &fairsched2_seq_op);
#endif
	if (ret)
		return ret;
	m = file->private_data;
	m->private = fairsched_do_dump(compat);
	if (m->private == NULL) {
		seq_release(inode, file);
		ret = -ENOMEM;
	}
	return ret;
}
static int fairsched_seq_release(struct inode *inode, struct file *file)
{
	struct seq_file *m;
	struct fairsched_dump *dump;

	m = file->private_data;
	dump = m->private;
	m->private = NULL;
	vfree(dump);
	seq_release(inode, file);
	return 0;
}
static struct file_operations proc_fairsched_operations = {
	.open		= fairsched_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= fairsched_seq_release
};


/*********************************************************************/
/*
 * Fairsched initialization
 */
/*********************************************************************/

int fsch_sysctl_latency(ctl_table *ctl, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int *valp = ctl->data;
	int val = *valp;
	int ret;

	ret = proc_dointvec(ctl, write, filp, buffer, lenp, ppos);

	if (!write || *valp == val)
		return ret;

	spin_lock_irq(&fairsched_lock);
	fairsched_recompute_max_latency();
	spin_unlock_irq(&fairsched_lock);
	return ret;
}

static void fairsched_calibrate(void)
{
	fairsched_nr_cpus = num_online_cpus();
	cycles_per_timeslice = msecs_to_jiffies(FSCH_TIMESLICE)
							* cycles_per_jiffy;
	max_value = FSCHVALUE(cycles_per_timeslice * (fairsched_nr_cpus + 1));
}

void __init fairsched_init_early(void)
{
	fairsched_init_node.vcpus = num_online_cpus();
	list_add(&fairsched_init_node.nodelist, &fairsched_node_head);
	fairsched_nr_nodes++;
}

/*
 * Note: this function is execute late in the initialization sequence.
 * We ourselves need calibrated cycles and initialized procfs...
 * The consequence of this late initialization is that start tags are
 * efficiently ignored and each node preempts others on insertion.
 * But it isn't a problem (only init node can be runnable).
 */
void __init fairsched_init_late(void)
{
	struct proc_dir_entry *entry;

	if (get_cycles() == 0)
		panic("FAIRSCHED: no TSC!\n");
	fairsched_calibrate();
	fairsched_recompute_max_latency();

	entry = create_proc_glob_entry("fairsched", S_IRUGO, NULL);
	if (entry)
		entry->proc_fops = &proc_fairsched_operations;
	entry = create_proc_glob_entry("fairsched2", S_IRUGO, NULL);
	if (entry)
		entry->proc_fops = &proc_fairsched_operations;
}


#else /* CONFIG_FAIRSCHED */


/*********************************************************************/
/*
 * No Fairsched
 */
/*********************************************************************/

asmlinkage int sys_fairsched_mknod(unsigned int parent, unsigned int weight,
				    unsigned int newid)
{
	return -ENOSYS;
}

asmlinkage int sys_fairsched_rmnod(unsigned int id)
{
	return -ENOSYS;
}

asmlinkage int sys_fairsched_chwt(unsigned int id, unsigned int weight)
{
	return -ENOSYS;
}

asmlinkage int sys_fairsched_mvpr(pid_t pid, unsigned int nodeid)
{
	return -ENOSYS;
}

asmlinkage int sys_fairsched_rate(unsigned int id, int op, unsigned rate)
{
	return -ENOSYS;
}

asmlinkage int sys_fairsched_vcpus(unsigned int id, unsigned int vcpus)
{
	return -ENOSYS;
}

void __init fairsched_init_late(void)
{
}

#endif /* CONFIG_FAIRSCHED */
