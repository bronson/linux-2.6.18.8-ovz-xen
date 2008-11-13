#ifndef __LINUX_FAIRSCHED_H__
#define __LINUX_FAIRSCHED_H__

/*
 * Fair Scheduler
 *
 * Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#define FAIRSCHED_SET_RATE	0
#define FAIRSCHED_DROP_RATE	1
#define FAIRSCHED_GET_RATE	2

#ifdef __KERNEL__
#include <linux/cache.h>
#include <asm/timex.h>

#define FAIRSCHED_HAS_CPU_BINDING	0

typedef struct { cycles_t t; } fschtag_t;
typedef struct { unsigned long d; } fschdur_t;
typedef struct { cycles_t v; } fschvalue_t;

struct vcpu_scheduler;

struct fairsched_node {
	struct list_head runlist;

	/*
	 * Fair Scheduler fields
	 *
	 * nr_running >= nr_ready (!= if delayed)
	 */
	fschtag_t start_tag;
	int nr_ready;
	int nr_runnable;
	int nr_pcpu;
	int vcpus;

	/*
	 * Rate limitator fields
	 */
	cycles_t last_updated_at;
	fschvalue_t value;	/* leaky function value */
	cycles_t delay;		/* removed from schedule till */
	unsigned char delayed;

	/*
	 * Configuration
	 *
	 * Read-only most of the time.
	 */
	unsigned weight ____cacheline_aligned_in_smp;
				/* fairness weight */
	unsigned char rate_limited;
	unsigned rate;		/* max CPU share */
	fschtag_t max_latency;
	unsigned min_weight;

	struct list_head nodelist;
	int id;
#ifdef CONFIG_VE
	struct ve_struct *owner_env;
#endif
	struct vcpu_scheduler *vsched;
};

#define for_each_fairsched_node(n)	\
	list_for_each_entry((n), &fairsched_node_head, nodelist)

#ifdef CONFIG_FAIRSCHED

#define FSCHWEIGHT_MAX			((1 << 16) - 1)
#define FSCHRATE_SHIFT			10
/* 
 * Fairsched timeslice value (in msecs) specifies maximum possible time a 
 * node can be running continuously without rescheduling, in other words
 * main linux scheduler must call fairsched_scheduler() during 
 * FSCH_TIMESLICE msecs or fairscheduler logic will be broken.
 *
 * Should be bigger for better performance, and smaller for good interactivity.
 */
#define FSCH_TIMESLICE			16

/*
 * Fairsched nodes used in boot process.
 */
extern struct fairsched_node fairsched_init_node;
extern struct fairsched_node fairsched_idle_node;

/*
 * For proc output.
 */
extern unsigned fairsched_nr_cpus;
extern void fairsched_cpu_online_map(int id, cpumask_t *mask);

/* I hope vsched_id is always equal to fairsched node id  --SAW */
#define task_fairsched_node_id(p)	task_vsched_id(p)

/*
 * Core functions.
 */
extern void fairsched_incrun(struct fairsched_node *node);
extern void fairsched_decrun(struct fairsched_node *node);
extern void fairsched_inccpu(struct fairsched_node *node);
extern void fairsched_deccpu(struct fairsched_node *node);
extern struct fairsched_node *fairsched_schedule(
		struct fairsched_node *prev_node,
		struct fairsched_node *cur_node,
		int cur_node_active,
		cycles_t time);

/*
 * Management functions.
 */
void fairsched_init_early(void);
asmlinkage int sys_fairsched_mknod(unsigned int parent, unsigned int weight,
		unsigned int newid);
asmlinkage int sys_fairsched_rmnod(unsigned int id);
asmlinkage int sys_fairsched_mvpr(pid_t pid, unsigned int nodeid);
asmlinkage int sys_fairsched_vcpus(unsigned int id, unsigned int vcpus);

#else /* CONFIG_FAIRSCHED */

#define task_fairsched_node_id(p)	0
#define fairsched_incrun(p)		do { } while (0)
#define fairsched_decrun(p)		do { } while (0)
#define fairsched_inccpu(p)		do { } while (0)
#define fairsched_deccpu(p)		do { } while (0)
#define fairsched_cpu_online_map(id, mask)      do { *(mask) = cpu_online_map; } while (0)

#endif /* CONFIG_FAIRSCHED */
#endif /* __KERNEL__ */

#endif /* __LINUX_FAIRSCHED_H__ */
