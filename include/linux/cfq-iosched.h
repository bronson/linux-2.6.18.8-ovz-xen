#ifndef _LINUX_CFQ_IOSCHED_H
#define _LINUX_CFQ_IOSCHED_H

#include <linux/ioprio.h>
#include <linux/mempool.h>
#include <linux/rbtree.h>

/*
 * Each block device managed by CFQ I/O scheduler is represented
 * by cfq_data structure. Certain members of this structure are
 * distinguished to cfq_bc_data on per-UBC basis. Thus cfq_bc_data
 * structure is per (Device, UBC) pare.
 *
 * BC holds a list head of all cfq_bc_data, that belong to UBC,
 * and cfq_data holds a list head of all active cfq_bc_data for
 * for the device (active means that there are requests in-flight).
 * cfq_bc_data has a pointers to owning UBC and cfq_data.
 *
 * For example, if there are two devices and three beancounters:
 *
 *	         cfq_data 1          cfq_data 2
 *	             |                   |
 *	             |                   |
 *	UB1 --- cfq_bc_data ------- cfq_bc_data
 *	             |                   |
 *	             |                   |
 *	UB2 --- cfq_bc_data ------- cfq_bc_data
 *	             |                   |
 *	             |                   |
 *	UB2 --- cfq_bc_data ------- cfq_bc_data
 *
 * One more basic structure in CFQ scheduler is cfq_queue,
 * which is a queue of requests. For sync queues it's a per-process
 * structure. While creating new cfq_queue we store cfq_bc_data
 * it belongs to, and later use this information in order to add
 * queue in proper lists.
 *
 */

extern kmem_cache_t *cfq_pool;

#define CFQ_PRIO_LISTS		IOPRIO_BE_NR

/*
 * Per (Device, UBC) queue data
 */
struct cfq_bc_data {
	/* for ub.iopriv->cfq_bc_head */
	struct list_head	cfq_bc_list;
	/* for cfqd->cfq_bc_queue */
	struct rb_node		cfq_bc_node;

	struct cfq_data		*cfqd;
	struct ub_iopriv	*ub_iopriv;

	/*
	 * rr list of queues with requests
	 */
	struct list_head	rr_list[CFQ_PRIO_LISTS];
	struct list_head	cur_rr;
	struct list_head	idle_rr;
	struct list_head	busy_rr;

	int			cur_prio;
	int			cur_end_prio;

	unsigned long		rqnum;
	unsigned long		on_dispatch;
	unsigned long		cfq_bc_iotime;
	struct cfq_queue	*async_cfqq[CFQ_PRIO_LISTS];
	/* write under cfqd->queue->request_queue_lock */
	seqcount_t		stat_lock;
	/* summarize delays between enqueue and activation. */
	unsigned long		wait_time;
	unsigned long		wait_start;
};

/*
 * Per block device queue structure
 */
struct cfq_data {
	struct request_queue *queue;

#ifndef CONFIG_UBC_IO_PRIO
	struct cfq_bc_data cfq_bc;
#endif
	unsigned int busy_queues;

	/*
	 * non-ordered list of empty cfqq's
	 */
	struct list_head empty_list;

	/*
	 * cfqq lookup hash
	 */
	struct hlist_head *cfq_hash;

	/*
	 * global crq hash for all queues
	 */
	struct hlist_head *crq_hash;

	mempool_t *crq_pool;

	int rq_in_driver;
	int hw_tag;

	/*
	 * schedule slice state info
	 */
	/*
	 * idle window management
	 */
	struct timer_list idle_slice_timer;
	struct work_struct unplug_work;

	struct cfq_queue *active_queue;
	struct cfq_io_context *active_cic;
	unsigned int dispatch_slice;

	struct timer_list idle_class_timer;

	sector_t last_sector;
	unsigned long last_end_request;

	unsigned int rq_starved;

	/*
	 * tunables, see top of file
	 */
	unsigned int cfq_quantum;
	unsigned int cfq_queued;
	unsigned int cfq_fifo_expire[2];
	unsigned int cfq_back_penalty;
	unsigned int cfq_back_max;
	unsigned int cfq_slice[2];
	unsigned int cfq_slice_async_rq;
	unsigned int cfq_slice_idle;

	struct list_head cic_list;

	/* bc priority queue */
	struct rb_root cfq_bc_queue;
	/* ub that owns a timeslice at the moment */
	struct cfq_bc_data *active_cfq_bc;
	unsigned int cfq_ub_slice;
	unsigned long slice_begin;
	unsigned long slice_end;
	unsigned long cfq_bc_position;
	int virt_mode;
	int write_virt_mode;
};

/*
 * Per process-grouping structure
 */
struct cfq_queue {
	/* reference count */
	atomic_t ref;
	/* parent cfq_data */
	struct cfq_data *cfqd;
	/* cfqq lookup hash */
	struct hlist_node cfq_hash;
	/* hash key */
	unsigned int key;
	/* on either rr or empty list of cfqd */
	struct list_head cfq_list;
	/* sorted list of pending requests */
	struct rb_root sort_list;
	/* if fifo isn't expired, next request to serve */
	struct cfq_rq *next_crq;
	/* requests queued in sort_list */
	int queued[2];
	/* currently allocated requests */
	int allocated[2];
	/* fifo list of requests in sort_list */
	struct list_head fifo;

	unsigned long slice_start;
	unsigned long slice_end;
	unsigned long slice_left;
	unsigned long service_last;

	/* number of requests that are on the dispatch list */
	int on_dispatch[2];

	/* io prio of this group */
	unsigned short ioprio, org_ioprio;
	unsigned short ioprio_class, org_ioprio_class;

	/* various state flags, see below */
	unsigned int flags;

	struct cfq_bc_data *cfq_bc;
};

static void inline cfq_init_cfq_bc(struct cfq_bc_data *cfq_bc)
{
	int i;

	for (i = 0; i < CFQ_PRIO_LISTS; i++)
		INIT_LIST_HEAD(&cfq_bc->rr_list[i]);

	INIT_LIST_HEAD(&cfq_bc->cur_rr);
	INIT_LIST_HEAD(&cfq_bc->idle_rr);
	INIT_LIST_HEAD(&cfq_bc->busy_rr);
}
#endif /* _LINUX_CFQ_IOSCHED_H */
