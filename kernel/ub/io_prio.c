/*
 *  kernel/ub/io_prio.c
 *
 *  Copyright (C) 2007 SWsoft
 *  All rights reserved.
 *
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 *  Vasily Tarasov <vtaras@openvz.org>
 *
 */

#include <linux/mm.h>
#include <ub/io_prio.h>
#include <ub/beancounter.h>
#include <ub/ub_hash.h>
#include <linux/module.h>
#include <ub/io_acct.h>
#include <linux/blkdev.h>
#include <ub/proc.h>

#define BC_MAX_RATIO	100

/* bc bandwidth inversely proportional coefficient per ioprio */
static int bc_ioprio_ratio[CFQ_PRIO_LISTS] = {100, 87, 77, 70, 63, 58, 53, 50};

struct cfq_bc_data *__find_cfq_bc(struct ub_iopriv *iopriv,
							struct cfq_data *cfqd)
{
	struct cfq_bc_data *cfq_bc;

	list_for_each_entry(cfq_bc, &iopriv->cfq_bc_head, cfq_bc_list)
		if (cfq_bc->cfqd == cfqd)
			return cfq_bc;

	return NULL;
}

struct cfq_bc_data *bc_findcreate_cfq_bc(struct ub_iopriv *iopriv,
					struct cfq_data *cfqd, gfp_t gfp_mask)
{
	struct cfq_bc_data *cfq_bc_new;
	struct cfq_bc_data *cfq_bc;
	unsigned long flags;

	read_lock_irqsave(&iopriv->cfq_bc_list_lock, flags);
	cfq_bc = __find_cfq_bc(iopriv, cfqd);
	read_unlock_irqrestore(&iopriv->cfq_bc_list_lock, flags);

	if (cfq_bc)
		return cfq_bc;

	cfq_bc_new = kzalloc(sizeof(*cfq_bc_new), gfp_mask);
	if (!cfq_bc_new)
		return NULL;

	cfq_init_cfq_bc(cfq_bc_new);
	cfq_bc_new->cfqd = cfqd;
	cfq_bc_new->ub_iopriv = iopriv;

	write_lock_irqsave(&iopriv->cfq_bc_list_lock, flags);
	cfq_bc = __find_cfq_bc(iopriv, cfqd);
	if (cfq_bc)
		kfree(cfq_bc_new);
	else {
		list_add_tail(&cfq_bc_new->cfq_bc_list,
					&iopriv->cfq_bc_head);
		cfq_bc = cfq_bc_new;
	}
	write_unlock_irqrestore(&iopriv->cfq_bc_list_lock, flags);

	return cfq_bc;
}

void bc_init_ioprio(struct ub_iopriv *iopriv)
{
	INIT_LIST_HEAD(&iopriv->cfq_bc_head);
	rwlock_init(&iopriv->cfq_bc_list_lock);
	iopriv->ioprio = UB_IOPRIO_BASE;
}

static void inline bc_cfq_bc_check_empty(struct cfq_bc_data *cfq_bc)
{
	int i;

	for (i = 0; i < CFQ_PRIO_LISTS; i++)
		BUG_ON(!list_empty(&cfq_bc->rr_list[i]));

	BUG_ON(!list_empty(&cfq_bc->cur_rr));
 	BUG_ON(!list_empty(&cfq_bc->busy_rr));
 	BUG_ON(!list_empty(&cfq_bc->idle_rr));
}

static void bc_release_cfq_bc(struct cfq_bc_data *cfq_bc)
{
	struct cfq_data *cfqd;
	elevator_t *eq;
	int i;

	cfqd = cfq_bc->cfqd;
	eq = cfqd->queue->elevator;

	for (i = 0; i < CFQ_PRIO_LISTS; i++)
 		if (cfq_bc->async_cfqq[i]) {
			eq->ops->put_queue(cfq_bc->async_cfqq[i]);
			cfq_bc->async_cfqq[i] = NULL;
		}
	/* 
	 * Note: this cfq_bc is already not in active list,
	 * but can be still pointed from cfqd as active.
	 */
	cfqd->active_cfq_bc = NULL;

	bc_cfq_bc_check_empty(cfq_bc);
	list_del(&cfq_bc->cfq_bc_list);
	kfree(cfq_bc);
}

void bc_fini_ioprio(struct ub_iopriv *iopriv)
{
	struct cfq_bc_data *cfq_bc;
	struct cfq_bc_data *cfq_bc_tmp;
	unsigned long flags;
	spinlock_t *queue_lock;

	/* 
	 * Don't get cfq_bc_list_lock since ub is already dead,
	 * but async cfqqs are still in hash list, consequently
	 * queue_lock should be hold.
	 */
	list_for_each_entry_safe(cfq_bc, cfq_bc_tmp,
			&iopriv->cfq_bc_head, cfq_bc_list) {
		queue_lock = cfq_bc->cfqd->queue->queue_lock;
		spin_lock_irqsave(queue_lock, flags);
		bc_release_cfq_bc(cfq_bc);
		spin_unlock_irqrestore(queue_lock, flags);
	}
}

void bc_cfq_exit_queue(struct cfq_data *cfqd)
{
	struct cfq_bc_data *cfq_bc;
	struct user_beancounter *ub;

	local_irq_disable();
	for_each_beancounter(ub) {
		write_lock(&ub->iopriv.cfq_bc_list_lock);
		cfq_bc = __find_cfq_bc(&ub->iopriv, cfqd);
		if (!cfq_bc) {
			write_unlock(&ub->iopriv.cfq_bc_list_lock);
			continue;
		}
		bc_release_cfq_bc(cfq_bc);
		write_unlock(&ub->iopriv.cfq_bc_list_lock);
	}
	local_irq_enable();
}

int bc_expired(struct cfq_data *cfqd)
{
	return time_after(jiffies, cfqd->slice_end) ?  1 : 0;
}

static inline int bc_empty(struct cfq_bc_data *cfq_bc)
{
	/*
	 * consider BC as empty only if there is no requests
	 * in elevator _and_ in driver
	 */
	if (!cfq_bc->rqnum && !cfq_bc->on_dispatch)
		return 1;

	return 0;
}

static void bc_wait_start(struct cfq_bc_data *cfq_bc, unsigned long now)
{
	write_seqcount_begin(&cfq_bc->stat_lock);
	cfq_bc->wait_start = now;
	write_seqcount_end(&cfq_bc->stat_lock);
}

static void bc_wait_stop(struct cfq_bc_data *cfq_bc, unsigned long now)
{
	write_seqcount_begin(&cfq_bc->stat_lock);
	cfq_bc->wait_time += now - cfq_bc->wait_start;
	cfq_bc->wait_start = 0;
	write_seqcount_end(&cfq_bc->stat_lock);
}

static unsigned int bc_wait_time(struct cfq_bc_data *cfq_bc, unsigned long now)
{
	unsigned long res;
	unsigned seq;

	do {
		seq = read_seqcount_begin(&cfq_bc->stat_lock);
		res = cfq_bc->wait_time + now - (cfq_bc->wait_start ?: now);
	} while (read_seqcount_retry(&cfq_bc->stat_lock, seq));

	return jiffies_to_msecs(res);
}

/* return true if a iotime after b, like time_after */
static int bc_iotime_after(unsigned long a, unsigned long b)
{
	return (long)a - (long)b > 0;
}

/* cfq bc queue rb_tree helper function */
static void bc_insert(struct cfq_data *cfqd, struct cfq_bc_data *cfq_bc)
{
	struct rb_node **p = &cfqd->cfq_bc_queue.rb_node;
	struct rb_node *parent = NULL;
	struct cfq_bc_data *__cfq_bc;

	while (*p) {
		parent = *p;
		__cfq_bc = rb_entry(parent, struct cfq_bc_data, cfq_bc_node);
		/* important: if equal push right */
		if (bc_iotime_after(__cfq_bc->cfq_bc_iotime,
					cfq_bc->cfq_bc_iotime))
			p = &(*p)->rb_left;
		else
			p = &(*p)->rb_right;
	}
	rb_link_node(&cfq_bc->cfq_bc_node, parent, p);
	rb_insert_color(&cfq_bc->cfq_bc_node, &cfqd->cfq_bc_queue);
}

static void bc_remove(struct cfq_data *cfqd, struct cfq_bc_data *cfq_bc)
{
	rb_erase(&cfq_bc->cfq_bc_node, &cfqd->cfq_bc_queue);
}

static void bc_enqueue(struct cfq_data *cfqd, struct cfq_bc_data *cfq_bc)
{
	unsigned long iotime, slice, position;

	iotime = cfq_bc->cfq_bc_iotime;
	slice = cfqd->cfq_ub_slice * BC_MAX_RATIO;
	position = cfqd->cfq_bc_position;

	/* adjust iotime to hit in interval position +/- maximum slice */
	if (bc_iotime_after(position, iotime + slice)
			|| bc_iotime_after(iotime, position + slice))
		cfq_bc->cfq_bc_iotime = position;

	bc_insert(cfqd, cfq_bc);

	if (cfq_bc != cfqd->active_cfq_bc)
		bc_wait_start(cfq_bc, jiffies);
}

static void bc_dequeue(struct cfq_data *cfqd, struct cfq_bc_data *cfq_bc)
{
	bc_remove(cfqd, cfq_bc);
}

/* update bc iotime */
static void bc_update(struct cfq_data *cfqd, struct cfq_bc_data *cfq_bc,
		unsigned long delta)
{
	int ioprio;

	ioprio = cfq_bc->ub_iopriv->ioprio;
	delta *= bc_ioprio_ratio[ioprio];
	cfq_bc->cfq_bc_iotime += delta;

	if (!cfq_bc->rqnum)
		return;

	bc_remove(cfqd, cfq_bc);
	bc_insert(cfqd, cfq_bc);
}

static inline void bc_set_active(struct cfq_data *cfqd)
{
	struct cfq_bc_data *cfq_bc;
	unsigned long now = jiffies;

	/* update iotime of last active bc according to used time */
	cfq_bc = cfqd->active_cfq_bc;
	if (cfq_bc && cfqd->slice_begin)
		bc_update(cfqd, cfq_bc, now - cfqd->slice_begin);

	/* if no active BCs then keep this as an active one */
	if (RB_EMPTY_ROOT(&cfqd->cfq_bc_queue)) {
		cfqd->slice_begin = 0;
		return;
	}

	if (cfq_bc && cfq_bc->rqnum)
		bc_wait_start(cfq_bc, now);

	/* peek first bc from queue */
	cfq_bc = rb_entry(rb_first(&cfqd->cfq_bc_queue),
			struct cfq_bc_data, cfq_bc_node);

	/* adjust queue active position */
	if (bc_iotime_after(cfq_bc->cfq_bc_iotime, cfqd->cfq_bc_position))
		cfqd->cfq_bc_position = cfq_bc->cfq_bc_iotime;

	cfqd->active_cfq_bc = cfq_bc;
	cfqd->slice_begin = now;
	cfqd->slice_end = now + cfqd->cfq_ub_slice;

	bc_wait_stop(cfq_bc, now);
}

void bc_schedule_active(struct cfq_data *cfqd)
{
	if (bc_expired(cfqd) || !cfqd->active_cfq_bc ||
				bc_empty(cfqd->active_cfq_bc))
		bc_set_active(cfqd);
}

void bc_inc_rqnum(struct cfq_queue *cfqq)
{
	struct cfq_bc_data *cfq_bc;

	cfq_bc = cfqq->cfq_bc;

	if (!cfq_bc->rqnum)
		bc_enqueue(cfq_bc->cfqd, cfq_bc);

	cfq_bc->rqnum++;
}

void bc_dec_rqnum(struct cfq_queue *cfqq)
{
	struct cfq_bc_data *cfq_bc;

	cfq_bc = cfqq->cfq_bc;

	cfq_bc->rqnum--;

	if (!cfq_bc->rqnum)
		bc_dequeue(cfq_bc->cfqd, cfq_bc);
}

unsigned long bc_set_ioprio(int ubid, int ioprio)
{
	struct user_beancounter *ub;

	if (ioprio < UB_IOPRIO_MIN || ioprio >= UB_IOPRIO_MAX)
		return -ERANGE;

	ub = get_beancounter_byuid(ubid, 0);
 	if (!ub)
		return -ESRCH;

	ub->iopriv.ioprio = ioprio;
	put_beancounter(ub);
 
	return 0;
}

struct user_beancounter *bc_io_switch_context(struct page *page)
{
	struct page_beancounter *pb;
	struct user_beancounter *old_ub = NULL;

	pb = page_iopb(page);
	pb = iopb_to_pb(pb);
	if (pb) {
		get_beancounter(pb->ub);
		old_ub = set_exec_ub(pb->ub);
	}
	
	return old_ub;
}

void bc_io_restore_context(struct user_beancounter *ub)
{
	struct user_beancounter *old_ub;

	if (ub) {
		old_ub = set_exec_ub(ub);
		put_beancounter(old_ub);
	}
}

#ifdef CONFIG_PROC_FS
static int bc_ioprio_show(struct seq_file *f, void *v)
{
	struct user_beancounter *bc;

	bc = seq_beancounter(f);
	seq_printf(f, "prio: %u\n", bc->iopriv.ioprio);

	return 0;
}

static struct bc_proc_entry bc_ioprio_entry = {
	.name = "ioprio",
	.u.show = bc_ioprio_show,
};

static int bc_ioprio_queue_show(struct seq_file *f, void *v)
{
	struct user_beancounter *bc;
	struct cfq_bc_data *cfq_bc;
	unsigned long now = jiffies;

	bc = seq_beancounter(f);

	read_lock_irq(&bc->iopriv.cfq_bc_list_lock);
	list_for_each_entry(cfq_bc, &bc->iopriv.cfq_bc_head, cfq_bc_list) {
		struct cfq_data *cfqd;

		cfqd = cfq_bc->cfqd;
		seq_printf(f, "\t%-10s%6lu %c%c %10u\n",
				/*
				 * this per-bc -> queue-data -> queue -> device
				 * access is safe w/o additional locks, since
				 * all the stuff above dies in the order shown
				 * and we're holding the first element
				 */
				kobject_name(cfqd->queue->kobj.parent),
				cfq_bc->rqnum,
				cfq_bc->on_dispatch ? 'D' : ' ',
				cfqd->active_cfq_bc == cfq_bc ? 'A' : ' ',
				bc_wait_time(cfq_bc, now));
	}
	read_unlock_irq(&bc->iopriv.cfq_bc_list_lock);

	return 0;
}

static struct bc_proc_entry bc_ioprio_queues_entry = {
	.name = "ioprio_queues",
	.u.show = bc_ioprio_queue_show,
};

static int __init bc_ioprio_init(void)
{
	bc_register_proc_entry(&bc_ioprio_entry);
	bc_register_proc_entry(&bc_ioprio_queues_entry);
	return 0;
}

late_initcall(bc_ioprio_init);
#endif

EXPORT_SYMBOL(bc_io_switch_context);
EXPORT_SYMBOL(bc_io_restore_context);
EXPORT_SYMBOL(__find_cfq_bc);
EXPORT_SYMBOL(bc_fini_ioprio);
EXPORT_SYMBOL(bc_init_ioprio);
EXPORT_SYMBOL(bc_findcreate_cfq_bc);
EXPORT_SYMBOL(bc_cfq_exit_queue);
EXPORT_SYMBOL(bc_expired);
EXPORT_SYMBOL(bc_schedule_active);
EXPORT_SYMBOL(bc_inc_rqnum);
EXPORT_SYMBOL(bc_dec_rqnum);
