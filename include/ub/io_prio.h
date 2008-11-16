/*
 *  include/ub/io_prio.h
 *
 *  Copyright (C) 2007 SWsoft
 *  All rights reserved.
 *
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 *  Vasily Tarasov <vtaras@openvz.org>
 *
 */

#ifndef _UB_IO_PRIO_H
#define _UB_IO_PRIO_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/cfq-iosched.h>

#define UB_IOPRIO_MIN 0
#define UB_IOPRIO_MAX IOPRIO_BE_NR
#define UB_IOPRIO_BASE 4

struct ub_iopriv {
	struct list_head	cfq_bc_head;
	rwlock_t		cfq_bc_list_lock;

	unsigned int		ioprio;
};

#ifdef CONFIG_UBC_IO_PRIO
extern void bc_init_ioprio(struct ub_iopriv *);
extern void bc_fini_ioprio(struct ub_iopriv *);
extern struct cfq_bc_data * bc_findcreate_cfq_bc(struct ub_iopriv *,
					struct cfq_data *, gfp_t gfp_mask);
extern void bc_cfq_exit_queue(struct cfq_data *);
extern int bc_expired(struct cfq_data *);
extern void bc_schedule_active(struct cfq_data *);
extern void  bc_inc_rqnum(struct cfq_queue *);
extern void bc_dec_rqnum(struct cfq_queue *);
extern unsigned long bc_set_ioprio(int, int);
extern struct cfq_bc_data *
__find_cfq_bc(struct ub_iopriv *iopriv, struct cfq_data *cfqd);
extern struct user_beancounter *bc_io_switch_context(struct page *);
extern void bc_io_restore_context(struct user_beancounter *);
#else
static inline void bc_init_ioprio(struct ub_iopriv *iopriv) { ; }
static inline void bc_fini_ioprio(struct ub_iopriv *iopriv) { ; }
static inline struct cfq_bc_data *
bc_findcreate_cfq_bc(struct ub_iopriv *iopriv,
			struct cfq_data *cfqd, gfp_t mask)
{
	return &cfqd->cfq_bc;
}
static inline void bc_cfq_exit_queue(struct cfq_data *cfqd) { ; }
static inline int bc_expired(struct cfq_data *cfqd) { return 0; }
static inline void bc_schedule_active(struct cfq_data *cfqd)
{
	cfqd->active_cfq_bc = &cfqd->cfq_bc;
}
static inline void bc_inc_rqnum(struct cfq_queue *cfqq) { ; }
static inline void bc_dec_rqnum(struct cfq_queue *cfqq) { ; }
static inline unsigned long bc_set_ioprio(int ubid, int ioprio)
{
	return -EINVAL;
}
static inline struct cfq_bc_data *
__find_cfq_bc(struct ub_iopriv *iopriv, struct cfq_data *cfqd)
{
	return &cfqd->cfq_bc;
}
static inline struct user_beancounter *
bc_io_switch_context(struct page *page) { return NULL; }
static inline void bc_io_restore_context(struct user_beancounter *ub) { ; }
#endif /* CONFIG_UBC_IO_PRIO */
#endif /* _UB_IO_PRIO_H */
