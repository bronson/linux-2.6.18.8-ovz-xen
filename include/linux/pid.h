#ifndef _LINUX_PID_H
#define _LINUX_PID_H

#include <linux/rcupdate.h>

#define VPID_BIT	10
#define VPID_DIV	(1<<VPID_BIT)

#ifdef CONFIG_VE
#define __is_virtual_pid(pid)	((pid) & VPID_DIV)
#define is_virtual_pid(pid)	\
   (__is_virtual_pid(pid) || ((pid)==1 && !ve_is_super(get_exec_env())))
#else
#define __is_virtual_pid(pid)	0
#define is_virtual_pid(pid)	0
#endif

enum pid_type
{
	PIDTYPE_PID,
	PIDTYPE_PGID,
	PIDTYPE_SID,
	PIDTYPE_MAX
};

/*
 * What is struct pid?
 *
 * A struct pid is the kernel's internal notion of a process identifier.
 * It refers to individual tasks, process groups, and sessions.  While
 * there are processes attached to it the struct pid lives in a hash
 * table, so it and then the processes that it refers to can be found
 * quickly from the numeric pid value.  The attached processes may be
 * quickly accessed by following pointers from struct pid.
 *
 * Storing pid_t values in the kernel and refering to them later has a
 * problem.  The process originally with that pid may have exited and the
 * pid allocator wrapped, and another process could have come along
 * and been assigned that pid.
 *
 * Referring to user space processes by holding a reference to struct
 * task_struct has a problem.  When the user space process exits
 * the now useless task_struct is still kept.  A task_struct plus a
 * stack consumes around 10K of low kernel memory.  More precisely
 * this is THREAD_SIZE + sizeof(struct task_struct).  By comparison
 * a struct pid is about 64 bytes.
 *
 * Holding a reference to struct pid solves both of these problems.
 * It is small so holding a reference does not consume a lot of
 * resources, and since a new struct pid is allocated when the numeric
 * pid value is reused we don't mistakenly refer to new processes.
 */

struct pid
{
	atomic_t count;
	/* Try to keep pid_chain in the same cacheline as nr for find_pid */
	int nr;
	struct hlist_node pid_chain;
	/* lists of tasks that use this pid */
	struct hlist_head tasks[PIDTYPE_MAX];
#ifdef CONFIG_VE
	int vnr;
	int veid;
	struct hlist_node vpid_chain;
#endif
#ifdef CONFIG_USER_RESOURCE
	struct user_beancounter *ub;
#endif
	struct rcu_head rcu;
};

struct pid_link
{
	struct hlist_node node;
	struct pid *pid;
};

static inline struct pid *get_pid(struct pid *pid)
{
	if (pid)
		atomic_inc(&pid->count);
	return pid;
}

extern void FASTCALL(put_pid(struct pid *pid));
extern struct task_struct *FASTCALL(pid_task(struct pid *pid, enum pid_type));
extern struct task_struct *FASTCALL(get_pid_task(struct pid *pid,
						enum pid_type));

/*
 * attach_pid() and detach_pid() must be called with the tasklist_lock
 * write-held.
 */
extern int FASTCALL(attach_pid(struct task_struct *task,
				enum pid_type type, int nr));

extern void FASTCALL(detach_pid(struct task_struct *task, enum pid_type));

/*
 * look up a PID in the hash table. Must be called with the tasklist_lock
 * or rcu_read_lock() held.
 */
extern struct pid *FASTCALL(find_pid(int nr));
extern struct pid *FASTCALL(find_vpid(int nr));

/*
 * Lookup a PID in the hash table, and return with it's count elevated.
 */
extern struct pid *find_get_pid(int nr);

extern struct pid *alloc_pid(void);
extern void FASTCALL(free_pid(struct pid *pid));

extern int alloc_pidmap(void);
extern fastcall void free_pidmap(int pid);

#ifndef CONFIG_VE

#define vpid_to_pid(pid)	(pid)
#define __vpid_to_pid(pid)	(pid)
#define pid_to_vpid(pid)	(pid)
#define _pid_to_vpid(pid)	(pid)

#define comb_vpid_to_pid(pid)	(pid)
#define comb_pid_to_vpid(pid)	(pid)

#else

struct ve_struct;
extern void free_vpid(struct pid *pid);
extern pid_t alloc_vpid(struct pid *pid, pid_t vpid);
extern pid_t vpid_to_pid(pid_t pid);
extern pid_t __vpid_to_pid(pid_t pid);
extern pid_t pid_to_vpid(pid_t pid);
extern pid_t _pid_to_vpid(pid_t pid);

static inline int comb_vpid_to_pid(int vpid)
{
	int pid = vpid;

	if (vpid > 0) {
		pid = vpid_to_pid(vpid);
		if (unlikely(pid < 0))
			return 0;
	} else if (vpid < 0) {
		pid = vpid_to_pid(-vpid);
		if (unlikely(pid < 0))
			return 0;
		pid = -pid;
	}
	return pid;
}

static inline int comb_pid_to_vpid(int pid)
{
	int vpid = pid;

	if (pid > 0) {
		vpid = pid_to_vpid(pid);
		if (unlikely(vpid < 0))
			return 0;
	} else if (pid < 0) {
		vpid = pid_to_vpid(-pid);
		if (unlikely(vpid < 0))
			return 0;
		vpid = -vpid;
	}
	return vpid;
}

extern int glob_virt_pids;
#endif

#define pid_next_all(task, type)				\
	((task)->pids[(type)].node.next)

#define pid_next_task_all(task, type) 				\
	hlist_entry(pid_next_all(task, type),			\
			struct task_struct, pids[(type)].node)

/* We could use hlist_for_each_entry_rcu here but it takes more arguments
 * than the do_each_task_pid/while_each_task_pid.  So we roll our own
 * to preserve the existing interface.
 */
#define do_each_task_pid_all(who, type, task)				\
	if ((task = find_task_by_pid_type_all(type, who))) {		\
		prefetch(pid_next_all(task, type));			\
		do {

#define while_each_task_pid_all(who, type, task)			\
		} while (pid_next_all(task, type) &&  ({		\
				task = pid_next_task_all(task, type);	\
				rcu_dereference(task);			\
				prefetch(pid_next_all(task, type));	\
				1; }) );				\
	}

#ifndef CONFIG_VE
#define __do_each_task_pid_ve(who, type, task, owner)			\
		do_each_task_pid_all(who, type, task)
#define __while_each_task_pid_ve(who, type, task, owner)		\
		while_each_task_pid_all(who, type, task)
#else /* CONFIG_VE */
#define __do_each_task_pid_ve(who, type, task, owner)			\
		do_each_task_pid_all(who, type, task)			\
			if (ve_accessible(VE_TASK_INFO(task)->owner_env, owner))
#define __while_each_task_pid_ve(who, type, task, owner)		\
		while_each_task_pid_all(who, type, task)
#endif /* CONFIG_VE */

#define do_each_task_pid_ve(who, type, task)				\
		__do_each_task_pid_ve(who, type, task, get_exec_env());
#define while_each_task_pid_ve(who, type, task)				\
		__while_each_task_pid_ve(who, type, task, get_exec_env());

#endif /* _LINUX_PID_H */
