/*
 *
 *  kernel/cpt/cpt_dump.c
 *
 *  Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/pagemap.h>
#include <linux/ptrace.h>
#include <linux/smp_lock.h>
#include <linux/ve.h>
#include <linux/ve_proto.h>
#include <linux/virtinfo.h>
#include <linux/virtinfoscp.h>
#include <ub/ub_task.h>
#include <linux/cpt_image.h>
#include <linux/nsproxy.h>
#include <linux/namespace.h>
#include <linux/netdevice.h>
#include <linux/nfcalls.h>
#include <linux/dcache.h>
#include <linux/if_tun.h>

#include "cpt_obj.h"
#include "cpt_context.h"
#include "cpt_dump.h"
#include "cpt_files.h"
#include "cpt_mm.h"
#include "cpt_process.h"
#include "cpt_net.h"
#include "cpt_socket.h"
#include "cpt_ubc.h"
#include "cpt_kernel.h"


static int vps_child_level(struct task_struct *root, struct task_struct *c)
{
	int level = 0;
	int veid = VE_TASK_INFO(c)->owner_env->veid;

	while (VE_TASK_INFO(c)->owner_env->veid == veid) {
		if (c->pid != c->tgid)
			c = c->group_leader;
		if (c == root)
			return level;

		c = c->real_parent;
		level++;
	}
	return -1;
}

static inline int freezable(struct task_struct * p)
{
	if (p->exit_state)
		return 0;

	switch (p->state) {
	case EXIT_ZOMBIE:
	case EXIT_DEAD:
	case TASK_STOPPED:
#if TASK_TRACED != TASK_STOPPED
	case TASK_TRACED:
#endif
		return 0;
	default:
		return 1;
	}
}

static void wake_ve(cpt_context_t *ctx)
{
	struct task_struct *p, *g;

	do_each_thread_ve(g, p) {
		spin_lock_irq(&p->sighand->siglock);
		if (p->flags & PF_FROZEN) {
			p->flags &= ~PF_FROZEN;
			wake_up_process(p);
		}
		spin_unlock_irq(&p->sighand->siglock);
	} while_each_thread_ve(g, p);
}

/*
 * Some comment is necessary about PF_FREEZE,PF_FROZEN,TIF_FREEZE...
 *
 * SWSUSP uses PF_FREEZE flag in tsk->flags raising it in context
 * of another process. Apparently, it is unacceptable on SMP.
 * Let's take freeze_processes() in kernel/power/process.c as an example.
 * Unserialized modifications tsk->flags easily
 * (believe or not, but it happens with probability of almost 100% :-))
 * creates the situation when setting PF_FREEZE in freeze_processes(),
 * which quickly spins raising PF_FREEZE of all the processes,
 * _clears_ PF_FROZEN just set in refrigerator(), so that suspend deadlocks.
 *
 * So, to make things clean, we require that those flags may be modified
 * only under tsk->sighand->siglock, which is quite natural because PF_FREEZE
 * is just a kind of signal.
 *
 * It is not enough, because we are still not allowed to change tsk->flags
 * in context of another process, we can corrupt another flags, when the process
 * running on another cpu modifies them. So, we use TIF_FREEZE in thread flags,
 * which can be changed atomically.
 *
 * PF_FROZEN also changes in context of another process, but this happens
 * only when the process is already in refrigerator() which does not modify
 * tsk->flags.
 */

enum
{
	OBSTACLE_NOGO = -1,
	OBSTACLE_TIMEOUT = -2,
	OBSTACLE_TRYAGAIN = -3,
};

#define SUSPEND_TIMEOUT	(10UL*HZ)

static int vps_stop_tasks(struct cpt_context *ctx)
{
	unsigned long start_time = jiffies;
	unsigned long target, timeout;
	struct task_struct *p, *g;
	int todo;
	int round = 0;

	do_gettimespec(&ctx->start_time); 
	do_posix_clock_monotonic_gettime(&ctx->cpt_monotonic_time);
	ctx->virt_jiffies64 = get_jiffies_64() + get_exec_env()->jiffies_fixup;

	read_lock(&tasklist_lock);

	atomic_inc(&get_exec_env()->suspend);
	timeout = HZ/5;
	target = jiffies + timeout;

	for(;;) {
		struct task_struct *root;
		todo = 0;

		root = find_task_by_pid_ve(1);
		if (!root) {
			read_unlock(&tasklist_lock);
			eprintk_ctx("cannot find ve init\n");
			atomic_dec(&get_exec_env()->suspend);
			return -ESRCH;
		}

		do_each_thread_ve(g, p) {
			if (vps_child_level(root, p) >= 0) {
				if (!is_virtual_pid(virt_pid(p))) {
					eprintk_ctx("external process %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n",
							virt_pid(p), p->pid, p->comm);
					todo = OBSTACLE_NOGO;
					goto out;
				}
				if (!is_virtual_pid(virt_pgid(p))) {
					eprintk_ctx("external process group %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n",
							virt_pgid(p), p->pid, p->comm);
					todo = OBSTACLE_NOGO;
					goto out;
				}
				if (!is_virtual_pid(virt_sid(p))) {
					eprintk_ctx("external process session %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n",
							virt_sid(p), p->pid, p->comm);
					todo = OBSTACLE_NOGO;
					goto out;
				}
				if (p->vfork_done) {
					/* Task between vfork()...exec()
					 * cannot be frozen, because parent
					 * wait in uninterruptible state.
					 * So, we do nothing, waiting for
					 * exec(), unless:
					 */
					if (p->state == TASK_STOPPED ||
					    p->state == TASK_TRACED) {
						eprintk_ctx("task " CPT_FID " is stopped while vfork(). "
								"Checkpointing is impossible.\n",
								CPT_TID(p));
						todo = OBSTACLE_NOGO;
						/* It is fatal, _user_ stopped
						 * vfork()ing task, so that we
						 * cannot suspend now.
						 */
					} else {
						todo = OBSTACLE_TRYAGAIN;
					}
					goto out;
				}
				if (p->signal->group_exit_task &&
				    p->signal->notify_count) {
					/* exec() waits for threads' death */
					wprintk_ctx("task " CPT_FID " waits for threads' death\n", CPT_TID(p));
					todo = OBSTACLE_TRYAGAIN;
					goto out;
				}
				if (p->state == TASK_TRACED
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
				    && !p->stopped_state
#endif
				    ) {
					int ptrace_id = p->pn_state;
					/* Debugger waits for signal. */
					switch (ptrace_id) {
					case PN_STOP_TF:
					case PN_STOP_TF_RT:
					case PN_STOP_ENTRY:
					case PN_STOP_FORK:
					case PN_STOP_VFORK:
					case PN_STOP_SIGNAL:
					case PN_STOP_EXIT:
					case PN_STOP_LEAVE:
						break;
					default:
						eprintk_ctx("task " CPT_FID " is stopped by debugger while %d.\n", CPT_TID(p), ptrace_id);
						todo = OBSTACLE_NOGO;
						goto out;
					}
				}
				if (p->flags & PF_NOFREEZE) {
					eprintk_ctx("task " CPT_FID " is unfreezable. Checkpointing is impossible.\n", CPT_TID(p));
					todo = OBSTACLE_NOGO;
					goto out;
				}

				if (!freezable(p))
					continue;

				spin_lock_irq(&p->sighand->siglock);
				if (!(p->flags & PF_FROZEN)) {
					set_tsk_thread_flag(p, TIF_FREEZE);
					signal_wake_up(p, 0);
				}
				spin_unlock_irq(&p->sighand->siglock);

				if (p->flags & PF_FROZEN) {
					if (p->state != TASK_UNINTERRUPTIBLE)
						printk("Holy Crap 1 %ld " CPT_FID "\n", p->state, CPT_TID(p));
					continue;
				}

				if (round == 10)
					wprintk_ctx(CPT_FID " is running\n", CPT_TID(p));

				todo++;
			} else {
				if (p != current) {
					eprintk_ctx("foreign process %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n",
							virt_pid(p), p->pid, p->comm);
					todo = OBSTACLE_NOGO;
					goto out;
				}
			}
		} while_each_thread_ve(g, p);

		if (todo > 0) {
			/* No visible obstacles, but VE did not freeze
			 * for timeout. Interrupt suspend, if it is major
			 * timeout or signal; if it is minor timeout
			 * we will wake VE and restart suspend.
			 */
			if (time_after(jiffies, start_time + SUSPEND_TIMEOUT)
			    || signal_pending(current))
				todo = OBSTACLE_TIMEOUT;
			else if (time_after(jiffies, target))
				todo = OBSTACLE_TRYAGAIN;
		}

out:
		if (todo < 0) {
			atomic_dec(&get_exec_env()->suspend);

			wake_ve(ctx);

#if 0
			/* This is sign of failure of printk(), which is not
			 * ours. So, no prefixes. */
			printk(">\n");
#endif
		}

		read_unlock(&tasklist_lock);

		if (!todo) {
			atomic_dec(&get_exec_env()->suspend);
			return 0;
		}

		switch (todo) {
		case OBSTACLE_NOGO:
			eprintk_ctx("suspend is impossible now.\n");
			return -EAGAIN;

		case OBSTACLE_TIMEOUT:
			eprintk_ctx("interrupted or timed out.\n");
			return -EINTR;

		case OBSTACLE_TRYAGAIN:
			if (time_after(jiffies, start_time + SUSPEND_TIMEOUT) ||
			    signal_pending(current)) {
				wprintk_ctx("suspend timed out\n");
				return -EAGAIN;
			}

			wprintk_ctx("minor suspend timeout (%lu) expired, "
				    "trying again\n", timeout);

			/* Try again. VE is awake, give it some time to run. */
			current->state = TASK_INTERRUPTIBLE;
			schedule_timeout(HZ);

			/* After a short wait restart suspend
			 * with longer timeout */
			atomic_inc(&get_exec_env()->suspend);
			timeout = min(timeout<<1, SUSPEND_TIMEOUT);
			target = jiffies + timeout;
			break;

		default:
			if (round > 0) {
				/* VE is partially frozen, give processes
				 * a chance to enter to refrigerator(). */
				current->state = TASK_INTERRUPTIBLE;
				schedule_timeout(HZ/20);
			} else {
				yield();
			}
		}

		read_lock(&tasklist_lock);
		round++;
	}
}

static int cpt_unlock_ve(struct cpt_context *ctx)
{
	struct ve_struct *env;

	env = get_ve_by_id(ctx->ve_id);
	if (!env)
		return -ESRCH;
	down_write(&env->op_sem);
	env->is_locked = 0;
	up_write(&env->op_sem);
	put_ve(env);
	return 0;
}

int cpt_resume(struct cpt_context *ctx)
{
	cpt_object_t *obj;

	virtinfo_notifier_call(VITYPE_SCP, VIRTINFO_SCP_DMPFIN, ctx);

	cpt_unlock_sockets(ctx);

#ifdef CONFIG_VZ_CHECKPOINT_LAZY
	if (ctx->pgin_task) {
		wait_for_completion(&ctx->pgin_notify);
		put_task_struct(ctx->pgin_task);
		ctx->pgin_task = NULL;
	}
#endif

	for_each_object(obj, CPT_OBJ_TASK) {
		struct task_struct *tsk = obj->o_obj;

		spin_lock_irq(&tsk->sighand->siglock);
		if (tsk->flags & PF_FROZEN) {
			tsk->flags &= ~PF_FROZEN;
			wake_up_process(tsk);
		} else if (freezable(tsk)) {
			eprintk_ctx("strange, %s not frozen\n", tsk->comm );
		}
		spin_unlock_irq(&tsk->sighand->siglock);
		put_task_struct(tsk);
	}

	cpt_resume_network(ctx);

	cpt_unlock_ve(ctx);

	cpt_finish_ubc(ctx);
	cpt_object_destroy(ctx);
	return 0;
}

int cpt_kill(struct cpt_context *ctx)
{
	int err = 0;
	struct ve_struct *env;
	cpt_object_t *obj;
	struct task_struct *root_task = NULL;
	long delay;

	if (!ctx->ve_id)
		return -EINVAL;

	env = get_ve_by_id(ctx->ve_id);
	if (!env)
		return -ESRCH;

	/* from here cpt_kill succeeds */
	virtinfo_notifier_call(VITYPE_SCP, VIRTINFO_SCP_DMPFIN, ctx);

	if (current->ve_task_info.owner_env == env) {
		wprintk_ctx("attempt to kill ve from inside, escaping...\n");
		ve_move_task(current, get_ve0());
	}

#ifdef CONFIG_VZ_CHECKPOINT_LAZY
	if (ctx->pgin_task) {
		wait_for_completion(&ctx->pgin_notify);
		put_task_struct(ctx->pgin_task);
		ctx->pgin_task = NULL;
	}
#endif

	cpt_kill_sockets(ctx);

	for_each_object(obj, CPT_OBJ_TASK) {
		struct task_struct *tsk = obj->o_obj;

		if (tsk->exit_state) {
			put_task_struct(tsk);
			continue;
		}

		if (virt_pid(tsk) == 1) {
			root_task = tsk;
			continue;
		}

		tsk->robust_list = NULL;
#ifdef CONFIG_COMPAT
		tsk->compat_robust_list = NULL;
#endif
		tsk->clear_child_tid = NULL;

		if (tsk->ptrace) {
			write_lock_irq(&tasklist_lock);
			tsk->ptrace = 0;
			if (!list_empty(&tsk->ptrace_list)) {
				list_del_init(&tsk->ptrace_list);
				remove_parent(tsk);
				tsk->parent = tsk->real_parent;
				add_parent(tsk);
			}
			write_unlock_irq(&tasklist_lock);
		}

		send_sig(SIGKILL, tsk, 1);

		spin_lock_irq(&tsk->sighand->siglock);
		sigfillset(&tsk->blocked);
		sigdelsetmask(&tsk->blocked, sigmask(SIGKILL));
		set_tsk_thread_flag(tsk, TIF_SIGPENDING);
		if (tsk->flags & PF_FROZEN)
			tsk->flags &= ~PF_FROZEN;
		spin_unlock_irq(&tsk->sighand->siglock);

		wake_up_process(tsk);
		put_task_struct(tsk);
	}

	yield();

	if (root_task != NULL) {
		send_sig(SIGKILL, root_task, 1);

		spin_lock_irq(&root_task->sighand->siglock);
		sigfillset(&root_task->blocked);
		sigdelsetmask(&root_task->blocked, sigmask(SIGKILL));
		set_tsk_thread_flag(root_task, TIF_SIGPENDING);
		clear_tsk_thread_flag(root_task, TIF_FREEZE);
		if (root_task->flags & PF_FROZEN)
			root_task->flags &= ~PF_FROZEN;
		spin_unlock_irq(&root_task->sighand->siglock);

		wake_up_process(root_task);
		put_task_struct(root_task);
	}

	cpt_finish_ubc(ctx);
	cpt_object_destroy(ctx);

	delay = 1;
	while (atomic_read(&env->counter) != 1) {
		if (signal_pending(current))
			break;
		current->state = TASK_INTERRUPTIBLE;
		delay = (delay < HZ) ? (delay << 1) : HZ;
		schedule_timeout(delay);
	}
	put_ve(env);

	return err;
}

#ifdef CONFIG_USER_RESOURCE
static void collect_task_ubc(struct task_struct *t, struct cpt_context *ctx)
{
	struct task_beancounter *tbc;

	tbc = &(t->task_bc);
	cpt_add_ubc(tbc->exec_ub, ctx);
	cpt_add_ubc(tbc->task_ub, ctx);
	cpt_add_ubc(tbc->fork_sub, ctx);
}
#else
static void inline collect_task_ubc(struct task_struct *t,
		struct cpt_context *ctx)
{ return; }
#endif

static cpt_object_t * remember_task(struct task_struct * child,
		cpt_object_t * head, cpt_context_t * ctx)
{
	cpt_object_t *cobj;

	if (freezable(child) && !(child->flags&PF_FROZEN)) {
		eprintk_ctx("process " CPT_FID " is not frozen\n", CPT_TID(child));
		put_task_struct(child);
		return NULL;
	}

	if (lookup_cpt_object(CPT_OBJ_TASK, child, ctx)) BUG();
	if ((cobj = alloc_cpt_object(GFP_KERNEL, ctx)) == NULL) {
		put_task_struct(child);
		return NULL;
	}
	cobj->o_count = 1;
	cpt_obj_setobj(cobj, child, ctx);
	insert_cpt_object(CPT_OBJ_TASK, cobj, head, ctx);
	collect_task_ubc(child, ctx);
	return cobj;
}

static int vps_collect_tasks(struct cpt_context *ctx)
{
	int err = -ESRCH;
	cpt_object_t *obj;
	struct task_struct *root;
	read_lock(&tasklist_lock);
	root = find_task_by_pid_ve(1);
	if (root)
		get_task_struct(root);
	read_unlock(&tasklist_lock);

	if (!root) {
		err = -ESRCH;
		eprintk_ctx("vps_collect_tasks: cannot find root\n");
		goto out;
	}

	if ((obj = alloc_cpt_object(GFP_KERNEL, ctx)) == NULL) {
		put_task_struct(root);
		return -ENOMEM;
	}
	obj->o_count = 1;
	cpt_obj_setobj(obj, root, ctx);
	intern_cpt_object(CPT_OBJ_TASK, obj, ctx);
	collect_task_ubc(root, ctx);

	/* Collect process subtree recursively */
	for_each_object(obj, CPT_OBJ_TASK) {
		cpt_object_t *head = obj;
		struct task_struct *tsk = obj->o_obj;
		struct task_struct *child;

		if (freezable(tsk) && !(tsk->flags&PF_FROZEN)) {
			eprintk_ctx("process " CPT_FID " is not frozen\n", CPT_TID(tsk));
			err = -EINVAL;
			goto out;
		}

		if (tsk->state == TASK_RUNNING)
			printk("Holy Crap 2 %ld " CPT_FID "\n", tsk->state, CPT_TID(tsk));

		wait_task_inactive(tsk);

		if (tsk->pid == tsk->tgid) {
			child = tsk;
			for (;;) {
				read_lock(&tasklist_lock);
				child = next_thread(child);
				if (child != tsk)
					get_task_struct(child);
				read_unlock(&tasklist_lock);

				if (child == tsk)
					break;

				if (child->real_parent != tsk->real_parent) {
					put_task_struct(child);
					eprintk_ctx("illegal thread structure, kernel bug\n");
					return -EINVAL;
				}

				if ((head = remember_task(child, head, ctx)) == NULL) {
					eprintk_ctx("task obj allocation failure\n");
					err = -ENOMEM;
					goto out;
				}
			}
		}

		/* About locking. VE is frozen. But lists of children
		 * may change at least for init, when entered task reparents
		 * to init and when reparented task exits. If we take care
		 * of this case, we still can unlock while scanning
		 * tasklists.
		 */
		read_lock(&tasklist_lock);
		list_for_each_entry(child, &tsk->children, sibling) {
			if (child->real_parent != tsk)
				continue;
			if (child->pid != child->tgid)
				continue;
			get_task_struct(child);
			read_unlock(&tasklist_lock);

			if ((head = remember_task(child, head, ctx)) == NULL) {
				eprintk_ctx("task obj allocation failure\n");
				err = -ENOMEM;
				goto out;
			}

			read_lock(&tasklist_lock);
		}

		list_for_each_entry(child, &tsk->ptrace_children, ptrace_list) {
			if (child->real_parent != tsk)
				continue;
			if (child->pid != child->tgid)
				continue;
			get_task_struct(child);
			read_unlock(&tasklist_lock);

			if ((head = remember_task(child, head, ctx)) == NULL) {
				eprintk_ctx("task obj allocation failure\n");
				err = -ENOMEM;
				goto out;
			}

			read_lock(&tasklist_lock);
		}
		read_unlock(&tasklist_lock);
	}

	return 0;

out:
	while (!list_empty(&ctx->object_array[CPT_OBJ_TASK])) {
		struct list_head *head = ctx->object_array[CPT_OBJ_TASK].next;
		cpt_object_t *obj = list_entry(head, cpt_object_t, o_list);
		struct task_struct *tsk;

		list_del(head);
		tsk = obj->o_obj;
		put_task_struct(tsk);
		free_cpt_object(obj, ctx);
	}
	return err;
}

static int cpt_collect(struct cpt_context *ctx)
{
	int err;

	if ((err = cpt_collect_mm(ctx)) != 0)
		return err;

	if ((err = cpt_collect_sysv(ctx)) != 0)
		return err;

	if ((err = cpt_collect_files(ctx)) != 0)
		return err;

	if ((err = cpt_collect_fs(ctx)) != 0)
		return err;

	if ((err = cpt_collect_namespace(ctx)) != 0)
		return err;

	if ((err = cpt_collect_signals(ctx)) != 0)
		return err;

	if (virtinfo_notifier_call(VITYPE_SCP,
				VIRTINFO_SCP_COLLECT, ctx) & NOTIFY_FAIL)
		return -ECHRNG;

	return 0;
}

static int cpt_dump_veinfo(cpt_context_t *ctx)
{
	struct cpt_veinfo_image i;
	struct ve_struct *ve;
	struct timespec delta;
	struct ipc_namespace *ns;

	cpt_open_section(ctx, CPT_SECT_VEINFO);
	cpt_open_object(NULL, ctx);

	i.cpt_next = CPT_NULL;
	i.cpt_object = CPT_OBJ_VEINFO;
	i.cpt_hdrlen = sizeof(i);
	i.cpt_content = CPT_CONTENT_VOID;

	ve = get_exec_env();
	ns = ve->ve_ns->ipc_ns;

	if (ns->shm_ctlall > 0xFFFFFFFFU)
		i.shm_ctl_all = 0xFFFFFFFFU;
	if (ns->shm_ctlmax > 0xFFFFFFFFU)
		i.shm_ctl_max = 0xFFFFFFFFU;
	i.shm_ctl_mni = ns->shm_ctlmni;

	i.msg_ctl_max = ns->msg_ctlmax;
	i.msg_ctl_mni = ns->msg_ctlmni;
	i.msg_ctl_mnb = ns->msg_ctlmnb;

	BUILD_BUG_ON(sizeof(ns->sem_ctls) != sizeof(i.sem_ctl_arr));
	i.sem_ctl_arr[0] = ns->sem_ctls[0];
	i.sem_ctl_arr[1] = ns->sem_ctls[1];
	i.sem_ctl_arr[2] = ns->sem_ctls[2];
	i.sem_ctl_arr[3] = ns->sem_ctls[3];

	do_posix_clock_monotonic_gettime(&delta);
	_set_normalized_timespec(&delta,
			delta.tv_sec - ve->start_timespec.tv_sec,
			delta.tv_nsec - ve->start_timespec.tv_nsec);
	i.start_timespec_delta = cpt_timespec_export(&delta);
	i.start_jiffies_delta = get_jiffies_64() - ve->start_jiffies;

	ctx->write(&i, sizeof(i), ctx);
	cpt_close_object(ctx);
	cpt_close_section(ctx);
	return 0;
}

static int cpt_dump_utsname(cpt_context_t *ctx)
{
	int len;
	struct cpt_object_hdr o;
	struct ve_struct *ve;
	struct uts_namespace *ns;

	cpt_open_section(ctx, CPT_SECT_UTSNAME);

	ve = get_exec_env();
	ns = ve->ve_ns->uts_ns;

 	cpt_open_object(NULL, ctx);
	len = strlen(ns->name.nodename);
 	o.cpt_next = CPT_NULL;
	o.cpt_object = CPT_OBJ_NAME;
	o.cpt_hdrlen = sizeof(o);
	o.cpt_content = CPT_CONTENT_NAME;

	ctx->write(&o, sizeof(o), ctx);
	ctx->write(ns->name.nodename, len+1, ctx);
	ctx->align(ctx);
 	cpt_close_object(ctx);
 
 	cpt_open_object(NULL, ctx);
	len = strlen(ns->name.domainname);
 	o.cpt_next = CPT_NULL;
	o.cpt_object = CPT_OBJ_NAME;
	o.cpt_hdrlen = sizeof(o);
	o.cpt_content = CPT_CONTENT_NAME;

	ctx->write(&o, sizeof(o), ctx);
	ctx->write(ns->name.domainname, len+1, ctx);
	ctx->align(ctx);
	cpt_close_object(ctx);

	cpt_close_section(ctx);
	return 0;
}

int cpt_dump(struct cpt_context *ctx)
{
	struct ve_struct *oldenv, *env;
	struct nsproxy *old_ns;
	int err, err2 = 0;

	if (!ctx->ve_id)
		return -EINVAL;

	env = get_ve_by_id(ctx->ve_id);
	if (!env)
		return -ESRCH;

	down_read(&env->op_sem);
	err = -ESRCH;
	if (!env->is_running)
		goto out_noenv;
	if (!env->is_locked)
		goto out_noenv;

	oldenv = set_exec_env(env);
	old_ns = current->nsproxy;
	current->nsproxy = env->ve_ns;

	/* Phase 2: real checkpointing */
	err = cpt_open_dumpfile(ctx);
	if (err)
		goto out;
	
	cpt_major_hdr_out(ctx);

	if (!err)
		err = cpt_dump_veinfo(ctx);
	if (!err)
		err = cpt_dump_ubc(ctx);
	if (!err)
		err = cpt_dump_files(ctx);
	if (!err)
		err = cpt_dump_files_struct(ctx);
	if (!err)
		err = cpt_dump_fs_struct(ctx);
	/* netdevices should be dumped after dumping open files
	   as we need to restore netdevice binding to /dev/net/tun file */
	if (!err)
		err = cpt_dump_ifinfo(ctx);
	if (!err)
		err = cpt_dump_namespace(ctx);
	if (!err)
		err = cpt_dump_sighand(ctx);
	if (!err)
		err = cpt_dump_vm(ctx);
	if (!err)
		err = cpt_dump_sysvsem(ctx);
	if (!err)
		err = cpt_dump_sysvmsg(ctx);
	if (!err)
		err = cpt_dump_tasks(ctx);
	if (!err)
		err = cpt_dump_orphaned_sockets(ctx);
#if defined(CONFIG_VE_IPTABLES) && \
    (defined(CONFIG_IP_NF_CONNTRACK) || defined(CONFIG_IP_NF_CONNTRACK_MODULE))
	if (!err)
		err = cpt_dump_ip_conntrack(ctx);
#endif
	if (!err) {
		if (virtinfo_notifier_call(VITYPE_SCP,
					VIRTINFO_SCP_DUMP, ctx) & NOTIFY_FAIL)
			err = -ECHRNG;
	}
	if (!err)
		err = cpt_dump_utsname(ctx);

	if (!err)
		err = cpt_dump_tail(ctx);

	err2 = cpt_close_dumpfile(ctx);

out:
	current->nsproxy = old_ns;
	set_exec_env(oldenv);
out_noenv:
	up_read(&env->op_sem);
	put_ve(env);
	return err ? : err2;
}

int cpt_vps_suspend(struct cpt_context *ctx)
{
	struct ve_struct *oldenv, *env;
	struct nsproxy *old_ns;
	int err = 0;

	ctx->kernel_config_flags = test_kernel_config();
	cpt_object_init(ctx);

	if (!ctx->ve_id) {
		env = get_exec_env();
		if (env == get_ve0())
			return -EINVAL;
		wprintk("undefined ve_id\n");
		ctx->ve_id = env->veid;
		get_ve(env);
	} else {
		env = get_ve_by_id(ctx->ve_id);
		if (!env)
			return -ESRCH;
	}

#ifdef CONFIG_VE_IPTABLES
	ctx->iptables_mask = env->_iptables_modules;
#endif
	ctx->features = env->features;

	down_write(&env->op_sem);
	err = -ESRCH;
	if (!env->is_running)
		goto out_noenv;

	err = -EBUSY;
	if (env->is_locked)
		goto out_noenv;
	env->is_locked = 1;
	downgrade_write(&env->op_sem);

	oldenv = set_exec_env(env);
	old_ns = current->nsproxy;
	current->nsproxy = env->ve_ns;

	/* Phase 0: find and stop all the tasks */
	if ((err = vps_stop_tasks(ctx)) != 0)
		goto out;

	if ((err = cpt_suspend_network(ctx)) != 0)
		goto out_wake;

	/* At the moment all the state is frozen. We do not need to lock
	 * the state, which can be changed only if the tasks are running.
	 */

	/* Phase 1: collect task tree */
	if ((err = vps_collect_tasks(ctx)) != 0)
		goto out_wake;

	/* Phase 1': collect all the resources */
	if ((err = cpt_collect(ctx)) != 0)
		goto out;

out:
	current->nsproxy = old_ns;
	set_exec_env(oldenv);
	up_read(&env->op_sem);
	put_ve(env);
        return err;

out_noenv:
	up_write(&env->op_sem);
	put_ve(env);
	return err;

out_wake:
	read_lock(&tasklist_lock);
	wake_ve(ctx);
	read_unlock(&tasklist_lock);
	goto out;
}

static void check_unsupported_netdevices(struct cpt_context *ctx, __u32 *caps)
{
	struct net_device *dev;

	read_lock(&dev_base_lock);
	for (dev = dev_base; dev; dev = dev->next) {
		if (dev != get_exec_env()->_loopback_dev
#if defined(CONFIG_VE_ETHDEV) || defined(CONFIG_VE_ETHDEV_MODULE)
		    && !(KSYMREF(veth_open) && dev->open == KSYMREF(veth_open))
#endif
#if defined(CONFIG_VE_NETDEV) || defined(CONFIG_VE_NETDEV_MODULE)
		    && dev != get_exec_env()->_venet_dev
#endif
#if defined(CONFIG_TUN) || defined(CONFIG_TUN_MODULE)
		    && dev->open != tun_net_open
#endif
							) {
			eprintk_ctx("unsupported netdevice %s\n", dev->name);
			*caps |= (1<<CPT_UNSUPPORTED_NETDEV);
		}
	}
	read_unlock(&dev_base_lock);
}

static void check_one_process(struct cpt_context *ctx, __u32 *caps,
		unsigned int flags, struct ve_struct *env,
		struct task_struct *root, struct task_struct *p)
{
	struct namespace *ns;
	if (tsk_used_math(p))
		*caps |= flags;
#ifdef CONFIG_X86_64
	if (!(p->thread_info->flags & _TIF_IA32))
		*caps |= (1<<CPT_CPU_X86_EMT64);
#endif
#ifdef CONFIG_IA64
	if (!IS_IA32_PROCESS(task_pt_regs(p)))
		*caps |= (1<<CPT_CPU_X86_IA64);
#endif
	if (vps_child_level(root, p) >= 0) {
		if (!is_virtual_pid(virt_pid(p))) {
			eprintk_ctx("external process %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n", virt_pid(p), p->pid, p->comm);
			*caps |= (1<<CPT_EXTERNAL_PROCESS);
		}
		if (!is_virtual_pid(virt_pgid(p))) {
			eprintk_ctx("external process group %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n", virt_pgid(p), p->pid, p->comm);
			*caps |= (1<<CPT_EXTERNAL_PROCESS);
		}
		if (!is_virtual_pid(virt_sid(p))) {
			eprintk_ctx("external process session %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n", virt_sid(p), p->pid, p->comm);
			*caps |= (1<<CPT_EXTERNAL_PROCESS);
		}
	} else {
		eprintk_ctx("foreign process %d/%d(%s) inside CT (e.g. vzctl enter or vzctl exec).\n", virt_pid(p), p->pid, p->comm);
		*caps |= (1<<CPT_EXTERNAL_PROCESS);
	}
	ns = get_task_mnt_ns(p);
	if (ns) {
		if (ns != current->nsproxy->namespace) {
			eprintk_ctx("namespaces are not supported: process %d/%d(%s)\n", virt_pid(p), p->pid, p->comm);
			*caps |= (1<<CPT_NAMESPACES);
		}
		put_namespace(ns);
	}
	if (p->policy != SCHED_NORMAL) {
		eprintk_ctx("scheduler policy is not supported %d/%d(%s)\n", virt_pid(p), p->pid, p->comm);
		*caps |= (1<<CPT_SCHEDULER_POLICY);
	}
	if (p->parent) {
		if (p->parent != p->real_parent &&
				VE_TASK_INFO(p->parent)->owner_env != env) {
			eprintk_ctx("task %d/%d(%s) is ptraced from host system\n", p->pid, virt_pid(p), p->comm);
			*caps |= (1<<CPT_PTRACED_FROM_VE0);
		}
	}
	if (cpt_check_unsupported(p, ctx)) {
		*caps |= (1<<CPT_UNSUPPORTED_MISC);
	}
}

static void check_unsupported_mounts(struct cpt_context *ctx, __u32 *caps,
		struct ve_struct *env, struct namespace *n, char *path_buf)
{
	struct list_head *p;
	char *path;

	down_read(&namespace_sem);
	list_for_each(p, &n->list) {
		struct vfsmount *mnt = list_entry(p, struct vfsmount, mnt_list);

		spin_lock(&dcache_lock);
		path = __d_path(mnt->mnt_root, mnt,
				env->fs_root, env->fs_rootmnt,
				path_buf, PAGE_SIZE);
		spin_unlock(&dcache_lock);
		if (IS_ERR(path))
			continue;

		if (check_one_vfsmount(mnt)) {
			eprintk_ctx("Unsupported filesystem %s\n", mnt->mnt_sb->s_type->name);
			*caps |= (1<<CPT_UNSUPPORTED_FSTYPE);
		}
	}
	up_read(&namespace_sem);
}

int cpt_vps_caps(struct cpt_context *ctx, __u32 *caps)
{
	struct task_struct *p;
	struct task_struct *root;
	struct ve_struct *env;
	struct ve_struct *old_env;
	struct nsproxy *old_ns;
	struct namespace *n;
	int err;
	unsigned int flags = test_cpu_caps();

	if (!ctx->ve_id)
		return -EINVAL;

	env = get_ve_by_id(ctx->ve_id);
	if (env == NULL)
		return -ESRCH;

	down_read(&env->op_sem);
	err = -ESRCH;
	if (!env->is_running) {
		eprintk_ctx("CT is not running\n");
		goto out_noenv;
	}

	err = -EBUSY;
	if (env->is_locked) {
		eprintk_ctx("CT is locked\n");
		goto out_noenv;
	}

	*caps = flags & (1<<CPT_CPU_X86_CMOV);
	flags &= ~((1<<CPT_CPU_X86_EMT64)|(1<<CPT_CPU_X86_IA64));

	old_env = set_exec_env(env);
	old_ns = current->nsproxy;
	current->nsproxy = env->ve_ns;

	check_unsupported_netdevices(ctx, caps);

	read_lock(&tasklist_lock);
	root = find_task_by_pid_ve(1);
	if (!root) {
		read_unlock(&tasklist_lock);
		eprintk_ctx("cannot find ve init\n");
		err = -ESRCH;
		goto out;
	}
	get_task_struct(root);
	for (p = __first_task_ve(env); p != NULL ; p = __next_task_ve(env, p))
		check_one_process(ctx, caps, flags, env, root, p);
	read_unlock(&tasklist_lock);

	n = get_task_mnt_ns(root);
	if (n) {
		char *path_buf;

		path_buf = (char *) __get_free_page(GFP_KERNEL);
		if (!path_buf) {
			put_namespace(n);
			err = -ENOMEM;
			goto out_root;
		}

		check_unsupported_mounts(ctx, caps, env, n, path_buf);

		free_page((unsigned long) path_buf);
		put_namespace(n);
	}

	err = 0;

out_root:
	put_task_struct(root);
out:
	current->nsproxy = old_ns;
	set_exec_env(old_env);
out_noenv:
	up_read(&env->op_sem);
	put_ve(env);

	return err;
}
