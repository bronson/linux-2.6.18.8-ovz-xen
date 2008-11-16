/*
 *
 *  kernel/cpt/rst_undump.c
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
#include <linux/namespace.h>
#include <linux/personality.h>
#include <linux/binfmts.h>
#include <linux/smp_lock.h>
#include <linux/ve_proto.h>
#include <linux/virtinfo.h>
#include <linux/virtinfoscp.h>
#include <linux/compat.h>
#include <linux/vzcalluser.h>
#include <ub/beancounter.h>
#ifdef CONFIG_X86
#include <asm/desc.h>
#endif
#include <asm/unistd.h>
#include <linux/nsproxy.h>

#include "cpt_obj.h"
#include "cpt_context.h"
#include "cpt_files.h"
#include "cpt_mm.h"
#include "cpt_process.h"
#include "cpt_socket.h"
#include "cpt_net.h"
#include "cpt_ubc.h"
#include "cpt_kernel.h"

static int rst_utsname(cpt_context_t *ctx);


struct thr_context {
	struct completion init_complete;
	struct completion task_done;
	int error;
	struct cpt_context *ctx;
	cpt_object_t	*tobj;
};

static int rst_clone_children(cpt_object_t *obj, struct cpt_context *ctx);

static int vps_rst_veinfo(struct cpt_context *ctx)
{
	int err;
	struct cpt_veinfo_image *i;
	struct ve_struct *ve;
	struct timespec delta;
	loff_t start, end;
	struct ipc_namespace *ns;

	err = rst_get_section(CPT_SECT_VEINFO, ctx, &start, &end);
	if (err)
		goto out;

	i = cpt_get_buf(ctx);
	err = rst_get_object(CPT_OBJ_VEINFO, start, i, ctx);
	if (err)
		goto out_rel;

	ve = get_exec_env();
	ns = ve->ve_ns->ipc_ns;

	/* Damn. Fatal mistake, these two values are size_t! */
	ns->shm_ctlall = i->shm_ctl_all ? : 0xFFFFFFFFU;
	ns->shm_ctlmax = i->shm_ctl_max ? : 0xFFFFFFFFU;
	ns->shm_ctlmni = i->shm_ctl_mni;

	ns->msg_ctlmax = i->msg_ctl_max;
	ns->msg_ctlmni = i->msg_ctl_mni;
	ns->msg_ctlmnb = i->msg_ctl_mnb;

	BUILD_BUG_ON(sizeof(ns->sem_ctls) != sizeof(i->sem_ctl_arr));
	ns->sem_ctls[0] = i->sem_ctl_arr[0];
	ns->sem_ctls[1] = i->sem_ctl_arr[1];
	ns->sem_ctls[2] = i->sem_ctl_arr[2];
	ns->sem_ctls[3] = i->sem_ctl_arr[3];

	cpt_timespec_import(&delta, i->start_timespec_delta);
	_set_normalized_timespec(&ve->start_timespec,
			ve->start_timespec.tv_sec - delta.tv_sec,
			ve->start_timespec.tv_nsec - delta.tv_nsec);
	ve->start_jiffies -= i->start_jiffies_delta;
	// // FIXME: what???
	// // ve->start_cycles -= (s64)i->start_jiffies_delta * cycles_per_jiffy;

	err = 0;
out_rel:
	cpt_release_buf(ctx);
out:
	return err;
}

static int vps_rst_reparent_root(cpt_object_t *obj, struct cpt_context *ctx)
{
	int err;
	struct env_create_param3 param;

	do_posix_clock_monotonic_gettime(&ctx->cpt_monotonic_time);
	do_gettimespec(&ctx->delta_time);

	_set_normalized_timespec(&ctx->delta_time,
				 ctx->delta_time.tv_sec - ctx->start_time.tv_sec,
				 ctx->delta_time.tv_nsec - ctx->start_time.tv_nsec);
	ctx->delta_nsec = (s64)ctx->delta_time.tv_sec*NSEC_PER_SEC + ctx->delta_time.tv_nsec;
	if (ctx->delta_nsec < 0) {
		wprintk_ctx("Wall time is behind source by %Ld ns, "
			    "time sensitive applications can misbehave\n", (long long)-ctx->delta_nsec);
	}

        _set_normalized_timespec(&ctx->cpt_monotonic_time,
                                 ctx->cpt_monotonic_time.tv_sec - ctx->delta_time.tv_sec,
                                 ctx->cpt_monotonic_time.tv_nsec - ctx->delta_time.tv_nsec);

	memset(&param, 0, sizeof(param));
	param.iptables_mask = ctx->iptables_mask;
	param.feature_mask = ctx->features;

	/* feature_mask is set as required - pretend we know everything */
	param.known_features = (ctx->image_version < CPT_VERSION_18) ?
		VE_FEATURES_OLD : ~(__u64)0;

	err = real_env_create(ctx->ve_id, VE_CREATE|VE_LOCK|VE_EXCLUSIVE, 2,
			&param, sizeof(param));
	if (err < 0)
		eprintk_ctx("real_env_create: %d\n", err);

	get_exec_env()->jiffies_fixup =
		(ctx->delta_time.tv_sec < 0 ?
		 0 : timespec_to_jiffies(&ctx->delta_time)) -
		(unsigned long)(get_jiffies_64() - ctx->virt_jiffies64);
	dprintk_ctx("JFixup %ld %Ld\n", get_exec_env()->jiffies_fixup,
		    (long long)ctx->delta_nsec);
	return err < 0 ? err : 0;
}

static int hook(void *arg)
{
	struct thr_context *thr_ctx = arg;
	struct cpt_context *ctx;
	cpt_object_t *tobj;
	struct cpt_task_image *ti;
	int err = 0;
	int exiting = 0;

	current->state = TASK_UNINTERRUPTIBLE;
	complete(&thr_ctx->init_complete);
	schedule();

	ctx = thr_ctx->ctx;
	tobj = thr_ctx->tobj;
	ti = tobj->o_image;

	current->fs->umask = 0;

	if (ti->cpt_pid == 1) {
#ifdef CONFIG_USER_RESOURCE
		struct user_beancounter *bc;
#endif

		err = vps_rst_reparent_root(tobj, ctx);

		if (err) {
			rst_report_error(err, ctx);
			goto out;
		}

		memcpy(&cap_bset, &ti->cpt_ecap, sizeof(kernel_cap_t));

		if (ctx->statusfile) {
			fput(ctx->statusfile);
			ctx->statusfile = NULL;
		}

		if (ctx->lockfile) {
			char b;
			mm_segment_t oldfs;
			err = -EINVAL;

			oldfs = get_fs(); set_fs(KERNEL_DS);
			if (ctx->lockfile->f_op && ctx->lockfile->f_op->read)
				err = ctx->lockfile->f_op->read(ctx->lockfile, &b, 1, &ctx->lockfile->f_pos);
			set_fs(oldfs);
			fput(ctx->lockfile);
			ctx->lockfile = NULL;
		}

		if (err) {
			eprintk_ctx("CPT: lock fd is closed incorrectly: %d\n", err);
			goto out;
		}
		err = vps_rst_veinfo(ctx);
		if (err) {
			eprintk_ctx("rst_veinfo: %d\n", err);
			goto out;
		}

		err = rst_utsname(ctx);
		if (err) {
			eprintk_ctx("rst_utsname: %d\n", err);
			goto out;
		}

		err = rst_root_namespace(ctx);
		if (err) {
			eprintk_ctx("rst_namespace: %d\n", err);
			goto out;
		}

		if ((err = rst_restore_net(ctx)) != 0) {
			eprintk_ctx("rst_restore_net: %d\n", err);
			goto out;
		}

		err = rst_sockets(ctx);
		if (err) {
			eprintk_ctx("rst_sockets: %d\n", err);
			goto out;
		}
		err = rst_sysv_ipc(ctx);
		if (err) {
			eprintk_ctx("rst_sysv_ipc: %d\n", err);
			goto out;
		}
#ifdef CONFIG_USER_RESOURCE
		bc = get_exec_ub();
		set_one_ubparm_to_max(bc->ub_parms, UB_KMEMSIZE);
		set_one_ubparm_to_max(bc->ub_parms, UB_NUMPROC);
		set_one_ubparm_to_max(bc->ub_parms, UB_NUMFILE);
		set_one_ubparm_to_max(bc->ub_parms, UB_DCACHESIZE);
#endif
	}

	do {
		if (current->user->uid != ti->cpt_user) {
			struct user_struct *u = alloc_uid(ti->cpt_user);
			if (!u) {
				eprintk_ctx("alloc_user\n");
			} else {
				switch_uid(u);
			}
		}
	} while (0);

	if ((err = rst_mm_complete(ti, ctx)) != 0) {
		eprintk_ctx("rst_mm: %d\n", err);
		goto out;
	}

	if ((err = rst_files_complete(ti, ctx)) != 0) {
		eprintk_ctx("rst_files: %d\n", err);
		goto out;
	}

	if ((err = rst_fs_complete(ti, ctx)) != 0) {
		eprintk_ctx("rst_fs: %d\n", err);
		goto out;
	}

	if ((err = rst_semundo_complete(ti, ctx)) != 0) {
		eprintk_ctx("rst_semundo: %d\n", err);
		goto out;
	}

	if ((err = rst_signal_complete(ti, &exiting, ctx)) != 0) {
		eprintk_ctx("rst_signal: %d\n", err);
		goto out;
	}

	if (ti->cpt_personality != 0)
		__set_personality(ti->cpt_personality);

#ifdef CONFIG_X86_64
	/* 32bit app from 32bit OS, won't have PER_LINUX32 set... :/ */
	if (!ti->cpt_64bit)
		__set_personality(PER_LINUX32);
#endif

	current->set_child_tid = NULL;
	current->clear_child_tid = NULL;
	current->flags &= ~(PF_FORKNOEXEC|PF_SUPERPRIV);
	current->flags |= ti->cpt_flags&(PF_FORKNOEXEC|PF_SUPERPRIV);
	current->exit_code = ti->cpt_exit_code;
	current->pdeath_signal = ti->cpt_pdeath_signal;

	if (ti->cpt_restart.fn != CPT_RBL_0) {
		if (ti->cpt_restart.fn != CPT_RBL_NANOSLEEP
		    && ti->cpt_restart.fn != CPT_RBL_COMPAT_NANOSLEEP
		    ) {
			eprintk_ctx("unknown restart block\n");
		} else {
			ktime_t e;

			e.tv64 = 0;

			current->thread_info->restart_block.fn = nanosleep_restart;
#if defined(CONFIG_X86_64) && defined(CONFIG_COMPAT)
			if (!ti->cpt_64bit)
				current->thread_info->restart_block.fn = compat_nanosleep_restart;
#endif
			if (ctx->image_version >= CPT_VERSION_9)
				e = ktime_add_ns(e, ti->cpt_restart.arg0);
			else
				e = ktime_add_ns(e, ti->cpt_restart.arg0*TICK_NSEC);
			if (e.tv64 < 0)
				e.tv64 = TICK_NSEC;
			e = ktime_add(e, timespec_to_ktime(ctx->cpt_monotonic_time));
			current->thread_info->restart_block.arg0 = e.tv64 & 0xFFFFFFFF;
			current->thread_info->restart_block.arg1 = e.tv64 >> 32;
			if (ctx->image_version >= CPT_VERSION_9) {
				current->thread_info->restart_block.arg2 = ti->cpt_restart.arg2;
				current->thread_info->restart_block.arg3 = ti->cpt_restart.arg3;	
			} else {
				current->thread_info->restart_block.arg2 = ti->cpt_restart.arg1;
				current->thread_info->restart_block.arg3 = CLOCK_MONOTONIC;	
			}
		}
	}

	if (thread_group_leader(current)) {
		current->signal->it_real_incr.tv64 = 0;
		if (ctx->image_version >= CPT_VERSION_9) {
			current->signal->it_real_incr =
			ktime_add_ns(current->signal->it_real_incr, ti->cpt_it_real_incr);
		} else {
			current->signal->it_real_incr =
			ktime_add_ns(current->signal->it_real_incr, ti->cpt_it_real_incr*TICK_NSEC);
		}
		current->signal->it_prof_incr = ti->cpt_it_prof_incr;
		current->signal->it_virt_incr = ti->cpt_it_virt_incr; 
		current->signal->it_prof_expires = ti->cpt_it_prof_value;
		current->signal->it_virt_expires = ti->cpt_it_virt_value;
	}

	err = rst_clone_children(tobj, ctx);
	if (err) {
		eprintk_ctx("rst_clone_children\n");
		goto out;
	}

	if (exiting)
		current->signal->flags |= SIGNAL_GROUP_EXIT;

	if (ti->cpt_pid == 1) {
		if ((err = rst_process_linkage(ctx)) != 0) {
			eprintk_ctx("rst_process_linkage: %d\n", err);
			goto out;
		}
		if ((err = rst_do_filejobs(ctx)) != 0) {
			eprintk_ctx("rst_do_filejobs: %d\n", err);
			goto out;
		}
		if ((err = rst_eventpoll(ctx)) != 0) {
			eprintk_ctx("rst_eventpoll: %d\n", err);
			goto out;
		}
#ifdef CONFIG_INOTIFY_USER
		if ((err = rst_inotify(ctx)) != 0) {
			eprintk_ctx("rst_inotify: %d\n", err);
			goto out;
		}
#endif
		if ((err = rst_sockets_complete(ctx)) != 0) {
			eprintk_ctx("rst_sockets_complete: %d\n", err);
			goto out;
		}
		if ((err = rst_stray_files(ctx)) != 0) {
			eprintk_ctx("rst_stray_files: %d\n", err);
			goto out;
		}
		if ((err = rst_posix_locks(ctx)) != 0) {
			eprintk_ctx("rst_posix_locks: %d\n", err);
			goto out;
		}
		if ((err = rst_tty_jobcontrol(ctx)) != 0) {
			eprintk_ctx("rst_tty_jobcontrol: %d\n", err);
			goto out;
		}
		if ((err = rst_restore_fs(ctx)) != 0) {
			eprintk_ctx("rst_restore_fs: %d\n", err);
			goto out;
		}
		if (virtinfo_notifier_call(VITYPE_SCP,
				VIRTINFO_SCP_RESTORE, ctx) & NOTIFY_FAIL) {
			err = -ECHRNG;
			eprintk_ctx("scp_restore failed\n");
			goto out;
		}
	}

out:
	thr_ctx->error = err;
	complete(&thr_ctx->task_done);

	if (!err && (ti->cpt_state & (EXIT_ZOMBIE|EXIT_DEAD))) {
		current->flags |= PF_EXIT_RESTART;
		do_exit(ti->cpt_exit_code);
	} else {
		__set_current_state(TASK_UNINTERRUPTIBLE);
	}

	schedule();

	dprintk_ctx("leaked through %d/%d %p\n", current->pid, virt_pid(current), current->mm);

	module_put(THIS_MODULE);
	complete_and_exit(NULL, 0);
	return 0;
}

#if 0
static void set_task_ubs(struct cpt_task_image *ti, struct cpt_context *ctx)
{
	struct task_beancounter *tbc;

	tbc = task_bc(current);

	put_beancounter(tbc->fork_sub);
	tbc->fork_sub = rst_lookup_ubc(ti->cpt_task_ub, ctx);
	if (ti->cpt_mm_ub != CPT_NULL) {
		put_beancounter(tbc->exec_ub);
		tbc->exec_ub = rst_lookup_ubc(ti->cpt_mm_ub, ctx);
	}
}
#endif

static int create_root_task(cpt_object_t *obj, struct cpt_context *ctx,
		struct thr_context *thr_ctx)
{
	struct task_struct *tsk;
	int pid;

	thr_ctx->ctx = ctx;
	thr_ctx->error = 0;
	init_completion(&thr_ctx->init_complete);
	init_completion(&thr_ctx->task_done);
#if 0
	set_task_ubs(obj->o_image, ctx);
#endif

	pid = local_kernel_thread(hook, thr_ctx, 0, 0);
	if (pid < 0)
		return pid;
	read_lock(&tasklist_lock);
	tsk = find_task_by_pid_ve(pid);
	if (tsk)
		get_task_struct(tsk);
	read_unlock(&tasklist_lock);
	if (tsk == NULL)
		return -ESRCH;
	cpt_obj_setobj(obj, tsk, ctx);
	thr_ctx->tobj = obj;
	return 0;
}

static int rst_basic_init_task(cpt_object_t *obj, struct cpt_context *ctx)
{
	struct task_struct *tsk = obj->o_obj;
	struct cpt_task_image *ti = obj->o_image;

	memcpy(tsk->comm, ti->cpt_comm, sizeof(tsk->comm));
	rst_mm_basic(obj, ti, ctx);
	return 0;
}

static int make_baby(cpt_object_t *cobj,
		     struct cpt_task_image *pi,
		     struct cpt_context *ctx)
{
	unsigned long flags;
	struct cpt_task_image *ci = cobj->o_image;
	struct thr_context thr_ctx;
	struct task_struct *tsk;
	pid_t pid;
	struct fs_struct *tfs = NULL;

	flags = rst_mm_flag(ci, ctx) | rst_files_flag(ci, ctx)
		| rst_signal_flag(ci, ctx) | rst_semundo_flag(ci, ctx);
	if (ci->cpt_rppid != pi->cpt_pid) {
		flags |= CLONE_THREAD|CLONE_PARENT;
		if (ci->cpt_signal != pi->cpt_signal ||
		    !(flags&CLONE_SIGHAND) ||
		    (!(flags&CLONE_VM) && pi->cpt_mm != CPT_NULL)) {
			eprintk_ctx("something is wrong with threads: %d %d %d %Ld %Ld %08lx\n",
			       (int)ci->cpt_pid, (int)ci->cpt_rppid, (int)pi->cpt_pid,
			       (long long)ci->cpt_signal, (long long)pi->cpt_signal, flags
			       );
			return -EINVAL;
		}
	}

	thr_ctx.ctx = ctx;
	thr_ctx.error = 0;
	init_completion(&thr_ctx.init_complete);
	init_completion(&thr_ctx.task_done);
	thr_ctx.tobj = cobj;

#if 0
	set_task_ubs(ci, ctx);
#endif

	if (current->fs == NULL) {
		tfs = get_exec_env()->init_entry->fs;
		if (tfs == NULL)
			return -EINVAL;
		atomic_inc(&tfs->count);
		current->fs = tfs;
	}
	pid = local_kernel_thread(hook, &thr_ctx, flags, ci->cpt_pid);
	if (tfs) {
		current->fs = NULL;
		atomic_dec(&tfs->count);
	}
	if (pid < 0)
		return pid;

	read_lock(&tasklist_lock);
	tsk = find_task_by_pid_ve(pid);
	if (tsk)
		get_task_struct(tsk);
	read_unlock(&tasklist_lock);
	if (tsk == NULL)
		return -ESRCH;
	cpt_obj_setobj(cobj, tsk, ctx);
	thr_ctx.tobj = cobj;
	wait_for_completion(&thr_ctx.init_complete);
	wait_task_inactive(cobj->o_obj);
	rst_basic_init_task(cobj, ctx);

	/* clone() increases group_stop_count if it was not zero and
	 * CLONE_THREAD was asked. Undo.
	 */
	if (current->signal->group_stop_count && (flags & CLONE_THREAD)) {
		if (tsk->signal != current->signal) BUG();
		current->signal->group_stop_count--;
	}

	wake_up_process(tsk);
	wait_for_completion(&thr_ctx.task_done);
	wait_task_inactive(tsk);

	return thr_ctx.error;
}

static int rst_clone_children(cpt_object_t *obj, struct cpt_context *ctx)
{
	int err = 0;
	struct cpt_task_image *ti = obj->o_image;
	cpt_object_t *cobj;

	for_each_object(cobj, CPT_OBJ_TASK) {
		struct cpt_task_image *ci = cobj->o_image;
		if (cobj == obj)
			continue;
		if ((ci->cpt_rppid == ti->cpt_pid && ci->cpt_tgid == ci->cpt_pid) ||
		    (ci->cpt_leader == ti->cpt_pid &&
		     ci->cpt_tgid != ci->cpt_pid && ci->cpt_pid != 1)) {
			err = make_baby(cobj, ti, ctx);
			if (err) {
				eprintk_ctx("make_baby: %d\n", err);
				return err;
			}
		}
	}
	return 0;
}

static int read_task_images(struct cpt_context *ctx)
{
	int err;
	loff_t start, end;

	err = rst_get_section(CPT_SECT_TASKS, ctx, &start, &end);
	if (err)
		return err;

	while (start < end) {
		cpt_object_t *obj;
		struct cpt_task_image *ti = cpt_get_buf(ctx);

		err = rst_get_object(CPT_OBJ_TASK, start, ti, ctx);
		if (err) {
			cpt_release_buf(ctx);
			return err;
		}
		if (ti->cpt_pid != 1 && !__is_virtual_pid(ti->cpt_pid)) {
			eprintk_ctx("BUG: pid %d is not virtual\n", ti->cpt_pid);
			cpt_release_buf(ctx);
			return -EINVAL;
		}
		obj = alloc_cpt_object(GFP_KERNEL, ctx);
		cpt_obj_setpos(obj, start, ctx);
		intern_cpt_object(CPT_OBJ_TASK, obj, ctx);
		obj->o_image = kmalloc(ti->cpt_next, GFP_KERNEL);
		if (obj->o_image == NULL) {
			cpt_release_buf(ctx);
			return -ENOMEM;
		}
		memcpy(obj->o_image, ti, sizeof(*ti));
		err = ctx->pread(obj->o_image + sizeof(*ti),
				 ti->cpt_next - sizeof(*ti), ctx, start + sizeof(*ti));
		cpt_release_buf(ctx);
		if (err)
			return err;
		start += ti->cpt_next;
	}
	return 0;
}


static int vps_rst_restore_tree(struct cpt_context *ctx)
{
	int err;
	cpt_object_t *obj;
	struct thr_context thr_ctx_root;

	err = read_task_images(ctx);
	if (err)
		return err;

	err = rst_undump_ubc(ctx);
	if (err)
		return err;

	if (virtinfo_notifier_call(VITYPE_SCP,
				VIRTINFO_SCP_RSTCHECK, ctx) & NOTIFY_FAIL)
		return -ECHRNG;
#ifdef CONFIG_VZ_CHECKPOINT_LAZY
	err = rst_setup_pagein(ctx);
	if (err)
		return err;
#endif
	for_each_object(obj, CPT_OBJ_TASK) {
		err = create_root_task(obj, ctx, &thr_ctx_root);
		if (err)
			return err;

		wait_for_completion(&thr_ctx_root.init_complete);
		wait_task_inactive(obj->o_obj);
		rst_basic_init_task(obj, ctx);

		wake_up_process(obj->o_obj);
		wait_for_completion(&thr_ctx_root.task_done);
		wait_task_inactive(obj->o_obj);
		err = thr_ctx_root.error;
		if (err)
			return err;
		break;
	}

	return err;
}


int vps_rst_undump(struct cpt_context *ctx)
{
	int err;
	unsigned long umask;

	err = rst_open_dumpfile(ctx);
	if (err)
		return err;

	if (ctx->tasks64) {
#if defined(CONFIG_IA64)
		if (ctx->image_arch != CPT_OS_ARCH_IA64)
#elif defined(CONFIG_X86_64)
		if (ctx->image_arch != CPT_OS_ARCH_EMT64)
#else
		if (1)
#endif
		{
			eprintk_ctx("Cannot restore 64 bit container on this architecture\n");
			return -EINVAL;
		}
	}

	umask = current->fs->umask;
	current->fs->umask = 0;

#ifdef CONFIG_VZ_CHECKPOINT_LAZY
	err = rst_setup_pagein(ctx);
#endif

	if (err == 0)
		err = vps_rst_restore_tree(ctx);

	if (err == 0)
		err = rst_restore_process(ctx);

	if (err)
		virtinfo_notifier_call(VITYPE_SCP,
				VIRTINFO_SCP_RSTFAIL, ctx);

	current->fs->umask = umask;

        return err;
}

static int rst_unlock_ve(struct cpt_context *ctx)
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

int rst_resume(struct cpt_context *ctx)
{
	cpt_object_t *obj;
	int err = 0;
#ifdef CONFIG_USER_RESOURCE
	struct user_beancounter *bc;
#endif

	for_each_object(obj, CPT_OBJ_FILE) {
		struct file *file = obj->o_obj;

		fput(file);
	}

#ifdef CONFIG_USER_RESOURCE
	bc = get_beancounter_byuid(ctx->ve_id, 0);
	BUG_ON(!bc);
	copy_one_ubparm(ctx->saved_ubc, bc->ub_parms, UB_KMEMSIZE);
	copy_one_ubparm(ctx->saved_ubc, bc->ub_parms, UB_NUMPROC);
	copy_one_ubparm(ctx->saved_ubc, bc->ub_parms, UB_NUMFILE);
	copy_one_ubparm(ctx->saved_ubc, bc->ub_parms, UB_DCACHESIZE);
	put_beancounter(bc);
#endif

	rst_resume_network(ctx);

	for_each_object(obj, CPT_OBJ_TASK) {
		struct task_struct *tsk = obj->o_obj;
		struct cpt_task_image *ti = obj->o_image;

		if (!tsk)
			continue;

		if (ti->cpt_state == TASK_UNINTERRUPTIBLE) {
			dprintk_ctx("task %d/%d(%s) is started\n", virt_pid(tsk), tsk->pid, tsk->comm);

			/* Weird... If a signal is sent to stopped task,
			 * nobody makes recalc_sigpending(). We have to do
			 * this by hands after wake_up_process().
			 * if we did this before a signal could arrive before
			 * wake_up_process() and stall.
			 */
			spin_lock_irq(&tsk->sighand->siglock);
			if (!signal_pending(tsk))
				recalc_sigpending_tsk(tsk);
			spin_unlock_irq(&tsk->sighand->siglock);

			wake_up_process(tsk);
		} else {
			if (ti->cpt_state == TASK_STOPPED ||
			    ti->cpt_state == TASK_TRACED) {
				set_task_state(tsk, ti->cpt_state);
			}
		}
		put_task_struct(tsk);
	}

	rst_unlock_ve(ctx);

#ifdef CONFIG_VZ_CHECKPOINT_LAZY
	rst_complete_pagein(ctx, 0);
#endif

	rst_finish_ubc(ctx);
	cpt_object_destroy(ctx);

        return err;
}

int rst_kill(struct cpt_context *ctx)
{
	cpt_object_t *obj;
	int err = 0;

	for_each_object(obj, CPT_OBJ_FILE) {
		struct file *file = obj->o_obj;

		fput(file);
	}

	for_each_object(obj, CPT_OBJ_TASK) {
		struct task_struct *tsk = obj->o_obj;

		if (tsk == NULL)
			continue;

		if (tsk->exit_state == 0) {
			send_sig(SIGKILL, tsk, 1);

			spin_lock_irq(&tsk->sighand->siglock);
			sigfillset(&tsk->blocked);
			sigdelsetmask(&tsk->blocked, sigmask(SIGKILL));
			set_tsk_thread_flag(tsk, TIF_SIGPENDING);
			clear_tsk_thread_flag(tsk, TIF_FREEZE);
			if (tsk->flags & PF_FROZEN)
				tsk->flags &= ~PF_FROZEN;
			spin_unlock_irq(&tsk->sighand->siglock);

			wake_up_process(tsk);
		}

		put_task_struct(tsk);
	}

#ifdef CONFIG_VZ_CHECKPOINT_LAZY
	rst_complete_pagein(ctx, 1);
#endif

	rst_finish_ubc(ctx);
	cpt_object_destroy(ctx);

        return err;
}

static int rst_utsname(cpt_context_t *ctx)
{
	int err;
	loff_t sec = ctx->sections[CPT_SECT_UTSNAME];
	loff_t endsec;
	struct cpt_section_hdr h;
	struct cpt_object_hdr o;
	struct ve_struct *ve;
	struct uts_namespace *ns;
	int i;

	if (sec == CPT_NULL)
		return 0;

	err = ctx->pread(&h, sizeof(h), ctx, sec);
	if (err)
		return err;
	if (h.cpt_section != CPT_SECT_UTSNAME || h.cpt_hdrlen < sizeof(h))
		return -EINVAL;

	ve = get_exec_env();
	ns = ve->ve_ns->uts_ns;

	i = 0;
	endsec = sec + h.cpt_next;
	sec += h.cpt_hdrlen;
	while (sec < endsec) {
		int len;
		char *ptr;
		err = rst_get_object(CPT_OBJ_NAME, sec, &o, ctx);
		if (err)
			return err;
		len = o.cpt_next - o.cpt_hdrlen;
		if (len > __NEW_UTS_LEN+1)
			return -ENAMETOOLONG;
		switch (i) {
		case 0:
			ptr = ns->name.nodename; break;
		case 1:
			ptr = ns->name.domainname; break;
		default:
			return -EINVAL;
		}
		err = ctx->pread(ptr, len, ctx, sec+o.cpt_hdrlen);
		if (err)
			return err;
		i++;
		sec += o.cpt_next;
	}

	return 0;
}
