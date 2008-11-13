/*
 *  kernel/ve/veowner.c
 *
 *  Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#include <linux/sched.h>
#include <linux/ve.h>
#include <linux/ve_proto.h>
#include <linux/ipc.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/inetdevice.h>
#include <asm/system.h>
#include <asm/io.h>

#include <net/tcp.h>

void prepare_ve0_process(struct task_struct *tsk)
{
	set_virt_pid(tsk, tsk->pid);
	set_virt_tgid(tsk, tsk->tgid);
	if (tsk->signal) {
		set_virt_pgid(tsk, tsk->signal->pgrp);
		set_virt_sid(tsk, tsk->signal->session);
	}
	VE_TASK_INFO(tsk)->exec_env = get_ve0();
	VE_TASK_INFO(tsk)->owner_env = get_ve0();
	VE_TASK_INFO(tsk)->sleep_time = 0;
	VE_TASK_INFO(tsk)->wakeup_stamp = 0;
	VE_TASK_INFO(tsk)->sched_time = 0;
	seqcount_init(&VE_TASK_INFO(tsk)->wakeup_lock);

	if (tsk->pid) {
		list_add_rcu(&tsk->ve_task_info.vetask_list,
				&get_ve0()->vetask_lh);
		atomic_inc(&get_ve0()->pcounter);
	}
}

#ifdef CONFIG_NET
void prepare_ve0_loopback(void)
{
	get_ve0()->_loopback_dev = &loopback_dev;
}
#endif

/*
 * ------------------------------------------------------------------------
 * proc entries
 * ------------------------------------------------------------------------
 */

#ifdef CONFIG_PROC_FS
static void proc_move(struct proc_dir_entry *ddir,
		struct proc_dir_entry *sdir,
		const char *name)
{
	struct proc_dir_entry **p, *q;
	int len;

	len = strlen(name);
	for (p = &sdir->subdir, q = *p; q != NULL; p = &q->next, q = *p)
		if (proc_match(len, name, q))
			break;
	if (q == NULL)
		return;
	*p = q->next;
	q->parent = ddir;
	q->next = ddir->subdir;
	ddir->subdir = q;
	if (S_ISDIR(q->mode)) {
		sdir->nlink--;
		ddir->nlink++;
	}
}
static void prepare_proc_misc(void)
{
	static char *table[] = {
		"loadavg",
		"uptime",
		"meminfo",
		"version",
		"stat",
		"filesystems",
		"locks",
		"swaps",
		"mounts",
		"net",
		"cpuinfo",
		"sysvipc",
		"sys",
		"fs",
		"vz",
		"cmdline",
		"vmstat",
		"modules",
		"devices",
		NULL,
	};
	char **p;

	for (p = table; *p != NULL; p++)
		proc_move(&proc_root, ve0.proc_root, *p);
}
int prepare_proc(void)
{
	struct ve_struct *envid;
	struct proc_dir_entry *de;
	struct proc_dir_entry *ve_root;

	envid = set_exec_env(&ve0);
	ve_root = ve0.proc_root->subdir;
	/* move the whole tree to be visible in VE0 only */
	ve0.proc_root->subdir = proc_root.subdir;
	ve0.proc_root->nlink += proc_root.nlink - 2;
	for (de = ve0.proc_root->subdir; de->next != NULL; de = de->next)
		de->parent = ve0.proc_root;
	de->parent = ve0.proc_root;
	de->next = ve_root;

	/* move back into the global scope some specific entries */
	proc_root.subdir = NULL;
	proc_root.nlink = 2;
	prepare_proc_misc();
	proc_net = proc_mkdir("net", ve0.proc_root);
	proc_net_stat = proc_mkdir("stat", proc_net);
	proc_mkdir("vz", NULL);
#ifdef CONFIG_SYSVIPC
	proc_mkdir("sysvipc", NULL);
#endif
	proc_root_fs = proc_mkdir("fs", NULL);
	/* XXX proc_tty_init(); */

	/* XXX process inodes */

	(void)set_exec_env(envid);

	(void)create_proc_glob_entry("vz", S_IFDIR|S_IRUGO|S_IXUGO, NULL);
	return 0;
}

static struct proc_dir_entry ve0_proc_root = {
	.name = "/proc",
	.namelen = 5,
	.mode = S_IFDIR | S_IRUGO | S_IXUGO,
	.nlink = 2
};

void prepare_ve0_proc_root(void)
{
	ve0.proc_root = &ve0_proc_root;
}
#endif

/*
 * ------------------------------------------------------------------------
 * Virtualized sysctl
 * ------------------------------------------------------------------------
 */
extern int ve_area_access_check;
#ifdef CONFIG_INET
static ctl_table vz_ipv4_route_table[] = {
	{
		.ctl_name	= NET_IPV4_ROUTE_SRC_CHECK,
		.procname	= "src_check",
		.data		= &ip_rt_src_check,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{ 0 }
};
static ctl_table vz_ipv4_table[] = {
	{NET_IPV4_ROUTE, "route", NULL, 0, 0555, vz_ipv4_route_table},
	{ 0 }
};
static ctl_table vz_net_table[] = {
	{NET_IPV4,   "ipv4",      NULL, 0, 0555, vz_ipv4_table},
	{ 0 }
};
#endif
static ctl_table vz_fs_table[] = {
	{
		.ctl_name	= 226,
		.procname	= "ve-area-access-check",
		.data		= &ve_area_access_check,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{ 0 }
};

static int dummy_pde_data = 0;
static ctl_table dummy_kern_table[] = {
	{
		.ctl_name	= 23571113,
		.procname	= ".dummy-pde",
		.data		= &dummy_pde_data,
		.maxlen		= sizeof(int),
		.mode		= 0400,
		.proc_handler	= proc_dointvec,
	},
	{}
};
static ctl_table root_table2[] = {
#ifdef CONFIG_INET
	{CTL_NET, "net", NULL, 0, 0555, vz_net_table},
#endif
	{CTL_FS, "fs", NULL, 0, 0555, vz_fs_table},
	{CTL_KERN, "kernel", NULL, 0, 0555, dummy_kern_table},
	{ 0 }
};
int prepare_sysctl(void)
{
	struct ve_struct *envid;

	envid = set_exec_env(&ve0);
	register_sysctl_table(root_table2, 0);
	(void)set_exec_env(envid);
	return 0;
}

void prepare_ve0_sysctl(void)
{
	INIT_LIST_HEAD(&ve0.sysctl_lh);
#ifdef CONFIG_SYSCTL
	ve0.proc_sys_root = proc_mkdir("sys", NULL);
#endif
}

/*
 * ------------------------------------------------------------------------
 * XXX init_ve_system
 * ------------------------------------------------------------------------
 */

void init_ve_system(void)
{
	struct task_struct *init_entry;
	struct ve_struct *ve;

	ve = get_ve0();

	init_entry = child_reaper;
	ve->init_entry = init_entry;
	/* if ve_move_task to VE0 (e.g. in cpt code)	*
	 * occurs, ve_cap_bset on VE0 is required	*/
	ve->ve_cap_bset = CAP_INIT_EFF_SET;

#ifdef CONFIG_INET
	ve->_ipv4_devconf = &ipv4_devconf;
	ve->_ipv4_devconf_dflt = &ipv4_devconf_dflt;
#endif

	read_lock(&init_entry->fs->lock);
	ve->fs_rootmnt = init_entry->fs->rootmnt;
	ve->fs_root = init_entry->fs->root;
	read_unlock(&init_entry->fs->lock);

	/* common prepares */
#ifdef CONFIG_PROC_FS
	prepare_proc();
#endif
	prepare_sysctl();
}
