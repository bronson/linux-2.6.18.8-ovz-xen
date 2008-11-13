/*
 *  linux/kernel/ve/ve.c
 *
 *  Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

/*
 * 've.c' helper file performing VE sub-system initialization
 */

#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/capability.h>
#include <linux/ve.h>
#include <linux/smp_lock.h>
#include <linux/init.h>

#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/sys.h>
#include <linux/kdev_t.h>
#include <linux/termios.h>
#include <linux/tty_driver.h>
#include <linux/netdevice.h>
#include <linux/utsname.h>
#include <linux/proc_fs.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/rcupdate.h>
#include <linux/ve_proto.h>
#include <linux/devpts_fs.h>

#include <linux/nfcalls.h>

unsigned long vz_rstamp = 0x37e0f59d;

#ifdef CONFIG_MODULES
struct module no_module = { .state = MODULE_STATE_GOING };
EXPORT_SYMBOL(no_module);
#endif

INIT_KSYM_MODULE(ip_tables);
INIT_KSYM_MODULE(ip6_tables);
INIT_KSYM_MODULE(iptable_filter);
INIT_KSYM_MODULE(ip6table_filter);
INIT_KSYM_MODULE(iptable_mangle);
INIT_KSYM_MODULE(ip6table_mangle);
INIT_KSYM_MODULE(ip_conntrack);
INIT_KSYM_MODULE(ip_conntrack_ftp);
INIT_KSYM_MODULE(ip_conntrack_irc);
INIT_KSYM_MODULE(ip_nat);
INIT_KSYM_MODULE(iptable_nat);
INIT_KSYM_MODULE(ip_nat_ftp);
INIT_KSYM_MODULE(ip_nat_irc);

INIT_KSYM_CALL(int, init_netfilter, (void));
INIT_KSYM_CALL(int, init_iptables, (void));
INIT_KSYM_CALL(int, init_ip6tables, (void));
INIT_KSYM_CALL(int, init_iptable_filter, (void));
INIT_KSYM_CALL(int, init_ip6table_filter, (void));
INIT_KSYM_CALL(int, init_iptable_mangle, (void));
INIT_KSYM_CALL(int, init_ip6table_mangle, (void));
INIT_KSYM_CALL(int, init_iptable_conntrack, (void));
INIT_KSYM_CALL(int, init_ip_ct_ftp, (void));
INIT_KSYM_CALL(int, init_ip_ct_irc, (void));
INIT_KSYM_CALL(int, ip_nat_init, (void));
INIT_KSYM_CALL(int, init_iptable_nat, (void));
INIT_KSYM_CALL(int, init_iptable_nat_ftp, (void));
INIT_KSYM_CALL(int, init_iptable_nat_irc, (void));
INIT_KSYM_CALL(void, fini_iptable_nat_irc, (void));
INIT_KSYM_CALL(void, fini_iptable_nat_ftp, (void));
INIT_KSYM_CALL(void, fini_iptable_nat, (void));
INIT_KSYM_CALL(void, ip_nat_cleanup, (void));
INIT_KSYM_CALL(void, fini_ip_ct_irc, (void));
INIT_KSYM_CALL(void, fini_ip_ct_ftp, (void));
INIT_KSYM_CALL(void, fini_iptable_conntrack, (void));
INIT_KSYM_CALL(void, fini_ip6table_filter, (void));
INIT_KSYM_CALL(void, fini_iptable_filter, (void));
INIT_KSYM_CALL(void, fini_ip6table_mangle, (void));
INIT_KSYM_CALL(void, fini_iptable_mangle, (void));
INIT_KSYM_CALL(void, fini_ip6tables, (void));
INIT_KSYM_CALL(void, fini_iptables, (void));
INIT_KSYM_CALL(void, fini_netfilter, (void));

#if defined(CONFIG_VE_CALLS_MODULE) || defined(CONFIG_VE_CALLS)
INIT_KSYM_MODULE(vzmon);
INIT_KSYM_CALL(void, real_do_env_free, (struct ve_struct *env));

void do_env_free(struct ve_struct *env)
{
	KSYMSAFECALL_VOID(vzmon, real_do_env_free, (env));
}
EXPORT_SYMBOL(do_env_free);
#endif

#if defined(CONFIG_VE_ETHDEV) || defined(CONFIG_VE_ETHDEV_MODULE)
INIT_KSYM_MODULE(vzethdev);
INIT_KSYM_CALL(int, veth_open, (struct net_device *dev));
#endif

struct ve_struct ve0 = {
	.ve_list		= LIST_HEAD_INIT(ve0.ve_list),
	.vetask_lh		= LIST_HEAD_INIT(ve0.vetask_lh),
	.start_jiffies		= INITIAL_JIFFIES,
#ifdef CONFIG_NET
	._net_dev_tail		= &ve0._net_dev_base,
	.ifindex		= -1,
#endif
#ifdef CONFIG_UNIX98_PTYS
	.devpts_config		= &devpts_config,
#endif
	.ve_ns			= &init_nsproxy,
	.is_running		= 1,
	.op_sem			= __RWSEM_INITIALIZER(ve0.op_sem),
};

EXPORT_SYMBOL(ve0);

#ifdef CONFIG_SMP
static struct percpu_data ve0_cpu_stats;
static struct percpu_data ve0_lat_pcpu_stats;
#endif
static struct ve_cpu_stats ve0_cpu_stats_data[NR_CPUS];
static struct kstat_lat_pcpu_snap_struct ve0_lat_pcpu_stats_data[NR_CPUS];

LIST_HEAD(ve_list_head);
rwlock_t ve_list_lock = RW_LOCK_UNLOCKED;

LIST_HEAD(ve_cleanup_list);
DEFINE_SPINLOCK(ve_cleanup_lock);
struct task_struct *ve_cleanup_thread;

EXPORT_SYMBOL(ve_list_lock);
EXPORT_SYMBOL(ve_list_head);
EXPORT_SYMBOL(ve_cleanup_lock);
EXPORT_SYMBOL(ve_cleanup_list);
EXPORT_SYMBOL(ve_cleanup_thread);

void init_ve0(void)
{
	struct ve_struct *ve;

	ve = get_ve0();
	(void)get_ve(ve);
	atomic_set(&ve->pcounter, 1);

	ve->cpu_stats = static_percpu_ptr(&ve0_cpu_stats,
			ve0_cpu_stats_data);
	ve->sched_lat_ve.cur = static_percpu_ptr(&ve0_lat_pcpu_stats,
			ve0_lat_pcpu_stats_data);

	list_add(&ve->ve_list, &ve_list_head);
}

void ve_cleanup_schedule(struct ve_struct *ve)
{
	BUG_ON(ve_cleanup_thread == NULL);

	spin_lock(&ve_cleanup_lock);
	list_add_tail(&ve->cleanup_list, &ve_cleanup_list);
	spin_unlock(&ve_cleanup_lock);

	wake_up_process(ve_cleanup_thread);
}
