/*
 * sysctl.c: General linux system control interface
 *
 * Begun 24 March 1995, Stephen Tweedie
 * Added /proc support, Dec 1995
 * Added bdflush entry and intvec min/max checking, 2/23/96, Tom Dyas.
 * Added hooks for /proc/sys/net (minor, minor patch), 96/4/1, Mike Shaver.
 * Added kernel/java-{interpreter,appletviewer}, 96/5/10, Mike Shaver.
 * Dynamic registration fixes, Stephen Tweedie.
 * Added kswapd-interval, ctrl-alt-del, printk stuff, 1/8/97, Chris Horn.
 * Made sysctl support optional via CONFIG_SYSCTL, 1/10/97, Chris
 *  Horn.
 * Added proc_doulongvec_ms_jiffies_minmax, 09/08/99, Carlos H. Bauer.
 * Added proc_doulongvec_minmax, 09/08/99, Carlos H. Bauer.
 * Changed linked lists to use list.h instead of lists.h, 02/24/00, Bill
 *  Wendling.
 * The list_for_each() macro wasn't appropriate for the sysctl loop.
 *  Removed it and replaced it with older style, 03/23/00, Bill Wendling
 */

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/ve.h>
#include <linux/capability.h>
#include <linux/ctype.h>
#include <linux/utsname.h>
#include <linux/capability.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/net.h>
#include <linux/sysrq.h>
#include <linux/highuid.h>
#include <linux/writeback.h>
#include <linux/hugetlb.h>
#include <linux/security.h>
#include <linux/initrd.h>
#include <linux/times.h>
#include <linux/limits.h>
#include <linux/dcache.h>
#include <linux/syscalls.h>
#include <linux/nfs_fs.h>
#include <linux/acpi.h>
#include <linux/pid.h>

#include <asm/uaccess.h>
#include <asm/processor.h>

extern int proc_nr_files(ctl_table *table, int write, struct file *filp,
                     void __user *buffer, size_t *lenp, loff_t *ppos);

#if defined(CONFIG_SYSCTL)
extern int gr_handle_sysctl_mod(const char *dirname, const char *name,
				const int op);

/* External variables not in a header file. */
extern int C_A_D;
extern int sysctl_overcommit_memory;
extern int sysctl_overcommit_ratio;
extern int sysctl_panic_on_oom;
extern int max_threads;
extern int sysrq_enabled;
extern int core_uses_pid;
extern int suid_dumpable;
extern int sysctl_at_vsyscall;
extern char core_pattern[];
extern int cad_pid;
extern int pid_max;
extern int min_free_kbytes;
extern int printk_ratelimit_jiffies;
extern int printk_ratelimit_burst;
extern int pid_max_min, pid_max_max;
extern int sysctl_drop_caches;
extern int percpu_pagelist_fraction;
extern int compat_log;
extern int ve_area_access_check; /* fs/namei.c */

#if defined(CONFIG_X86_LOCAL_APIC) && defined(CONFIG_X86)
int unknown_nmi_panic;
extern int proc_unknown_nmi_panic(ctl_table *, int, struct file *,
				  void __user *, size_t *, loff_t *);
#endif

/* this is needed for the proc_dointvec_minmax for [fs_]overflow UID and GID */
static int maxolduid = 65535;
static int minolduid;
static int min_percpu_pagelist_fract = 8;

static int ngroups_max = NGROUPS_MAX;

int ve_allow_kthreads = 1;
EXPORT_SYMBOL(ve_allow_kthreads);
#ifdef CONFIG_KMOD
extern char modprobe_path[];
#endif
#ifdef CONFIG_CHR_DEV_SG
extern int sg_big_buff;
#endif
#ifdef CONFIG_SYSVIPC
static int proc_do_ipc_string(ctl_table *table, int write, struct file *filp,
		void __user *buffer, size_t *lenp, loff_t *ppos);
#endif

#ifdef __sparc__
extern char reboot_command [];
extern int stop_a_enabled;
extern int scons_pwroff;
#endif
#ifdef CONFIG_SCHED_VCPU
extern u32 vcpu_sched_timeslice;
extern int vcpu_timeslice;
extern u32 vcpu_hot_timeslice;
#endif

extern int alloc_fail_warn;

#ifdef __hppa__
extern int pwrsw_enabled;
extern int unaligned_enabled;
#endif

#ifdef CONFIG_S390
#ifdef CONFIG_MATHEMU
extern int sysctl_ieee_emulation_warnings;
#endif
extern int sysctl_userprocess_debug;
extern int spin_retry;
#endif

extern int sysctl_hz_timer;
int decode_call_traces = 1;

#ifdef CONFIG_BSD_PROCESS_ACCT
extern int acct_parm[];
#endif

#ifdef CONFIG_IA64
extern int no_unaligned_warning;
#endif
#ifdef CONFIG_VE
int glob_ve_meminfo = 0;
EXPORT_SYMBOL(glob_ve_meminfo);
#endif

#ifdef CONFIG_RT_MUTEXES
extern int max_lock_depth;
#endif

#ifdef CONFIG_FAIRSCHED
extern int fairsched_max_latency;
extern int scale_vcpu_frequency;
int fsch_sysctl_latency(ctl_table *ctl, int write, struct file *filp,
		        void __user *buffer, size_t *lenp, loff_t *ppos);
#endif

static int parse_table(int __user *, int, void __user *, size_t __user *, void __user *, size_t,
		       ctl_table *, void **);
static int proc_do_uts_string(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos);
static int sysctl_uts_string(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context);

static ctl_table root_table[];
static struct ctl_table_header root_table_header =
	{ root_table, LIST_HEAD_INIT(root_table_header.ctl_entry) };

static ctl_table kern_table[];
static ctl_table vm_table[];
static ctl_table fs_table[];
static ctl_table debug_table[];
static ctl_table dev_table[];
extern ctl_table random_table[];
#ifdef CONFIG_UNIX98_PTYS
extern ctl_table pty_table[];
#endif
#ifdef CONFIG_INOTIFY_USER
extern ctl_table inotify_table[];
#endif

#ifdef HAVE_ARCH_PICK_MMAP_LAYOUT
int sysctl_legacy_va_layout;
#endif
extern ctl_table grsecurity_table[];

/* /proc declarations: */

#ifdef CONFIG_PROC_FS

static ssize_t proc_readsys(struct file *, char __user *, size_t, loff_t *);
static ssize_t proc_writesys(struct file *, const char __user *, size_t, loff_t *);
static int proc_opensys(struct inode *, struct file *);

struct file_operations proc_sys_file_operations = {
	.open		= proc_opensys,
	.read		= proc_readsys,
	.write		= proc_writesys,
};

extern struct proc_dir_entry *proc_sys_root;

static void register_proc_table(ctl_table *, struct proc_dir_entry *, void *);
static void unregister_proc_table(ctl_table *, struct proc_dir_entry *);
#endif

extern struct new_utsname virt_utsname;

/* The default sysctl tables: */

static ctl_table root_table[] = {
	{
		.ctl_name	= CTL_KERN,
		.procname	= "kernel",
		.mode		= 0555,
		.child		= kern_table,
	},
	{
		.ctl_name	= CTL_VM,
		.procname	= "vm",
		.mode		= 0555,
		.child		= vm_table,
	},
#ifdef CONFIG_NET
	{
		.ctl_name	= CTL_NET,
		.procname	= "net",
		.mode		= 0555,
		.child		= net_table,
	},
#endif
	{
		.ctl_name	= CTL_FS,
		.procname	= "fs",
		.mode		= 0555,
		.child		= fs_table,
	},
	{
		.ctl_name	= CTL_DEBUG,
		.procname	= "debug",
		.mode		= 0555,
		.child		= debug_table,
	},
	{
		.ctl_name	= CTL_DEV,
		.procname	= "dev",
		.mode		= 0555,
		.child		= dev_table,
	},

	{ .ctl_name = 0 }
};

static ctl_table kern_table[] = {
#ifndef CONFIG_UTS_NS
	{
		.ctl_name	= KERN_OSTYPE,
		.procname	= "ostype",
		.data		= init_uts_ns.name.sysname,
		.maxlen		= sizeof(init_uts_ns.name.sysname),
		.mode		= 0444,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_OSRELEASE,
		.procname	= "osrelease",
		.data		= init_uts_ns.name.release,
		.maxlen		= sizeof(init_uts_ns.name.release),
		.mode		= 0444,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_VERSION,
		.procname	= "version",
		.data		= init_uts_ns.name.version,
		.maxlen		= sizeof(init_uts_ns.name.version),
		.mode		= 0444,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_NODENAME,
		.procname	= "hostname",
		.data		= init_uts_ns.name.nodename,
		.maxlen		= sizeof(init_uts_ns.name.nodename),
		.mode		= 0644,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_DOMAINNAME,
		.procname	= "domainname",
		.data		= init_uts_ns.name.domainname,
		.maxlen		= sizeof(init_uts_ns.name.domainname),
		.mode		= 0644,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
#else  /* !CONFIG_UTS_NS */
	{
		.ctl_name	= KERN_OSTYPE,
		.procname	= "ostype",
		.data		= NULL,
		/* could maybe use __NEW_UTS_LEN here? */
		.maxlen		= FIELD_SIZEOF(struct new_utsname, sysname),
		.mode		= 0444,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_OSRELEASE,
		.procname	= "osrelease",
		.data		= NULL,
		.maxlen		= FIELD_SIZEOF(struct new_utsname, release),
		.mode		= 0444,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_VERSION,
		.procname	= "version",
		.data		= NULL,
		.maxlen		= FIELD_SIZEOF(struct new_utsname, version),
		.mode		= 0444,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_NODENAME,
		.procname	= "hostname",
		.data		= NULL,
		.maxlen		= FIELD_SIZEOF(struct new_utsname, nodename),
		.mode		= 0644,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= KERN_DOMAINNAME,
		.procname	= "domainname",
		.data		= NULL,
		.maxlen		= FIELD_SIZEOF(struct new_utsname, domainname),
		.mode		= 0644,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
		.virt_handler	= 1,
	},
#endif /* !CONFIG_UTS_NS */
	{
		.ctl_name	= KERN_VIRT_OSRELEASE,
		.procname	= "virt_osrelease",
		.data		= virt_utsname.release,
		.maxlen		= sizeof(virt_utsname.release),
		.mode		= 0644,
		.proc_handler	= &proc_do_uts_string,
		.strategy	= &sysctl_uts_string,
	},
	{
		.ctl_name	= KERN_PANIC,
		.procname	= "panic",
		.data		= &panic_timeout,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_CORE_USES_PID,
		.procname	= "core_uses_pid",
		.data		= &core_uses_pid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_CORE_PATTERN,
		.procname	= "core_pattern",
		.data		= core_pattern,
		.maxlen		= 64,
		.mode		= 0644,
		.proc_handler	= &proc_dostring,
		.strategy	= &sysctl_string,
	},
	{
		.ctl_name	= KERN_TAINTED,
		.procname	= "tainted",
		.data		= &tainted,
		.maxlen		= sizeof(int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_CAP_BSET,
		.procname	= "cap-bound",
		.data		= NULL,
		.maxlen		= sizeof(kernel_cap_t),
		.mode		= 0600,
		.proc_handler	= &proc_dointvec_bset,
		.strategy	= &sysctl_strategy_bset,
	},
#ifdef CONFIG_BLK_DEV_INITRD
	{
		.ctl_name	= KERN_REALROOTDEV,
		.procname	= "real-root-dev",
		.data		= &real_root_dev,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef __sparc__
	{
		.ctl_name	= KERN_SPARC_REBOOT,
		.procname	= "reboot-cmd",
		.data		= reboot_command,
		.maxlen		= 256,
		.mode		= 0644,
		.proc_handler	= &proc_dostring,
		.strategy	= &sysctl_string,
	},
	{
		.ctl_name	= KERN_SPARC_STOP_A,
		.procname	= "stop-a",
		.data		= &stop_a_enabled,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_SPARC_SCONS_PWROFF,
		.procname	= "scons-poweroff",
		.data		= &scons_pwroff,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
	{
		.ctl_name	= KERN_SILENCE_LEVEL,
		.procname	= "silence-level",
		.data		= &console_silence_loglevel,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.ctl_name	= KERN_ALLOC_FAIL_WARN,
		.procname	= "alloc_fail_warn",
		.data		= &alloc_fail_warn,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
#ifdef __hppa__
	{
		.ctl_name	= KERN_HPPA_PWRSW,
		.procname	= "soft-power",
		.data		= &pwrsw_enabled,
		.maxlen		= sizeof (int),
	 	.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_HPPA_UNALIGNED,
		.procname	= "unaligned-trap",
		.data		= &unaligned_enabled,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
	{
		.ctl_name	= KERN_CTLALTDEL,
		.procname	= "ctrl-alt-del",
		.data		= &C_A_D,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_PRINTK,
		.procname	= "printk",
		.data		= &console_loglevel,
		.maxlen		= 4*sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#ifdef CONFIG_KMOD
	{
		.ctl_name	= KERN_MODPROBE,
		.procname	= "modprobe",
		.data		= &modprobe_path,
		.maxlen		= KMOD_PATH_LEN,
		.mode		= 0644,
		.proc_handler	= &proc_dostring,
		.strategy	= &sysctl_string,
	},
#endif
#if defined(CONFIG_HOTPLUG) && defined(CONFIG_NET)
	{
		.ctl_name	= KERN_HOTPLUG,
		.procname	= "hotplug",
		.data		= &uevent_helper,
		.maxlen		= UEVENT_HELPER_PATH_LEN,
		.mode		= 0644,
		.proc_handler	= &proc_dostring,
		.strategy	= &sysctl_string,
	},
#endif
#ifdef CONFIG_CHR_DEV_SG
	{
		.ctl_name	= KERN_SG_BIG_BUFF,
		.procname	= "sg-big-buff",
		.data		= &sg_big_buff,
		.maxlen		= sizeof (int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_BSD_PROCESS_ACCT
	{
		.ctl_name	= KERN_ACCT,
		.procname	= "acct",
		.data		= &acct_parm,
		.maxlen		= 3*sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_SYSVIPC
	{
		.ctl_name	= KERN_SHMMAX,
		.procname	= "shmmax",
		.data		= NULL,
		.maxlen		= sizeof (size_t),
		.mode		= 0644,
		.proc_handler	= &proc_do_ipc_string,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= KERN_SHMALL,
		.procname	= "shmall",
		.data		= NULL,
		.maxlen		= sizeof (size_t),
		.mode		= 0644,
		.proc_handler	= &proc_do_ipc_string,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= KERN_SHMMNI,
		.procname	= "shmmni",
		.data		= NULL,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_do_ipc_string,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= KERN_MSGMAX,
		.procname	= "msgmax",
		.data		= NULL,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_do_ipc_string,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= KERN_MSGMNI,
		.procname	= "msgmni",
		.data		= NULL,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_do_ipc_string,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= KERN_MSGMNB,
		.procname	=  "msgmnb",
		.data		= NULL,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_do_ipc_string,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= KERN_SEM,
		.procname	= "sem",
		.data		= NULL,
		.maxlen		= 4*sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_do_ipc_string,
		.virt_handler	= 1,
	},
#endif
#ifdef CONFIG_MAGIC_SYSRQ
	{
		.ctl_name	= KERN_SYSRQ,
		.procname	= "sysrq",
		.data		= &sysrq_enabled,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
	{
		.ctl_name	= KERN_CADPID,
		.procname	= "cad_pid",
		.data		= &cad_pid,
		.maxlen		= sizeof (int),
		.mode		= 0600,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_MAX_THREADS,
		.procname	= "threads-max",
		.data		= &max_threads,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_RANDOM,
		.procname	= "random",
		.mode		= 0555,
		.child		= random_table,
	},
#ifdef CONFIG_UNIX98_PTYS
	{
		.ctl_name	= KERN_PTY,
		.procname	= "pty",
		.mode		= 0555,
		.child		= pty_table,
	},
#endif
	{
		.ctl_name	= KERN_OVERFLOWUID,
		.procname	= "overflowuid",
		.data		= &overflowuid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= &sysctl_intvec,
		.extra1		= &minolduid,
		.extra2		= &maxolduid,
	},
	{
		.ctl_name	= KERN_OVERFLOWGID,
		.procname	= "overflowgid",
		.data		= &overflowgid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= &sysctl_intvec,
		.extra1		= &minolduid,
		.extra2		= &maxolduid,
	},
#ifdef CONFIG_S390
#ifdef CONFIG_MATHEMU
	{
		.ctl_name	= KERN_IEEE_EMULATION_WARNINGS,
		.procname	= "ieee_emulation_warnings",
		.data		= &sysctl_ieee_emulation_warnings,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_NO_IDLE_HZ
	{
		.ctl_name       = KERN_HZ_TIMER,
		.procname       = "hz_timer",
		.data           = &sysctl_hz_timer,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = &proc_dointvec,
	},
#endif
	{
		.ctl_name	= KERN_S390_USER_DEBUG_LOGGING,
		.procname	= "userprocess_debug",
		.data		= &sysctl_userprocess_debug,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_SCHED_VCPU
	{
		.ctl_name	= KERN_VCPU_SCHED_TIMESLICE,
		.procname	= "vcpu_sched_timeslice",
		.data		= &vcpu_sched_timeslice,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_VCPU_TIMESLICE,
		.procname	= "vcpu_timeslice",
		.data		= &vcpu_timeslice,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_VCPU_HOT_TIMESLICE,
		.procname	= "vcpu_hot_timeslice",
		.data		= &vcpu_hot_timeslice,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
	{
		.ctl_name	= KERN_PIDMAX,
		.procname	= "pid_max",
		.data		= &pid_max,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= sysctl_intvec,
		.extra1		= &pid_max_min,
		.extra2		= &pid_max_max,
	},
#ifdef CONFIG_VE
	{
		.ctl_name	= KERN_VIRT_PIDS,
		.procname	= "virt_pids",
		.data		= &glob_virt_pids,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_VE_MEMINFO,
		.procname	= "ve_meminfo",
		.data		= &glob_ve_meminfo,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
	{
		.ctl_name	= KERN_PANIC_ON_OOPS,
		.procname	= "panic_on_oops",
		.data		= &panic_on_oops,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_PRINTK_RATELIMIT,
		.procname	= "printk_ratelimit",
		.data		= &printk_ratelimit_jiffies,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_jiffies,
		.strategy	= &sysctl_jiffies,
	},
	{
		.ctl_name	= KERN_PRINTK_RATELIMIT_BURST,
		.procname	= "printk_ratelimit_burst",
		.data		= &printk_ratelimit_burst,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= KERN_NGROUPS_MAX,
		.procname	= "ngroups_max",
		.data		= &ngroups_max,
		.maxlen		= sizeof (int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec,
	},
#if defined(CONFIG_X86_LOCAL_APIC) && defined(CONFIG_X86)
	{
		.ctl_name       = KERN_UNKNOWN_NMI_PANIC,
		.procname       = "unknown_nmi_panic",
		.data           = &unknown_nmi_panic,
		.maxlen         = sizeof (int),
		.mode           = 0644,
		.proc_handler   = &proc_unknown_nmi_panic,
	},
#endif
#if defined(CONFIG_X86)
	{
		.ctl_name	= KERN_BOOTLOADER_TYPE,
		.procname	= "bootloader_type",
		.data		= &bootloader_type,
		.maxlen		= sizeof (int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec,
	},
#endif
#if defined(CONFIG_MMU)
	{
		.ctl_name	= KERN_RANDOMIZE,
		.procname	= "randomize_va_space",
		.data		= &randomize_va_space,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#if defined(CONFIG_S390) && defined(CONFIG_SMP)
	{
		.ctl_name	= KERN_SPIN_RETRY,
		.procname	= "spin_retry",
		.data		= &spin_retry,
		.maxlen		= sizeof (int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_ACPI_SLEEP
	{
		.ctl_name	= KERN_ACPI_VIDEO_FLAGS,
		.procname	= "acpi_video_flags",
		.data		= &acpi_video_flags,
		.maxlen		= sizeof (unsigned long),
		.mode		= 0644,
		.proc_handler	= &proc_doulongvec_minmax,
	},
#endif
#ifdef CONFIG_IA64
	{
		.ctl_name	= KERN_IA64_UNALIGNED,
		.procname	= "ignore-unaligned-usertrap",
		.data		= &no_unaligned_warning,
		.maxlen		= sizeof (int),
	 	.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_COMPAT
	{
		.ctl_name	= KERN_COMPAT_LOG,
		.procname	= "compat-log",
		.data		= &compat_log,
		.maxlen		= sizeof (int),
	 	.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_RT_MUTEXES
	{
		.ctl_name	= KERN_MAX_LOCK_DEPTH,
		.procname	= "max_lock_depth",
		.data		= &max_lock_depth,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_GRKERNSEC_SYSCTL
	{
		.ctl_name	= KERN_GRSECURITY,
		.procname	= "grsecurity",
		.mode		= 0500,
		.child		= grsecurity_table,
	},
#endif

	{ .ctl_name = 0 }
};

/* Constants for minimum and maximum testing in vm_table.
   We use these as one-element integer vectors. */
static int zero;
static int one_hundred = 100;


static ctl_table vm_table[] = {
	{
		.ctl_name	= VM_OVERCOMMIT_MEMORY,
		.procname	= "overcommit_memory",
		.data		= &sysctl_overcommit_memory,
		.maxlen		= sizeof(sysctl_overcommit_memory),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= VM_PANIC_ON_OOM,
		.procname	= "panic_on_oom",
		.data		= &sysctl_panic_on_oom,
		.maxlen		= sizeof(sysctl_panic_on_oom),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= VM_OVERCOMMIT_RATIO,
		.procname	= "overcommit_ratio",
		.data		= &sysctl_overcommit_ratio,
		.maxlen		= sizeof(sysctl_overcommit_ratio),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= VM_PAGE_CLUSTER,
		.procname	= "page-cluster", 
		.data		= &page_cluster,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= VM_DIRTY_BACKGROUND,
		.procname	= "dirty_background_ratio",
		.data		= &dirty_background_ratio,
		.maxlen		= sizeof(dirty_background_ratio),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
		.extra2		= &one_hundred,
	},
	{
		.ctl_name	= VM_DIRTY_RATIO,
		.procname	= "dirty_ratio",
		.data		= &vm_dirty_ratio,
		.maxlen		= sizeof(vm_dirty_ratio),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
		.extra2		= &one_hundred,
	},
	{
		.ctl_name	= VM_DIRTY_WB_CS,
		.procname	= "dirty_writeback_centisecs",
		.data		= &dirty_writeback_interval,
		.maxlen		= sizeof(dirty_writeback_interval),
		.mode		= 0644,
		.proc_handler	= &dirty_writeback_centisecs_handler,
	},
	{
		.ctl_name	= VM_DIRTY_EXPIRE_CS,
		.procname	= "dirty_expire_centisecs",
		.data		= &dirty_expire_interval,
		.maxlen		= sizeof(dirty_expire_interval),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_userhz_jiffies,
	},
	{
		.ctl_name	= VM_NR_PDFLUSH_THREADS,
		.procname	= "nr_pdflush_threads",
		.data		= &nr_pdflush_threads,
		.maxlen		= sizeof nr_pdflush_threads,
		.mode		= 0444 /* read-only*/,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= VM_SWAPPINESS,
		.procname	= "swappiness",
		.data		= &vm_swappiness,
		.maxlen		= sizeof(vm_swappiness),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
		.extra2		= &one_hundred,
	},
#ifdef CONFIG_HUGETLB_PAGE
	 {
		.ctl_name	= VM_HUGETLB_PAGES,
		.procname	= "nr_hugepages",
		.data		= &max_huge_pages,
		.maxlen		= sizeof(unsigned long),
		.mode		= 0644,
		.proc_handler	= &hugetlb_sysctl_handler,
		.extra1		= (void *)&hugetlb_zero,
		.extra2		= (void *)&hugetlb_infinity,
	 },
	 {
		.ctl_name	= VM_HUGETLB_GROUP,
		.procname	= "hugetlb_shm_group",
		.data		= &sysctl_hugetlb_shm_group,
		.maxlen		= sizeof(gid_t),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	 },
#endif
	{
		.ctl_name	= VM_LOWMEM_RESERVE_RATIO,
		.procname	= "lowmem_reserve_ratio",
		.data		= &sysctl_lowmem_reserve_ratio,
		.maxlen		= sizeof(sysctl_lowmem_reserve_ratio),
		.mode		= 0644,
		.proc_handler	= &lowmem_reserve_ratio_sysctl_handler,
		.strategy	= &sysctl_intvec,
	},
	{
		.ctl_name	= VM_DROP_PAGECACHE,
		.procname	= "drop_caches",
		.data		= &sysctl_drop_caches,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= drop_caches_sysctl_handler,
		.strategy	= &sysctl_intvec,
	},
	{
		.ctl_name	= VM_MIN_FREE_KBYTES,
		.procname	= "min_free_kbytes",
		.data		= &min_free_kbytes,
		.maxlen		= sizeof(min_free_kbytes),
		.mode		= 0644,
		.proc_handler	= &min_free_kbytes_sysctl_handler,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
	},
	{
		.ctl_name	= VM_PERCPU_PAGELIST_FRACTION,
		.procname	= "percpu_pagelist_fraction",
		.data		= &percpu_pagelist_fraction,
		.maxlen		= sizeof(percpu_pagelist_fraction),
		.mode		= 0644,
		.proc_handler	= &percpu_pagelist_fraction_sysctl_handler,
		.strategy	= &sysctl_intvec,
		.extra1		= &min_percpu_pagelist_fract,
	},
#ifdef CONFIG_MMU
	{
		.ctl_name	= VM_MAX_MAP_COUNT,
		.procname	= "max_map_count",
		.data		= &sysctl_max_map_count,
		.maxlen		= sizeof(sysctl_max_map_count),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
#endif
	{
		.ctl_name	= VM_LAPTOP_MODE,
		.procname	= "laptop_mode",
		.data		= &laptop_mode,
		.maxlen		= sizeof(laptop_mode),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_jiffies,
		.strategy	= &sysctl_jiffies,
	},
	{
		.ctl_name	= VM_BLOCK_DUMP,
		.procname	= "block_dump",
		.data		= &block_dump,
		.maxlen		= sizeof(block_dump),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
	},
	{
		.ctl_name	= VM_VFS_CACHE_PRESSURE,
		.procname	= "vfs_cache_pressure",
		.data		= &sysctl_vfs_cache_pressure,
		.maxlen		= sizeof(sysctl_vfs_cache_pressure),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
	},
#ifdef HAVE_ARCH_PICK_MMAP_LAYOUT
	{
		.ctl_name	= VM_LEGACY_VA_LAYOUT,
		.procname	= "legacy_va_layout",
		.data		= &sysctl_legacy_va_layout,
		.maxlen		= sizeof(sysctl_legacy_va_layout),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
	},
#endif
#ifdef CONFIG_SWAP
	{
		.ctl_name	= VM_SWAP_TOKEN_TIMEOUT,
		.procname	= "swap_token_timeout",
		.data		= &swap_token_default_timeout,
		.maxlen		= sizeof(swap_token_default_timeout),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_jiffies,
		.strategy	= &sysctl_jiffies,
	},
#endif
#ifdef CONFIG_NUMA
	{
		.ctl_name	= VM_ZONE_RECLAIM_MODE,
		.procname	= "zone_reclaim_mode",
		.data		= &zone_reclaim_mode,
		.maxlen		= sizeof(zone_reclaim_mode),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
	},
	{
		.ctl_name	= VM_MIN_UNMAPPED,
		.procname	= "min_unmapped_ratio",
		.data		= &sysctl_min_unmapped_ratio,
		.maxlen		= sizeof(sysctl_min_unmapped_ratio),
		.mode		= 0644,
		.proc_handler	= &sysctl_min_unmapped_ratio_sysctl_handler,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
		.extra2		= &one_hundred,
	},
	{
		.ctl_name	= VM_MIN_SLAB,
		.procname	= "min_slab_ratio",
		.data		= &sysctl_min_slab_ratio,
		.maxlen		= sizeof(sysctl_min_slab_ratio),
		.mode		= 0644,
		.proc_handler	= &sysctl_min_slab_ratio_sysctl_handler,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
		.extra2		= &one_hundred,
	},
#endif
#ifdef CONFIG_X86_32
	{
		.ctl_name	= VM_VDSO_ENABLED,
		.procname	= "vdso_enabled",
		.data		= &vdso_enabled,
		.maxlen		= sizeof(vdso_enabled),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
		.strategy	= &sysctl_intvec,
		.extra1		= &zero,
	},
#endif
#ifdef CONFIG_FAIRSCHED
	{
		.ctl_name	= KERN_FAIRSCHED_MAX_LATENCY,
		.procname	= "fairsched-max-latency",
		.data		=  &fairsched_max_latency,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &fsch_sysctl_latency
	},
	{
		.ctl_name	= KERN_SCALE_VCPU_FREQUENCY,
		.procname	= "scale_vcpu_frequency",
		.data		= &scale_vcpu_frequency,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
	{ .ctl_name = 0 }
};

static ctl_table fs_table[] = {
	{
		.ctl_name	= FS_NRINODE,
		.procname	= "inode-nr",
		.data		= &inodes_stat,
		.maxlen		= 2*sizeof(int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= FS_STATINODE,
		.procname	= "inode-state",
		.data		= &inodes_stat,
		.maxlen		= 7*sizeof(int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= FS_NRFILE,
		.procname	= "file-nr",
		.data		= &files_stat,
		.maxlen		= 3*sizeof(int),
		.mode		= 0444,
		.proc_handler	= &proc_nr_files,
	},
	{
		.ctl_name	= FS_MAXFILE,
		.procname	= "file-max",
		.data		= &files_stat.max_files,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= FS_DENTRY,
		.procname	= "dentry-state",
		.data		= &dentry_stat,
		.maxlen		= 6*sizeof(int),
		.mode		= 0444,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= FS_OVERFLOWUID,
		.procname	= "overflowuid",
		.data		= &fs_overflowuid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= &sysctl_intvec,
		.extra1		= &minolduid,
		.extra2		= &maxolduid,
	},
	{
		.ctl_name	= FS_OVERFLOWGID,
		.procname	= "overflowgid",
		.data		= &fs_overflowgid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec_minmax,
		.strategy	= &sysctl_intvec,
		.extra1		= &minolduid,
		.extra2		= &maxolduid,
	},
	{
		.ctl_name	= FS_LEASES,
		.procname	= "leases-enable",
		.data		= &leases_enable,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#ifdef CONFIG_DNOTIFY
	{
		.ctl_name	= FS_DIR_NOTIFY,
		.procname	= "dir-notify-enable",
		.data		= &dir_notify_enable,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
#endif
#ifdef CONFIG_MMU
	{
		.ctl_name	= FS_LEASE_TIME,
		.procname	= "lease-break-time",
		.data		= &lease_break_time,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= FS_AIO_NR,
		.procname	= "aio-nr",
		.data		= &aio_nr,
		.maxlen		= sizeof(aio_nr),
		.mode		= 0444,
		.proc_handler	= &proc_doulongvec_minmax,
	},
	{
		.ctl_name	= FS_AIO_MAX_NR,
		.procname	= "aio-max-nr",
		.data		= &aio_max_nr,
		.maxlen		= sizeof(aio_max_nr),
		.mode		= 0644,
		.proc_handler	= &proc_doulongvec_minmax,
	},
#ifdef CONFIG_INOTIFY_USER
	{
		.ctl_name	= FS_INOTIFY,
		.procname	= "inotify",
		.mode		= 0555,
		.child		= inotify_table,
	},
#endif	
#endif
	{
		.ctl_name	= KERN_SETUID_DUMPABLE,
		.procname	= "suid_dumpable",
		.data		= &suid_dumpable,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= FS_AT_VSYSCALL,
		.procname	= "vsyscall",
		.data		= &sysctl_at_vsyscall,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{ .ctl_name = 0 }
};

static ctl_table debug_table[] = {
	{
		.ctl_name	= DBG_DECODE_CALLTRACES,
		.procname	= "decode_call_traces",
		.data		= &decode_call_traces,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{
		.ctl_name	= FS_ODIRECT,
		.procname	= "odirect_enable",
		.data		= &odirect_enable,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec
	},
	{ .ctl_name = 0 }
};

static ctl_table dev_table[] = {
	{ .ctl_name = 0 }
};

extern void init_irq_proc (void);

static DEFINE_SPINLOCK(sysctl_lock);

/* called under sysctl_lock */
static int use_table(struct ctl_table_header *p)
{
	if (unlikely(p->unregistering))
		return 0;
	p->used++;
	return 1;
}

/* called under sysctl_lock */
static void unuse_table(struct ctl_table_header *p)
{
	if (!--p->used)
		if (unlikely(p->unregistering))
			complete(p->unregistering);
}

/* called under sysctl_lock, will reacquire if has to wait */
static void start_unregistering(struct ctl_table_header *p)
{
	/*
	 * if p->used is 0, nobody will ever touch that entry again;
	 * we'll eliminate all paths to it before dropping sysctl_lock
	 */
	if (unlikely(p->used)) {
		struct completion wait;
		init_completion(&wait);
		p->unregistering = &wait;
		spin_unlock(&sysctl_lock);
		wait_for_completion(&wait);
		spin_lock(&sysctl_lock);
	}
	/*
	 * do not remove from the list until nobody holds it; walking the
	 * list in do_sysctl() relies on that.
	 */
	list_del_init(&p->ctl_entry);
}

void __init sysctl_init(void)
{
#ifdef CONFIG_PROC_FS
	register_proc_table(root_table, proc_sys_root, &root_table_header);
	init_irq_proc();
#endif
}

int do_sysctl(int __user *name, int nlen, void __user *oldval, size_t __user *oldlenp,
	       void __user *newval, size_t newlen)
{
	struct list_head *tmp;
	int error = -ENOTDIR;
	struct ve_struct *ve;

	if (nlen <= 0 || nlen >= CTL_MAXNAME)
		return -ENOTDIR;
	if (oldval) {
		int old_len;
		if (!oldlenp || get_user(old_len, oldlenp))
			return -EFAULT;
	}
	ve = get_exec_env();
	spin_lock(&sysctl_lock);
#ifdef CONFIG_VE
	tmp = ve->sysctl_lh.next;
#else
	tmp = &root_table_header.ctl_entry;
#endif
	do {
		struct ctl_table_header *head;
		void *context = NULL;

#ifdef CONFIG_VE
		if (tmp == &ve->sysctl_lh)
			/* second pass over global variables */
			tmp = &root_table_header.ctl_entry;
#endif

		head = list_entry(tmp, struct ctl_table_header, ctl_entry);
		if (!use_table(head))
			continue;

		spin_unlock(&sysctl_lock);

		error = parse_table(name, nlen, oldval, oldlenp, 
					newval, newlen, head->ctl_table,
					&context);
		kfree(context);

		spin_lock(&sysctl_lock);
		unuse_table(head);
		if (error != -ENOTDIR)
			break;
	} while ((tmp = tmp->next) != &root_table_header.ctl_entry);
	spin_unlock(&sysctl_lock);
	return error;
}

asmlinkage long sys_sysctl(struct __sysctl_args __user *args)
{
	struct __sysctl_args tmp;
	int error;

	if (copy_from_user(&tmp, args, sizeof(tmp)))
		return -EFAULT;

	lock_kernel();
	error = do_sysctl(tmp.name, tmp.nlen, tmp.oldval, tmp.oldlenp,
			  tmp.newval, tmp.newlen);
	unlock_kernel();
	return error;
}

/*
 * ctl_perm does NOT grant the superuser all rights automatically, because
 * some sysctl variables are readonly even to root.
 */

static int test_perm(int mode, int op)
{
	if (!current->euid)
		mode >>= 6;
	else if (in_egroup_p(0))
		mode >>= 3;
	if ((mode & op & 0007) == op)
		return 0;
	return -EACCES;
}

static inline int ctl_perm(ctl_table *table, int op)
{
	int error;
	int mode = table->mode;

	if (table->de && gr_handle_sysctl_mod(table->de->parent->name, table->de->name, op))
		return -EACCES;
	error = security_sysctl(table, op);
	if (error)
		return error;
	if (!ve_accessible(table->owner_env, get_exec_env()) &&
			!table->virt_handler)
		mode &= ~0222; /* disable write access */
	return test_perm(mode, op);
}

static int parse_table(int __user *name, int nlen,
		       void __user *oldval, size_t __user *oldlenp,
		       void __user *newval, size_t newlen,
		       ctl_table *table, void **context)
{
	int n;
repeat:
	if (!nlen)
		return -ENOTDIR;
	if (get_user(n, name))
		return -EFAULT;
	for ( ; table->ctl_name; table++) {
		if (n == table->ctl_name || table->ctl_name == CTL_ANY) {
			int error;
			if (table->child) {
				if (ctl_perm(table, 001))
					return -EPERM;
				if (table->strategy) {
					error = table->strategy(
						table, name, nlen,
						oldval, oldlenp,
						newval, newlen, context);
					if (error)
						return error;
				}
				name++;
				nlen--;
				table = table->child;
				goto repeat;
			}
			error = do_sysctl_strategy(table, name, nlen,
						   oldval, oldlenp,
						   newval, newlen, context);
			return error;
		}
	}
	return -ENOTDIR;
}

int __do_sysctl_strategy (void  *data, ctl_table *table,
			int __user *name, int nlen,
			void __user *oldval, size_t __user *oldlenp,
			void __user *newval, size_t newlen, void **context) {
	size_t len;

	if (oldval && oldlenp) {
		if (get_user(len, oldlenp))
			return -EFAULT;
		if (len) {
			if (len > table->maxlen)
				len = table->maxlen;
			if (copy_to_user(oldval, data, len))
				return -EFAULT;
			if (put_user(len, oldlenp))
				return -EFAULT;
		}
	}

	if (newval && newlen) {
		len = newlen;
		if (len > table->maxlen)
			len = table->maxlen;
		if (copy_from_user(data, newval, len))
			return -EFAULT;
	}

	return 0;
}

/* Perform the actual read/write of a sysctl table entry. */
int do_sysctl_strategy (ctl_table *table, 
			int __user *name, int nlen,
			void __user *oldval, size_t __user *oldlenp,
			void __user *newval, size_t newlen, void **context)
{
	int op = 0, rc;

	if (oldval)
		op |= 004;
	if (newval) 
		op |= 002;
	if (ctl_perm(table, op))
		return -EPERM;

	if (table->strategy) {
		rc = table->strategy(table, name, nlen, oldval, oldlenp,
				     newval, newlen, context);
		if (rc < 0)
			return rc;
		if (rc > 0)
			return 0;
	}

	/* If there is no strategy routine, or if the strategy returns
	 * zero, proceed with automatic r/w */
	if (table->data && table->maxlen)
		return __do_sysctl_strategy (table->data, table, name, nlen,
				oldval, oldlenp, newval, newlen, context);

	return 0;
}

/**
 * register_sysctl_table - register a sysctl hierarchy
 * @table: the top-level table structure
 * @insert_at_head: whether the entry should be inserted in front or at the end
 *
 * Register a sysctl table hierarchy. @table should be a filled in ctl_table
 * array. An entry with a ctl_name of 0 terminates the table. 
 *
 * The members of the &ctl_table structure are used as follows:
 *
 * ctl_name - This is the numeric sysctl value used by sysctl(2). The number
 *            must be unique within that level of sysctl
 *
 * procname - the name of the sysctl file under /proc/sys. Set to %NULL to not
 *            enter a sysctl file
 *
 * data - a pointer to data for use by proc_handler
 *
 * maxlen - the maximum size in bytes of the data
 *
 * mode - the file permissions for the /proc/sys file, and for sysctl(2)
 *
 * child - a pointer to the child sysctl table if this entry is a directory, or
 *         %NULL.
 *
 * proc_handler - the text handler routine (described below)
 *
 * strategy - the strategy routine (described below)
 *
 * de - for internal use by the sysctl routines
 *
 * extra1, extra2 - extra pointers usable by the proc handler routines
 *
 * Leaf nodes in the sysctl tree will be represented by a single file
 * under /proc; non-leaf nodes will be represented by directories.
 *
 * sysctl(2) can automatically manage read and write requests through
 * the sysctl table.  The data and maxlen fields of the ctl_table
 * struct enable minimal validation of the values being written to be
 * performed, and the mode field allows minimal authentication.
 *
 * More sophisticated management can be enabled by the provision of a
 * strategy routine with the table entry.  This will be called before
 * any automatic read or write of the data is performed.
 *
 * The strategy routine may return
 *
 * < 0 - Error occurred (error is passed to user process)
 *
 * 0   - OK - proceed with automatic read or write.
 *
 * > 0 - OK - read or write has been done by the strategy routine, so
 *       return immediately.
 *
 * There must be a proc_handler routine for any terminal nodes
 * mirrored under /proc/sys (non-terminals are handled by a built-in
 * directory handler).  Several default handlers are available to
 * cover common cases -
 *
 * proc_dostring(), proc_dointvec(), proc_dointvec_jiffies(),
 * proc_dointvec_userhz_jiffies(), proc_dointvec_minmax(), 
 * proc_doulongvec_ms_jiffies_minmax(), proc_doulongvec_minmax()
 *
 * It is the handler's job to read the input buffer from user memory
 * and process it. The handler should return 0 on success.
 *
 * This routine returns %NULL on a failure to register, and a pointer
 * to the table header on success.
 */
struct ctl_table_header *register_sysctl_table(ctl_table * table, 
					       int insert_at_head)
{
	struct ctl_table_header *tmp;
	struct list_head *lh;

	tmp = kmalloc(sizeof(struct ctl_table_header), GFP_KERNEL);
	if (!tmp)
		return NULL;
	tmp->ctl_table = table;
	INIT_LIST_HEAD(&tmp->ctl_entry);
	tmp->used = 0;
	tmp->unregistering = NULL;
	spin_lock(&sysctl_lock);
#ifdef CONFIG_VE
	lh = &get_exec_env()->sysctl_lh;
#else
	lh = &root_table_header.ctl_entry;
#endif
	if (insert_at_head)
		list_add(&tmp->ctl_entry, lh);
	else
		list_add_tail(&tmp->ctl_entry, lh);
	spin_unlock(&sysctl_lock);
#ifdef CONFIG_PROC_FS
#ifdef CONFIG_VE
	register_proc_table(table, get_exec_env()->proc_sys_root, tmp);
#else
	register_proc_table(table, proc_sys_root, tmp);
#endif
#endif
	return tmp;
}

void free_sysctl_clone(ctl_table *clone)
{
	int i;

	for (i = 0; clone[i].ctl_name != 0; i++)
		if (clone[i].child != NULL)
			free_sysctl_clone(clone[i].child);

	kfree(clone);
}

ctl_table *clone_sysctl_template(ctl_table *tmpl)
{
	int i, nr;
	ctl_table *clone;

	nr = 0;
	while (tmpl[nr].ctl_name != 0)
		nr++;
	nr++;

	clone = kmalloc(nr * sizeof(ctl_table), GFP_KERNEL);
	if (clone == NULL)
		return NULL;

	memcpy(clone, tmpl, nr * sizeof(ctl_table));
	for (i = 0; i < nr; i++) {
		clone[i].owner_env = get_exec_env();
		clone[i].de = NULL;
		if (tmpl[i].child == NULL)
			continue;

		clone[i].child = clone_sysctl_template(tmpl[i].child);
		if (clone[i].child == NULL)
			goto unroll;
	}
	return clone;

unroll:
	for (i--; i >= 0; i--)
		if (clone[i].child != NULL)
			free_sysctl_clone(clone[i].child);

	kfree(clone);
	return NULL;
}

/**
 * unregister_sysctl_table - unregister a sysctl table hierarchy
 * @header: the header returned from register_sysctl_table
 *
 * Unregisters the sysctl table and all children. proc entries may not
 * actually be removed until they are no longer used by anyone.
 */
void unregister_sysctl_table(struct ctl_table_header * header)
{
	might_sleep();
	spin_lock(&sysctl_lock);
	start_unregistering(header);
#ifdef CONFIG_PROC_FS
#ifdef CONFIG_VE
	unregister_proc_table(header->ctl_table, get_exec_env()->proc_sys_root);
#else
	unregister_proc_table(header->ctl_table, proc_sys_root);
#endif
#endif
	spin_unlock(&sysctl_lock);
	kfree(header);
}

/*
 * /proc/sys support
 */

#ifdef CONFIG_PROC_FS

/* Scan the sysctl entries in table and add them all into /proc */
static void register_proc_table(ctl_table * table, struct proc_dir_entry *root, void *set)
{
	struct proc_dir_entry *de;
	int len;
	mode_t mode;
	
	for (; table->ctl_name; table++) {
		/* Can't do anything without a proc name. */
		if (!table->procname)
			continue;
		/* Maybe we can't do anything with it... */
		if (!table->proc_handler && !table->child) {
			printk(KERN_WARNING "SYSCTL: Can't register %s\n",
				table->procname);
			continue;
		}

		len = strlen(table->procname);
		mode = table->mode;

		de = NULL;
		if (table->proc_handler)
			mode |= S_IFREG;
		else {
			mode |= S_IFDIR;
			for (de = root->subdir; de; de = de->next) {
				if (proc_match(len, table->procname, de))
					break;
			}
			/* If the subdir exists already, de is non-NULL */
		}

		if (!de) {
			de = create_proc_entry(table->procname, mode, root);
			if (!de)
				continue;
			de->set = set;
			de->data = (void *) table;
			if (table->proc_handler)
				de->proc_fops = &proc_sys_file_operations;
		}
		table->de = de;
		if (de->mode & S_IFDIR)
			register_proc_table(table->child, de, set);
	}
}

/*
 * Unregister a /proc sysctl table and any subdirectories.
 */
static void unregister_proc_table(ctl_table * table, struct proc_dir_entry *root)
{
	struct proc_dir_entry *de;
	for (; table->ctl_name; table++) {
		if (!(de = table->de))
			continue;
		if (de->mode & S_IFDIR) {
			if (!table->child) {
				printk (KERN_ALERT "Help - malformed sysctl tree on free\n");
				continue;
			}
			unregister_proc_table(table->child, de);

			/* Don't unregister directories which still have entries.. */
			if (de->subdir)
				continue;
		}

		/*
		 * In any case, mark the entry as goner; we'll keep it
		 * around if it's busy, but we'll know to do nothing with
		 * its fields.  We are under sysctl_lock here.
		 */
		de->data = NULL;
		table->de = NULL;
		remove_proc_entry(table->procname, root);
	}
}

static ssize_t do_rw_proc(int write, struct file * file, char __user * buf,
			  size_t count, loff_t *ppos)
{
	int op;
	struct proc_dir_entry *de = PDE(file->f_dentry->d_inode);
	struct ctl_table *table;
	size_t res;
	ssize_t error = -ENOTDIR;
	
	spin_lock(&sysctl_lock);
	if (de && de->data && use_table(de->set)) {
		/*
		 * at that point we know that sysctl was not unregistered
		 * and won't be until we finish
		 */
		spin_unlock(&sysctl_lock);
		table = (struct ctl_table *) de->data;
		if (!table || !table->proc_handler)
			goto out;
		error = -EPERM;
		op = (write ? 002 : 004);
		if (ctl_perm(table, op))
			goto out;
		
		/* careful: calling conventions are nasty here */
		res = count;
		error = (*table->proc_handler)(table, write, file,
						buf, &res, ppos);
		if (!error)
			error = res;
	out:
		spin_lock(&sysctl_lock);
		unuse_table(de->set);
	}
	spin_unlock(&sysctl_lock);
	return error;
}

static int proc_opensys(struct inode *inode, struct file *file)
{
	if (file->f_mode & FMODE_WRITE) {
		/*
		 * sysctl entries that are not writable,
		 * are _NOT_ writable, capabilities or not.
		 */
		if (!(inode->i_mode & S_IWUSR))
			return -EPERM;
	}

	return 0;
}

static ssize_t proc_readsys(struct file * file, char __user * buf,
			    size_t count, loff_t *ppos)
{
	return do_rw_proc(0, file, buf, count, ppos);
}

static ssize_t proc_writesys(struct file * file, const char __user * buf,
			     size_t count, loff_t *ppos)
{
	return do_rw_proc(1, file, (char __user *) buf, count, ppos);
}

/**
 * proc_dostring - read a string sysctl
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: file position
 *
 * Reads/writes a string from/to the user buffer. If the kernel
 * buffer provided is not large enough to hold the string, the
 * string is truncated. The copied string is %NULL-terminated.
 * If the string is being read by the user process, it is copied
 * and a newline '\n' is added. It is truncated if the buffer is
 * not large enough.
 *
 * Returns 0 on success.
 */
static int _proc_dostring(void *data, int maxlen, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	size_t len;
	char __user *p;
	char c;
	
	if (!data || !maxlen || !*lenp ||
	    (*ppos && !write)) {
		*lenp = 0;
		return 0;
	}
	
	if (write) {
		len = 0;
		p = buffer;
		while (len < *lenp) {
			if (get_user(c, p++))
				return -EFAULT;
			if (c == 0 || c == '\n')
				break;
			len++;
		}
		if (len >= maxlen)
			len = maxlen-1;
		if(copy_from_user(data, buffer, len))
			return -EFAULT;
		((char *) data)[len] = 0;
		*ppos += *lenp;
	} else {
		len = strlen(data);
		if (len > maxlen)
			len = maxlen;
		if (len > *lenp)
			len = *lenp;
		if (len)
			if(copy_to_user(buffer, data, len))
				return -EFAULT;
		if (len < *lenp) {
			if(put_user('\n', ((char __user *) buffer) + len))
				return -EFAULT;
			len++;
		}
		*lenp = len;
		*ppos += len;
	}
	return 0;
}

int proc_dostring(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return _proc_dostring(table->data, table->maxlen, write,
			filp, buffer, lenp, ppos);
}

/*
 *	Special case of dostring for the UTS structure. This has locks
 *	to observe. Should this be in kernel/sys.c ????
 */
 
#ifndef CONFIG_UTS_NS
static int proc_do_uts_string(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int r;

	if (!write) {
		down_read(&uts_sem);
		r=proc_dostring(table,0,filp,buffer,lenp, ppos);
		up_read(&uts_sem);
	} else {
		down_write(&uts_sem);
		r=proc_dostring(table,1,filp,buffer,lenp, ppos);
		up_write(&uts_sem);
	}
	return r;
}

static int sysctl_uts_string(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	int r;

	if (newval && newlen) {
		down_write(&uts_sem);
		r = sysctl_string(table, name, nlen,
				oldval, oldlenp, newval, newlen, context);
		up_write(&uts_sem);
	} else {
		down_read(&uts_sem);
		r = sysctl_string(table, name, nlen,
				oldval, oldlenp, newval, newlen, context);
		up_read(&uts_sem);
	}
	return rv;
}
#else /* !CONFIG_UTS_NS */
static char *choose_uts_string(int ctl_name)
{
	struct uts_namespace *uts_ns = current->nsproxy->uts_ns;

	switch (ctl_name) {
	case KERN_OSTYPE:
		return uts_ns->name.sysname;
	case KERN_NODENAME:
		return uts_ns->name.nodename;
	case KERN_OSRELEASE:
		return uts_ns->name.release;
	case KERN_VERSION:
		return uts_ns->name.version;
	case KERN_DOMAINNAME:
		return uts_ns->name.domainname;
	case KERN_VIRT_OSRELEASE:
		return virt_utsname.release;
	default:
		return NULL;
	}
}
static int proc_do_uts_string(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int r;
	char* which;

	which = choose_uts_string(table->ctl_name);
	if (!which)
		return -EINVAL;

	if (!write) {
		down_read(&uts_sem);
		r=_proc_dostring(which,table->maxlen,0,filp,buffer,lenp, ppos);
		up_read(&uts_sem);
	} else {
		down_write(&uts_sem);
		r=_proc_dostring(which,table->maxlen,1,filp,buffer,lenp, ppos);
		up_write(&uts_sem);
	}
	return r;
}

static int sysctl_uts_string(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	ctl_table tmp_table;
	char *which;
	int r;

	which = choose_uts_string(table->ctl_name);
	if (!which)
		return -EINVAL;

	tmp_table = *table;
	tmp_table.data = which;
	if (newval && newlen) {
		down_write(&uts_sem);
		r = sysctl_string(&tmp_table, name, nlen,
				oldval, oldlenp, newval, newlen, context);
		up_write(&uts_sem);
	} else {
		down_read(&uts_sem);
		r = sysctl_string(&tmp_table, name, nlen,
				oldval, oldlenp, newval, newlen, context);
		up_read(&uts_sem);
	}
	return r;
}
#endif /* !CONFIG_UTS_NS */

static int do_proc_dointvec_conv(int *negp, unsigned long *lvalp,
				 int *valp,
				 int write, void *data)
{
	if (write) {
		*valp = *negp ? -*lvalp : *lvalp;
	} else {
		int val = *valp;
		if (val < 0) {
			*negp = -1;
			*lvalp = (unsigned long)-val;
		} else {
			*negp = 0;
			*lvalp = (unsigned long)val;
		}
	}
	return 0;
}

static int __do_proc_dointvec(void *tbl_data, ctl_table *table,
		  int write, struct file *filp, void __user *buffer,
		  size_t *lenp, loff_t *ppos,
		  int (*conv)(int *negp, unsigned long *lvalp, int *valp,
			      int write, void *data),
		  void *data)
{
#define TMPBUFLEN 21
	int *i, vleft, first=1, neg, val;
	unsigned long lval;
	size_t left, len;
	
	char buf[TMPBUFLEN], *p;
	char __user *s = buffer;
	
	if (!tbl_data || !table->maxlen || !*lenp ||
	    (*ppos && !write)) {
		*lenp = 0;
		return 0;
	}
	
	i = (int *) tbl_data;
	vleft = table->maxlen / sizeof(*i);
	left = *lenp;

	if (!conv)
		conv = do_proc_dointvec_conv;

	for (; left && vleft--; i++, first=0) {
		if (write) {
			while (left) {
				char c;
				if (get_user(c, s))
					return -EFAULT;
				if (!isspace(c))
					break;
				left--;
				s++;
			}
			if (!left)
				break;
			neg = 0;
			len = left;
			if (len > sizeof(buf) - 1)
				len = sizeof(buf) - 1;
			if (copy_from_user(buf, s, len))
				return -EFAULT;
			buf[len] = 0;
			p = buf;
			if (*p == '-' && left > 1) {
				neg = 1;
				left--, p++;
			}
			if (*p < '0' || *p > '9')
				break;

			lval = simple_strtoul(p, &p, 0);

			len = p-buf;
			if ((len < left) && *p && !isspace(*p))
				break;
			if (neg)
				val = -val;
			s += len;
			left -= len;

			if (conv(&neg, &lval, i, 1, data))
				break;
		} else {
			p = buf;
			if (!first)
				*p++ = '\t';
	
			if (conv(&neg, &lval, i, 0, data))
				break;

			sprintf(p, "%s%lu", neg ? "-" : "", lval);
			len = strlen(buf);
			if (len > left)
				len = left;
			if(copy_to_user(s, buf, len))
				return -EFAULT;
			left -= len;
			s += len;
		}
	}

	if (!write && !first && left) {
		if(put_user('\n', s))
			return -EFAULT;
		left--, s++;
	}
	if (write) {
		while (left) {
			char c;
			if (get_user(c, s++))
				return -EFAULT;
			if (!isspace(c))
				break;
			left--;
		}
	}
	if (write && first)
		return -EINVAL;
	*lenp -= left;
	*ppos += *lenp;
	return 0;
#undef TMPBUFLEN
}

static int do_proc_dointvec(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos,
		  int (*conv)(int *negp, unsigned long *lvalp, int *valp,
			      int write, void *data),
		  void *data)
{
	return __do_proc_dointvec(table->data, table, write, filp,
			buffer, lenp, ppos, conv, data);
}

/**
 * proc_dointvec - read a vector of integers
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: file position
 *
 * Reads/writes up to table->maxlen/sizeof(unsigned int) integer
 * values from/to the user buffer, treated as an ASCII string. 
 *
 * Returns 0 on success.
 */
int proc_dointvec(ctl_table *table, int write, struct file *filp,
		     void __user *buffer, size_t *lenp, loff_t *ppos)
{
    return do_proc_dointvec(table,write,filp,buffer,lenp,ppos,
		    	    NULL,NULL);
}

#define OP_SET	0
#define OP_AND	1
#define OP_OR	2
#define OP_MAX	3
#define OP_MIN	4

static int do_proc_dointvec_bset_conv(int *negp, unsigned long *lvalp,
				      int *valp,
				      int write, void *data)
{
	int op = *(int *)data;
	if (write) {
		int val = *negp ? -*lvalp : *lvalp;
		switch(op) {
		case OP_SET:	*valp = val; break;
		case OP_AND:	*valp &= val; break;
		case OP_OR:	*valp |= val; break;
		case OP_MAX:	if(*valp < val)
					*valp = val;
				break;
		case OP_MIN:	if(*valp > val)
				*valp = val;
				break;
		}
	} else {
		int val = *valp;
		if (val < 0) {
			*negp = -1;
			*lvalp = (unsigned long)-val;
		} else {
			*negp = 0;
			*lvalp = (unsigned long)val;
		}
	}
	return 0;
}

/*
 *	init may raise the set.
 */
 
int proc_dointvec_bset(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int op;

	if (write && !capable(CAP_SYS_MODULE))
		return -EPERM;

	op = (current->pid == 1) ? OP_SET : OP_AND;
	return __do_proc_dointvec(&cap_bset, table, write, filp,
			buffer, lenp, ppos, do_proc_dointvec_bset_conv, &op);
}

int sysctl_strategy_bset(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context) {

	if (newval && !capable(CAP_SYS_MODULE))
		return -EPERM;

	return __do_sysctl_strategy (&cap_bset, table, name, nlen,
			oldval, oldlenp, newval, newlen, context);

	return 1;
}

struct do_proc_dointvec_minmax_conv_param {
	int *min;
	int *max;
};

static int do_proc_dointvec_minmax_conv(int *negp, unsigned long *lvalp, 
					int *valp, 
					int write, void *data)
{
	struct do_proc_dointvec_minmax_conv_param *param = data;
	if (write) {
		int val = *negp ? -*lvalp : *lvalp;
		if ((param->min && *param->min > val) ||
		    (param->max && *param->max < val))
			return -EINVAL;
		*valp = val;
	} else {
		int val = *valp;
		if (val < 0) {
			*negp = -1;
			*lvalp = (unsigned long)-val;
		} else {
			*negp = 0;
			*lvalp = (unsigned long)val;
		}
	}
	return 0;
}

/**
 * proc_dointvec_minmax - read a vector of integers with min/max values
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: file position
 *
 * Reads/writes up to table->maxlen/sizeof(unsigned int) integer
 * values from/to the user buffer, treated as an ASCII string.
 *
 * This routine will ensure the values are within the range specified by
 * table->extra1 (min) and table->extra2 (max).
 *
 * Returns 0 on success.
 */
int proc_dointvec_minmax(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	struct do_proc_dointvec_minmax_conv_param param = {
		.min = (int *) table->extra1,
		.max = (int *) table->extra2,
	};
	return do_proc_dointvec(table, write, filp, buffer, lenp, ppos,
				do_proc_dointvec_minmax_conv, &param);
}

static int __do_proc_doulongvec_minmax(void *data, ctl_table *table, int write,
				     struct file *filp,
				     void __user *buffer,
				     size_t *lenp, loff_t *ppos,
				     unsigned long convmul,
				     unsigned long convdiv)
{
#define TMPBUFLEN 21
	unsigned long *i, *min, *max, val;
	int vleft, first=1, neg;
	size_t len, left;
	char buf[TMPBUFLEN], *p;
	char __user *s = buffer;
	
	if (!data || !table->maxlen || !*lenp ||
	    (*ppos && !write)) {
		*lenp = 0;
		return 0;
	}
	
	i = (unsigned long *) data;
	min = (unsigned long *) table->extra1;
	max = (unsigned long *) table->extra2;
	vleft = table->maxlen / sizeof(unsigned long);
	left = *lenp;
	
	for (; left && vleft--; i++, min++, max++, first=0) {
		if (write) {
			while (left) {
				char c;
				if (get_user(c, s))
					return -EFAULT;
				if (!isspace(c))
					break;
				left--;
				s++;
			}
			if (!left)
				break;
			neg = 0;
			len = left;
			if (len > TMPBUFLEN-1)
				len = TMPBUFLEN-1;
			if (copy_from_user(buf, s, len))
				return -EFAULT;
			buf[len] = 0;
			p = buf;
			if (*p == '-' && left > 1) {
				neg = 1;
				left--, p++;
			}
			if (*p < '0' || *p > '9')
				break;
			val = simple_strtoul(p, &p, 0) * convmul / convdiv ;
			len = p-buf;
			if ((len < left) && *p && !isspace(*p))
				break;
			if (neg)
				val = -val;
			s += len;
			left -= len;

			if(neg)
				continue;
			if ((min && val < *min) || (max && val > *max))
				continue;
			*i = val;
		} else {
			p = buf;
			if (!first)
				*p++ = '\t';
			sprintf(p, "%lu", convdiv * (*i) / convmul);
			len = strlen(buf);
			if (len > left)
				len = left;
			if(copy_to_user(s, buf, len))
				return -EFAULT;
			left -= len;
			s += len;
		}
	}

	if (!write && !first && left) {
		if(put_user('\n', s))
			return -EFAULT;
		left--, s++;
	}
	if (write) {
		while (left) {
			char c;
			if (get_user(c, s++))
				return -EFAULT;
			if (!isspace(c))
				break;
			left--;
		}
	}
	if (write && first)
		return -EINVAL;
	*lenp -= left;
	*ppos += *lenp;
	return 0;
#undef TMPBUFLEN
}

static int do_proc_doulongvec_minmax(ctl_table *table, int write,
				     struct file *filp,
				     void __user *buffer,
				     size_t *lenp, loff_t *ppos,
				     unsigned long convmul,
				     unsigned long convdiv)
{
	return __do_proc_doulongvec_minmax(table->data, table, write,
			filp, buffer, lenp, ppos, convmul, convdiv);
}

/**
 * proc_doulongvec_minmax - read a vector of long integers with min/max values
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: file position
 *
 * Reads/writes up to table->maxlen/sizeof(unsigned long) unsigned long
 * values from/to the user buffer, treated as an ASCII string.
 *
 * This routine will ensure the values are within the range specified by
 * table->extra1 (min) and table->extra2 (max).
 *
 * Returns 0 on success.
 */
int proc_doulongvec_minmax(ctl_table *table, int write, struct file *filp,
			   void __user *buffer, size_t *lenp, loff_t *ppos)
{
    return do_proc_doulongvec_minmax(table, write, filp, buffer, lenp, ppos, 1l, 1l);
}

/**
 * proc_doulongvec_ms_jiffies_minmax - read a vector of millisecond values with min/max values
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: file position
 *
 * Reads/writes up to table->maxlen/sizeof(unsigned long) unsigned long
 * values from/to the user buffer, treated as an ASCII string. The values
 * are treated as milliseconds, and converted to jiffies when they are stored.
 *
 * This routine will ensure the values are within the range specified by
 * table->extra1 (min) and table->extra2 (max).
 *
 * Returns 0 on success.
 */
int proc_doulongvec_ms_jiffies_minmax(ctl_table *table, int write,
				      struct file *filp,
				      void __user *buffer,
				      size_t *lenp, loff_t *ppos)
{
    return do_proc_doulongvec_minmax(table, write, filp, buffer,
				     lenp, ppos, HZ, 1000l);
}


static int do_proc_dointvec_jiffies_conv(int *negp, unsigned long *lvalp,
					 int *valp,
					 int write, void *data)
{
	if (write) {
		if (*lvalp > LONG_MAX / HZ)
			return 1;
		*valp = *negp ? -(*lvalp*HZ) : (*lvalp*HZ);
	} else {
		int val = *valp;
		unsigned long lval;
		if (val < 0) {
			*negp = -1;
			lval = (unsigned long)-val;
		} else {
			*negp = 0;
			lval = (unsigned long)val;
		}
		*lvalp = lval / HZ;
	}
	return 0;
}

static int do_proc_dointvec_userhz_jiffies_conv(int *negp, unsigned long *lvalp,
						int *valp,
						int write, void *data)
{
	if (write) {
		if (USER_HZ < HZ && *lvalp > (LONG_MAX / HZ) * USER_HZ)
			return 1;
		*valp = clock_t_to_jiffies(*negp ? -*lvalp : *lvalp);
	} else {
		int val = *valp;
		unsigned long lval;
		if (val < 0) {
			*negp = -1;
			lval = (unsigned long)-val;
		} else {
			*negp = 0;
			lval = (unsigned long)val;
		}
		*lvalp = jiffies_to_clock_t(lval);
	}
	return 0;
}

static int do_proc_dointvec_ms_jiffies_conv(int *negp, unsigned long *lvalp,
					    int *valp,
					    int write, void *data)
{
	if (write) {
		*valp = msecs_to_jiffies(*negp ? -*lvalp : *lvalp);
	} else {
		int val = *valp;
		unsigned long lval;
		if (val < 0) {
			*negp = -1;
			lval = (unsigned long)-val;
		} else {
			*negp = 0;
			lval = (unsigned long)val;
		}
		*lvalp = jiffies_to_msecs(lval);
	}
	return 0;
}

/**
 * proc_dointvec_jiffies - read a vector of integers as seconds
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: file position
 *
 * Reads/writes up to table->maxlen/sizeof(unsigned int) integer
 * values from/to the user buffer, treated as an ASCII string. 
 * The values read are assumed to be in seconds, and are converted into
 * jiffies.
 *
 * Returns 0 on success.
 */
int proc_dointvec_jiffies(ctl_table *table, int write, struct file *filp,
			  void __user *buffer, size_t *lenp, loff_t *ppos)
{
    return do_proc_dointvec(table,write,filp,buffer,lenp,ppos,
		    	    do_proc_dointvec_jiffies_conv,NULL);
}

/**
 * proc_dointvec_userhz_jiffies - read a vector of integers as 1/USER_HZ seconds
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: pointer to the file position
 *
 * Reads/writes up to table->maxlen/sizeof(unsigned int) integer
 * values from/to the user buffer, treated as an ASCII string. 
 * The values read are assumed to be in 1/USER_HZ seconds, and 
 * are converted into jiffies.
 *
 * Returns 0 on success.
 */
int proc_dointvec_userhz_jiffies(ctl_table *table, int write, struct file *filp,
				 void __user *buffer, size_t *lenp, loff_t *ppos)
{
    return do_proc_dointvec(table,write,filp,buffer,lenp,ppos,
		    	    do_proc_dointvec_userhz_jiffies_conv,NULL);
}

/**
 * proc_dointvec_ms_jiffies - read a vector of integers as 1 milliseconds
 * @table: the sysctl table
 * @write: %TRUE if this is a write to the sysctl file
 * @filp: the file structure
 * @buffer: the user buffer
 * @lenp: the size of the user buffer
 * @ppos: file position
 * @ppos: the current position in the file
 *
 * Reads/writes up to table->maxlen/sizeof(unsigned int) integer
 * values from/to the user buffer, treated as an ASCII string. 
 * The values read are assumed to be in 1/1000 seconds, and 
 * are converted into jiffies.
 *
 * Returns 0 on success.
 */
int proc_dointvec_ms_jiffies(ctl_table *table, int write, struct file *filp,
			     void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return do_proc_dointvec(table, write, filp, buffer, lenp, ppos,
				do_proc_dointvec_ms_jiffies_conv, NULL);
}

#ifdef CONFIG_SYSVIPC
static int proc_do_ipc_string(ctl_table *table, int write, struct file *filp,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	void *data;
	struct ipc_namespace *ns;

	ns = current->nsproxy->ipc_ns;

	switch (table->ctl_name) {
	case KERN_SHMMAX:
		data = &ns->shm_ctlmax;
		goto proc_minmax;
	case KERN_SHMALL:
		data = &ns->shm_ctlall;
		goto proc_minmax;
	case KERN_SHMMNI:
		data = &ns->shm_ctlmni;
		break;
	case KERN_MSGMAX:
		data = &ns->msg_ctlmax;
		break;
	case KERN_MSGMNI:
		data = &ns->msg_ctlmni;
		break;
	case KERN_MSGMNB:
		data = &ns->msg_ctlmnb;
		break;
	case KERN_SEM:
		data = &ns->sem_ctls;
		break;
	default:
		return -EINVAL;
	}

	return __do_proc_dointvec(data, table, write, filp, buffer,
			lenp, ppos, NULL, NULL);
proc_minmax:
	return __do_proc_doulongvec_minmax(data, table, write, filp, buffer,
			lenp, ppos, 1l, 1l);
}
#endif

#else /* CONFIG_PROC_FS */

int proc_dostring(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

static int proc_do_uts_string(ctl_table *table, int write, struct file *filp,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

static int sysctl_uts_string(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	return -ENOSYS;
}

#ifdef CONFIG_SYSVIPC
static int proc_do_ipc_string(ctl_table *table, int write, struct file *filp,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}
#endif

int proc_dointvec(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_bset(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_minmax(ctl_table *table, int write, struct file *filp,
		    void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_jiffies(ctl_table *table, int write, struct file *filp,
		    void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_userhz_jiffies(ctl_table *table, int write, struct file *filp,
		    void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_ms_jiffies(ctl_table *table, int write, struct file *filp,
			     void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_doulongvec_minmax(ctl_table *table, int write, struct file *filp,
		    void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_doulongvec_ms_jiffies_minmax(ctl_table *table, int write,
				      struct file *filp,
				      void __user *buffer,
				      size_t *lenp, loff_t *ppos)
{
    return -ENOSYS;
}


#endif /* CONFIG_PROC_FS */


/*
 * General sysctl support routines 
 */

/* The generic string strategy routine: */
int sysctl_string(ctl_table *table, int __user *name, int nlen,
		  void __user *oldval, size_t __user *oldlenp,
		  void __user *newval, size_t newlen, void **context)
{
	if (!table->data || !table->maxlen) 
		return -ENOTDIR;
	
	if (oldval && oldlenp) {
		size_t bufsize;
		if (get_user(bufsize, oldlenp))
			return -EFAULT;
		if (bufsize) {
			size_t len = strlen(table->data), copied;

			/* This shouldn't trigger for a well-formed sysctl */
			if (len > table->maxlen)
				len = table->maxlen;

			/* Copy up to a max of bufsize-1 bytes of the string */
			copied = (len >= bufsize) ? bufsize - 1 : len;

			if (copy_to_user(oldval, table->data, copied) ||
			    put_user(0, (char __user *)(oldval + copied)))
				return -EFAULT;
			if (put_user(len, oldlenp))
				return -EFAULT;
		}
	}
	if (newval && newlen) {
		size_t len = newlen;
		if (len > table->maxlen)
			len = table->maxlen;
		if(copy_from_user(table->data, newval, len))
			return -EFAULT;
		if (len == table->maxlen)
			len--;
		((char *) table->data)[len] = 0;
	}
	return 1;
}

/*
 * This function makes sure that all of the integers in the vector
 * are between the minimum and maximum values given in the arrays
 * table->extra1 and table->extra2, respectively.
 */
int sysctl_intvec(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{

	if (newval && newlen) {
		int __user *vec = (int __user *) newval;
		int *min = (int *) table->extra1;
		int *max = (int *) table->extra2;
		size_t length;
		int i;

		if (newlen % sizeof(int) != 0)
			return -EINVAL;

		if (!table->extra1 && !table->extra2)
			return 0;

		if (newlen > table->maxlen)
			newlen = table->maxlen;
		length = newlen / sizeof(int);

		for (i = 0; i < length; i++) {
			int value;
			if (get_user(value, vec + i))
				return -EFAULT;
			if (min && value < min[i])
				return -EINVAL;
			if (max && value > max[i])
				return -EINVAL;
		}
	}
	return 0;
}

/* Strategy function to convert jiffies to seconds */ 
int sysctl_jiffies(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	if (oldval) {
		size_t olen;
		if (oldlenp) { 
			if (get_user(olen, oldlenp))
				return -EFAULT;
			if (olen!=sizeof(int))
				return -EINVAL; 
		}
		if (put_user(*(int *)(table->data)/HZ, (int __user *)oldval) ||
		    (oldlenp && put_user(sizeof(int),oldlenp)))
			return -EFAULT;
	}
	if (newval && newlen) { 
		int new;
		if (newlen != sizeof(int))
			return -EINVAL; 
		if (get_user(new, (int __user *)newval))
			return -EFAULT;
		*(int *)(table->data) = new*HZ; 
	}
	return 1;
}

/* Strategy function to convert jiffies to seconds */ 
int sysctl_ms_jiffies(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	if (oldval) {
		size_t olen;
		if (oldlenp) { 
			if (get_user(olen, oldlenp))
				return -EFAULT;
			if (olen!=sizeof(int))
				return -EINVAL; 
		}
		if (put_user(jiffies_to_msecs(*(int *)(table->data)), (int __user *)oldval) ||
		    (oldlenp && put_user(sizeof(int),oldlenp)))
			return -EFAULT;
	}
	if (newval && newlen) { 
		int new;
		if (newlen != sizeof(int))
			return -EINVAL; 
		if (get_user(new, (int __user *)newval))
			return -EFAULT;
		*(int *)(table->data) = msecs_to_jiffies(new);
	}
	return 1;
}

#else /* CONFIG_SYSCTL */


asmlinkage long sys_sysctl(struct __sysctl_args __user *args)
{
	return -ENOSYS;
}

int sysctl_string(ctl_table *table, int __user *name, int nlen,
		  void __user *oldval, size_t __user *oldlenp,
		  void __user *newval, size_t newlen, void **context)
{
	return -ENOSYS;
}

int sysctl_intvec(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	return -ENOSYS;
}

int sysctl_jiffies(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	return -ENOSYS;
}

int sysctl_ms_jiffies(ctl_table *table, int __user *name, int nlen,
		void __user *oldval, size_t __user *oldlenp,
		void __user *newval, size_t newlen, void **context)
{
	return -ENOSYS;
}

int proc_dostring(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec(ctl_table *table, int write, struct file *filp,
		  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_bset(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_minmax(ctl_table *table, int write, struct file *filp,
		    void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_jiffies(ctl_table *table, int write, struct file *filp,
			  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_userhz_jiffies(ctl_table *table, int write, struct file *filp,
			  void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_dointvec_ms_jiffies(ctl_table *table, int write, struct file *filp,
			     void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_doulongvec_minmax(ctl_table *table, int write, struct file *filp,
		    void __user *buffer, size_t *lenp, loff_t *ppos)
{
	return -ENOSYS;
}

int proc_doulongvec_ms_jiffies_minmax(ctl_table *table, int write,
				      struct file *filp,
				      void __user *buffer,
				      size_t *lenp, loff_t *ppos)
{
    return -ENOSYS;
}

struct ctl_table_header * register_sysctl_table(ctl_table * table, 
						int insert_at_head)
{
	return NULL;
}

void unregister_sysctl_table(struct ctl_table_header * table)
{
}

ctl_table * clone_sysctl_template(ctl_table *tmpl, int nr)
{
	return NULL;
}

void free_sysctl_clone(ctl_table *tmpl)
{
}
#endif /* CONFIG_SYSCTL */

/*
 * No sense putting this after each symbol definition, twice,
 * exception granted :-)
 */
EXPORT_SYMBOL(proc_dointvec);
EXPORT_SYMBOL(proc_dointvec_jiffies);
EXPORT_SYMBOL(proc_dointvec_minmax);
EXPORT_SYMBOL(proc_dointvec_userhz_jiffies);
EXPORT_SYMBOL(proc_dointvec_ms_jiffies);
EXPORT_SYMBOL(proc_dostring);
EXPORT_SYMBOL(proc_doulongvec_minmax);
EXPORT_SYMBOL(proc_doulongvec_ms_jiffies_minmax);
EXPORT_SYMBOL(register_sysctl_table);
EXPORT_SYMBOL(sysctl_intvec);
EXPORT_SYMBOL(sysctl_jiffies);
EXPORT_SYMBOL(sysctl_ms_jiffies);
EXPORT_SYMBOL(sysctl_string);
EXPORT_SYMBOL(unregister_sysctl_table);
EXPORT_SYMBOL(clone_sysctl_template);
EXPORT_SYMBOL(free_sysctl_clone);
