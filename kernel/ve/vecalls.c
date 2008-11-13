/*
 *  linux/kernel/ve/vecalls.c
 *
 *  Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *
 */

/*
 * 'vecalls.c' is file with basic VE support. It provides basic primities
 * along with initialization script
 */

#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/capability.h>
#include <linux/ve.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/sys.h>
#include <linux/fs.h>
#include <linux/namespace.h>
#include <linux/termios.h>
#include <linux/tty_driver.h>
#include <linux/netdevice.h>
#include <linux/wait.h>
#include <linux/inetdevice.h>
#include <net/addrconf.h>
#include <linux/utsname.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/devpts_fs.h>
#include <linux/shmem_fs.h>
#include <linux/sysfs.h>
#include <linux/seq_file.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/rcupdate.h>
#include <linux/in.h>
#include <linux/idr.h>
#include <linux/inetdevice.h>
#include <linux/pid.h>
#include <net/pkt_sched.h>
#include <linux/divert.h>
#include <ub/beancounter.h>
#include <linux/nsproxy.h>
#include <linux/kobject.h>

#include <net/route.h>
#include <net/ip_fib.h>
#include <net/ip6_route.h>
#include <net/arp.h>
#include <net/ipv6.h>

#include <linux/ve_proto.h>
#include <linux/venet.h>
#include <linux/vzctl.h>
#include <linux/vzcalluser.h>
#ifdef CONFIG_FAIRSCHED
#include <linux/fairsched.h>
#endif

#include <linux/nfcalls.h>
#include <linux/virtinfo.h>
#include <linux/utsrelease.h>
#include <linux/major.h>

int nr_ve = 1;	/* One VE always exists. Compatibility with vestat */
EXPORT_SYMBOL(nr_ve);

static int	do_env_enter(struct ve_struct *ve, unsigned int flags);
static int	alloc_ve_tty_drivers(struct ve_struct* ve);
static void	free_ve_tty_drivers(struct ve_struct* ve);
static int	register_ve_tty_drivers(struct ve_struct* ve);
static void	unregister_ve_tty_drivers(struct ve_struct* ve);
static int	init_ve_tty_drivers(struct ve_struct *);
static void	fini_ve_tty_drivers(struct ve_struct *);
static void	clear_termios(struct tty_driver* driver );
#ifdef CONFIG_INET
static void	ve_mapped_devs_cleanup(struct ve_struct *ve);
#endif
static void	ve_del_ipv6_addrs(struct net_device *dev);

static void vecalls_exit(void);
extern void grsecurity_setup(void);

struct ve_struct *__find_ve_by_id(envid_t veid)
{
	struct ve_struct *ve;

	for_each_ve(ve) {
		if (ve->veid == veid)
			return ve;
	}
	return NULL;
}
EXPORT_SYMBOL(__find_ve_by_id);

struct ve_struct *get_ve_by_id(envid_t veid)
{
	struct ve_struct *ve;
	read_lock(&ve_list_lock);
	ve = __find_ve_by_id(veid);
	get_ve(ve);
	read_unlock(&ve_list_lock);
	return ve;
}
EXPORT_SYMBOL(get_ve_by_id);

/*
 * real_put_ve() MUST be used instead of put_ve() inside vecalls.
 */
void real_do_env_free(struct ve_struct *ve);
static inline void real_put_ve(struct ve_struct *ve)
{
	if (ve && atomic_dec_and_test(&ve->counter)) {
		if (atomic_read(&ve->pcounter) > 0)
			BUG();
		if (ve->is_running)
			BUG();
		real_do_env_free(ve);
	}
}

static int ve_get_cpu_stat(envid_t veid, struct vz_cpu_stat __user *buf)
{
	struct ve_struct *ve;
	struct vz_cpu_stat *vstat;
	int retval;
	int i, cpu;
	unsigned long tmp;

	if (!ve_is_super(get_exec_env()) && (veid != get_exec_env()->veid))
		return -EPERM;
	if (veid == 0)
		return -ESRCH;

	vstat = kzalloc(sizeof(*vstat), GFP_KERNEL);
	if (!vstat)
		return -ENOMEM;
	
	retval = -ESRCH;
	read_lock(&ve_list_lock);
	ve = __find_ve_by_id(veid);
	if (ve == NULL)
		goto out_unlock;
	for_each_online_cpu(cpu) {
		struct ve_cpu_stats *st;

		st = VE_CPU_STATS(ve, cpu);
		vstat->user_jif += (unsigned long)cputime64_to_clock_t(st->user);
		vstat->nice_jif += (unsigned long)cputime64_to_clock_t(st->nice);
		vstat->system_jif += (unsigned long)cputime64_to_clock_t(st->system);
		vstat->idle_clk += __ve_sched_get_idle_time(ve, cpu);
	}
	vstat->uptime_clk = get_cycles() - ve->start_cycles;
	vstat->uptime_jif = (unsigned long)cputime64_to_clock_t(
				get_jiffies_64() - ve->start_jiffies);
	for (i = 0; i < 3; i++) {
		tmp = ve->avenrun[i] + (FIXED_1/200);
		vstat->avenrun[i].val_int = LOAD_INT(tmp);
		vstat->avenrun[i].val_frac = LOAD_FRAC(tmp);
	}
	read_unlock(&ve_list_lock);

	retval = 0;
	if (copy_to_user(buf, vstat, sizeof(*vstat)))
		retval = -EFAULT;
out_free:
	kfree(vstat);
	return retval;

out_unlock:
	read_unlock(&ve_list_lock);
	goto out_free;
}

static int real_setdevperms(envid_t veid, unsigned type,
		dev_t dev, unsigned mask)
{
	struct ve_struct *ve;
	int err;

	if (!capable(CAP_SETVEID) || veid == 0)
		return -EPERM;

	if ((ve = get_ve_by_id(veid)) == NULL)
		return -ESRCH;

	down_read(&ve->op_sem);
	err = -ESRCH;
	if (ve->is_running)
		err = set_device_perms_ve(veid, type, dev, mask);
	up_read(&ve->op_sem);
	real_put_ve(ve);
	return err;
}

/**********************************************************************
 **********************************************************************
 *
 * FS-related helpers to VE start/stop
 *
 **********************************************************************
 **********************************************************************/

#ifdef CONFIG_SYSCTL
static inline int register_ve_sysctltables(struct ve_struct *ve)
{
	/*
	 * a code to register kernel sysctl table used to be here
	 * it registered utsname and ipc ones only, but since
	 * we have namespaves for them booth nothing is to be done
	 * here.
	 */
	return 0;
}

static inline void unregister_ve_sysctltables(struct ve_struct *ve)
{
}

static inline void free_ve_sysctltables(struct ve_struct *ve)
{
}
#endif

/**********************************************************************
 **********************************************************************
 *
 * VE start: subsystems
 *
 **********************************************************************
 **********************************************************************/

#ifdef CONFIG_INET
#include <net/ip.h>
#include <net/tcp.h>
#include <net/udp.h>
#include <net/icmp.h>

#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
static int init_fini_ve_mibs6(struct ve_struct *ve, int fini)
{
	if (fini)
		goto fini;

	if (!(ve->_ipv6_statistics[0] = alloc_percpu(struct ipstats_mib)))
		goto out1;
	if (!(ve->_ipv6_statistics[1] = alloc_percpu(struct ipstats_mib)))
		goto out2;
	if (!(ve->_icmpv6_statistics[0] = alloc_percpu(struct icmpv6_mib)))
		goto out3;
	if (!(ve->_icmpv6_statistics[1] = alloc_percpu(struct icmpv6_mib)))
		goto out4;
	if (!(ve->_udp_stats_in6[0] = alloc_percpu(struct udp_mib)))
		goto out5;
	if (!(ve->_udp_stats_in6[1] = alloc_percpu(struct udp_mib)))
		goto out6;
	return 0;

fini:
	free_percpu(ve->_udp_stats_in6[1]);
out6:
	free_percpu(ve->_udp_stats_in6[0]);
out5:
	free_percpu(ve->_icmpv6_statistics[1]);
out4:
	free_percpu(ve->_icmpv6_statistics[0]);
out3:
	free_percpu(ve->_ipv6_statistics[1]);
out2:
	free_percpu(ve->_ipv6_statistics[0]);
out1:
	return -ENOMEM;
}
#else
static int init_fini_ve_mibs6(struct ve_struct *ve, int fini) { return 0; }
#endif

static int init_fini_ve_mibs(struct ve_struct *ve, int fini)
{
	if (fini)
		goto fini;

	if (!(ve->_net_statistics[0] = alloc_percpu(struct linux_mib)))
		goto out1;
	if (!(ve->_net_statistics[1] = alloc_percpu(struct linux_mib)))
		goto out2;
	if (!(ve->_ip_statistics[0] = alloc_percpu(struct ipstats_mib)))
		goto out3;
	if (!(ve->_ip_statistics[1] = alloc_percpu(struct ipstats_mib)))
		goto out4;
	if (!(ve->_icmp_statistics[0] = alloc_percpu(struct icmp_mib)))
		goto out5;
	if (!(ve->_icmp_statistics[1] = alloc_percpu(struct icmp_mib)))
		goto out6;
	if (!(ve->_tcp_statistics[0] = alloc_percpu(struct tcp_mib)))
		goto out7;
	if (!(ve->_tcp_statistics[1] = alloc_percpu(struct tcp_mib)))
		goto out8;
	if (!(ve->_udp_statistics[0] = alloc_percpu(struct udp_mib)))
		goto out9;
	if (!(ve->_udp_statistics[1] = alloc_percpu(struct udp_mib)))
		goto out10;
	if (init_fini_ve_mibs6(ve, fini))
		goto out11;
	return 0;

fini:
	init_fini_ve_mibs6(ve, fini);
out11:
	free_percpu(ve->_udp_statistics[1]);
out10:
	free_percpu(ve->_udp_statistics[0]);
out9:
	free_percpu(ve->_tcp_statistics[1]);
out8:
	free_percpu(ve->_tcp_statistics[0]);
out7:
	free_percpu(ve->_icmp_statistics[1]);
out6:
	free_percpu(ve->_icmp_statistics[0]);
out5:
	free_percpu(ve->_ip_statistics[1]);
out4:
	free_percpu(ve->_ip_statistics[0]);
out3:
	free_percpu(ve->_net_statistics[1]);
out2:
	free_percpu(ve->_net_statistics[0]);
out1:
	return -ENOMEM;
}

static inline int init_ve_mibs(struct ve_struct *ve)
{
	return init_fini_ve_mibs(ve, 0);
}

static inline void fini_ve_mibs(struct ve_struct *ve)
{
	(void)init_fini_ve_mibs(ve, 1);
}

static void veloop_setup(struct net_device *dev)
{
	int padded;
	padded = dev->padded;
	memcpy(dev, &templ_loopback_dev, sizeof(struct net_device));
	dev->padded = padded;
}

static int init_ve_netdev(void)
{
	struct ve_struct *ve;
	struct net_device_stats *stats;
	int err;

	ve = get_exec_env();
	INIT_HLIST_HEAD(&ve->_net_dev_head);
	ve->_net_dev_base = NULL;
	ve->_net_dev_tail = &ve->_net_dev_base;

	err = -ENOMEM;
	ve->_loopback_dev = alloc_netdev(0, templ_loopback_dev.name, 
					 veloop_setup);
	if (ve->_loopback_dev == NULL)
		goto out;

	ve->_loopback_stats = alloc_percpu(struct net_device_stats);
	if (ve->_loopback_stats == NULL)
		goto out_free_netdev;
	stats = kzalloc(sizeof(struct net_device_stats), GFP_KERNEL);
	if (stats != NULL)
		ve->_loopback_dev->priv = stats;
	err = register_netdev(ve->_loopback_dev);
	if (err)
		goto out_free_stats;
	return 0;

out_free_stats:
	if (ve->_loopback_dev->priv != NULL)
		kfree(ve->_loopback_dev->priv);
	free_percpu(ve->_loopback_stats);
out_free_netdev:
	free_netdev(ve->_loopback_dev);
out:
	return err;
}

static void fini_ve_netdev(void)
{
	struct ve_struct *ve;
	struct net_device *dev;

	ve = get_exec_env();
	while (1) {
		rtnl_lock();
		/* 
		 * loopback is special, it can be referenced in  fib's, 
		 * so it must be freed the last. Doing so is 
		 * sufficient to guarantee absence of such references.
		 */
		if (dev_base == ve->_loopback_dev)
			dev = dev_base->next;
		else
			dev = dev_base;
		if (dev == NULL)
			break;
		unregister_netdevice(dev);
		rtnl_unlock();
		free_netdev(dev);
	}
	ve_del_ipv6_addrs(ve->_loopback_dev);
	unregister_netdevice(ve->_loopback_dev);
	rtnl_unlock();
	free_netdev(ve->_loopback_dev);
	ve->_loopback_dev = NULL;

	free_percpu(ve->_loopback_stats);
	ve->_loopback_stats = NULL;
}
#else
#define init_ve_mibs(ve)	(0)
#define fini_ve_mibs(ve)	do { } while (0)
#define init_ve_netdev()	(0)
#define fini_ve_netdev()	do { } while (0)
#endif

static int prepare_proc_root(struct ve_struct *ve)
{
	struct proc_dir_entry *de;

	de = kzalloc(sizeof(struct proc_dir_entry) + 6, GFP_KERNEL);
	if (de == NULL)
		return -ENOMEM;

	memcpy(de + 1, "/proc", 6);
	de->name = (char *)(de + 1);
	de->namelen = 5;
	de->mode = S_IFDIR | S_IRUGO | S_IXUGO;
	de->nlink = 2;
	atomic_set(&de->count, 1);

	ve->proc_root = de;
	return 0;
}

#ifdef CONFIG_PROC_FS
static int init_ve_proc(struct ve_struct *ve)
{
	int err;
	struct proc_dir_entry *de;

	err = prepare_proc_root(ve);
	if (err)
		goto out_root;

	err = register_ve_fs_type(ve, &proc_fs_type,
			&ve->proc_fstype, &ve->proc_mnt);
	if (err)
		goto out_reg;

	err = -ENOMEM;
	de = create_proc_entry("kmsg", S_IRUSR, NULL);
	if (!de)
		goto out_kmsg;
	de->proc_fops = &proc_kmsg_operations;

	/* create necessary /proc subdirs in VE local proc tree */
	err = -ENOMEM;
	de = create_proc_entry("vz", S_IFDIR|S_IRUGO|S_IXUGO, NULL);
	if (!de)
		goto out_vz;

	proc_net = proc_mkdir("net", NULL);
	if (!proc_net)
		goto out_net;

	if (ve_snmp_proc_init())
		goto out_snmp;

	return 0;

out_snmp:
	remove_proc_entry("net", NULL);
out_net:
	remove_proc_entry("vz", NULL);
out_vz:
	remove_proc_entry("kmsg", NULL);
out_kmsg:
	unregister_ve_fs_type(ve->proc_fstype, ve->proc_mnt);
	ve->proc_mnt = NULL;
out_reg:
	/* proc_fstype and proc_root are freed in real_put_ve -> free_ve_proc */
	;
out_root:
	return err;
}

static void fini_ve_proc(struct ve_struct *ve)
{
	ve_snmp_proc_fini();
	remove_proc_entry("net", NULL);
	proc_net =  NULL;
	remove_proc_entry("vz", NULL);
	remove_proc_entry("kmsg", NULL);
	unregister_ve_fs_type(ve->proc_fstype, ve->proc_mnt);
	ve->proc_mnt = NULL;
}

static void free_ve_proc(struct ve_struct *ve)
{
	/* proc filesystem frees proc_dir_entries on remove_proc_entry() only,
	   so we check that everything was removed and not lost */
	if (ve->proc_root && ve->proc_root->subdir) {
		struct proc_dir_entry *p = ve->proc_root;
		printk(KERN_WARNING "CT: %d: proc entry /proc", ve->veid);
		while ((p = p->subdir) != NULL)
			printk("/%s", p->name);
		printk(" is not removed!\n");
	}

	kfree(ve->proc_root);
	kfree(ve->proc_fstype);

	ve->proc_fstype = NULL;
	ve->proc_root = NULL;
}
#else
#define init_ve_proc(ve)	(0)
#define fini_ve_proc(ve)	do { } while (0)
#define free_ve_proc(ve)	do { } while (0)
#endif

#ifdef CONFIG_SYSCTL
static int init_ve_sysctl(struct ve_struct *ve)
{
	int err;

#ifdef CONFIG_PROC_FS
	err = -ENOMEM;
	ve->proc_sys_root = proc_mkdir("sys", NULL);
	if (ve->proc_sys_root == NULL)
		goto out_proc;
#endif
	INIT_LIST_HEAD(&ve->sysctl_lh);
	err = register_ve_sysctltables(ve);
	if (err)
		goto out_reg;

	err = devinet_sysctl_init(ve);
	if (err)
		goto out_dev;

	err = addrconf_sysctl_init(ve);
	if (err)
		goto out_dev6;

	return 0;

out_dev6:
	devinet_sysctl_fini(ve);
out_dev:
	unregister_ve_sysctltables(ve);
	free_ve_sysctltables(ve);
out_reg:
#ifdef CONFIG_PROC_FS
	remove_proc_entry("sys", NULL);
out_proc:
#endif
	return err;
}

static void fini_ve_sysctl(struct ve_struct *ve)
{
	addrconf_sysctl_fini(ve);
	devinet_sysctl_fini(ve);
	unregister_ve_sysctltables(ve);
	remove_proc_entry("sys", NULL);
}

static void free_ve_sysctl(struct ve_struct *ve)
{
	addrconf_sysctl_free(ve);
	devinet_sysctl_free(ve);
	free_ve_sysctltables(ve);
}
#else
#define init_ve_sysctl(ve)	(0)
#define fini_ve_sysctl(ve)	do { } while (0)
#define free_ve_sysctl(ve)	do { } while (0)
#endif

#ifdef CONFIG_UNIX98_PTYS
#include <linux/devpts_fs.h>

/*
 * DEVPTS needs a virtualization: each environment should see each own list of
 * pseudo-terminals.
 * To implement it we need to have separate devpts superblocks for each
 * VE, and each VE should mount its own one.
 * Thus, separate vfsmount structures are required.
 * To minimize intrusion into vfsmount lookup code, separate file_system_type
 * structures are created.
 *
 * In addition to this, patch fo character device itself is required, as file
 * system itself is used only for MINOR/MAJOR lookup.
 */

static int init_ve_devpts(struct ve_struct *ve)
{
	int err;

	err = -ENOMEM;
	ve->devpts_config = kzalloc(sizeof(struct devpts_config), GFP_KERNEL);
	if (ve->devpts_config == NULL)
		goto out;

	ve->devpts_config->mode = 0600;
	err = register_ve_fs_type(ve, &devpts_fs_type,
			&ve->devpts_fstype, &ve->devpts_mnt);
	if (err) {
		kfree(ve->devpts_config);
		ve->devpts_config = NULL;
	}
out:
	return err;
}

static void fini_ve_devpts(struct ve_struct *ve)
{
	unregister_ve_fs_type(ve->devpts_fstype, ve->devpts_mnt);
	/* devpts_fstype is freed in real_put_ve -> free_ve_filesystems */
	ve->devpts_mnt = NULL;
	kfree(ve->devpts_config);
	ve->devpts_config = NULL;
}
#else
#define init_ve_devpts(ve)	(0)
#define fini_ve_devpts(ve)	do { } while (0)
#endif

static int init_ve_shmem(struct ve_struct *ve)
{
	return register_ve_fs_type(ve,
				   &tmpfs_fs_type,
				   &ve->shmem_fstype,
				   &ve->shmem_mnt);
}

static void fini_ve_shmem(struct ve_struct *ve)
{
	unregister_ve_fs_type(ve->shmem_fstype, ve->shmem_mnt);
	/* shmem_fstype is freed in real_put_ve -> free_ve_filesystems */
	ve->shmem_mnt = NULL;
}

static inline int init_ve_sysfs_root(struct ve_struct *ve)
{
	struct sysfs_dirent *sysfs_root;

	sysfs_root = kzalloc(sizeof(struct sysfs_dirent), GFP_KERNEL);
	if (sysfs_root == NULL)
		return -ENOMEM;

	INIT_LIST_HEAD(&sysfs_root->s_sibling);
	INIT_LIST_HEAD(&sysfs_root->s_children);
	sysfs_root->s_type = SYSFS_ROOT;
	ve->sysfs_root = sysfs_root;
	return 0;
}

#if defined(CONFIG_NET) && defined(CONFIG_SYSFS)
extern struct class_device_attribute ve_net_class_attributes[];
static inline int init_ve_netclass(struct ve_struct *ve)
{
	struct class *nc;
	int err;

	nc = kzalloc(sizeof(*nc), GFP_KERNEL);
	if (!nc)
		return -ENOMEM;

	nc->name = net_class.name;
	nc->release = net_class.release;
	nc->uevent = net_class.uevent;
	nc->class_dev_attrs = ve_net_class_attributes;

	err = class_register(nc);
	if (!err) {
		ve->net_class = nc;
		return 0;
	}
	kfree(nc);	
	return err;
}

static inline void fini_ve_netclass(struct ve_struct *ve)
{
	class_unregister(ve->net_class);
	kfree(ve->net_class);
	ve->net_class = NULL;
}
#else
static inline int init_ve_netclass(struct ve_struct *ve) { return 0; }
static inline void fini_ve_netclass(struct ve_struct *ve) { ; }
#endif

static const struct {
	unsigned	minor;
	char		*name;
} mem_class_devices [] = {
	{3, "null"},
	{5, "zero"},
	{7, "full"},
	{8, "random"},
	{9, "urandom"},
	{0, NULL},
};

static struct class *init_ve_mem_class(void)
{
	int i;
	struct class *ve_mem_class;

	ve_mem_class = class_create(THIS_MODULE, "mem");
	if (IS_ERR(ve_mem_class))
		return ve_mem_class;
	for (i = 0; mem_class_devices[i].name; i++)
		class_device_create(ve_mem_class, NULL,
				MKDEV(MEM_MAJOR, mem_class_devices[i].minor),
				NULL, mem_class_devices[i].name);
	return ve_mem_class;
}


void fini_ve_mem_class(struct class *ve_mem_class)
{
	int i;

	for (i = 0; mem_class_devices[i].name; i++)
		class_device_destroy(ve_mem_class,
				MKDEV(MEM_MAJOR, mem_class_devices[i].minor));
	class_destroy(ve_mem_class);
}

static int init_ve_sysfs(struct ve_struct *ve)
{
	struct subsystem *subsys;
	int err;

#ifdef CONFIG_SYSFS
	err = 0;
	if (ve->features & VE_FEATURE_SYSFS) {
		err = init_ve_sysfs_root(ve);
		if (err != 0)
			goto out;
		err = register_ve_fs_type(ve,
				   &sysfs_fs_type,
				   &ve->sysfs_fstype,
				   &ve->sysfs_mnt);
	}
	if (err != 0)
		goto out_fs_type;
#endif
	err = -ENOMEM;
	subsys = kzalloc(sizeof(*subsys), GFP_KERNEL);
	if (subsys == NULL)
		goto out_class_obj;
	/* ick, this is ugly, the things we go through to keep from showing up
	 * in sysfs... */
	memcpy(&subsys->kset.kobj.name, &class_obj_subsys.kset.kobj.name,
			sizeof(subsys->kset.kobj.name));
	subsys->kset.ktype = class_obj_subsys.kset.ktype;
	subsys->kset.uevent_ops = class_obj_subsys.kset.uevent_ops;
	subsystem_init(subsys);
	if (!subsys->kset.subsys)
			subsys->kset.subsys = subsys;
	ve->class_obj_subsys = subsys;

	err = -ENOMEM;
	subsys = kzalloc(sizeof(*subsys), GFP_KERNEL);
	if (subsys == NULL)
		goto out_class_subsys;
	/* ick, this is ugly, the things we go through to keep from showing up
	 * in sysfs... */
	memcpy(&subsys->kset.kobj.name, &class_subsys.kset.kobj.name,
			sizeof(subsys->kset.kobj.name));
	subsys->kset.ktype = class_subsys.kset.ktype;
	subsys->kset.uevent_ops = class_subsys.kset.uevent_ops;
	ve->class_subsys = subsys;
	err = subsystem_register(subsys);
	if (err != 0)
		goto out_register;

	err = init_ve_netclass(ve);
	if (err)
		goto out_nc;

	ve->tty_class = init_ve_tty_class();
	if (IS_ERR(ve->tty_class)) {
		err = PTR_ERR(ve->tty_class);
		ve->tty_class = NULL;
		goto out_tty_class_register;
	}

	ve->mem_class = init_ve_mem_class();
	if (IS_ERR(ve->mem_class)) {
		err = PTR_ERR(ve->mem_class);
		ve->mem_class = NULL;
		goto out_mem_class_register;
	}

	return err;

out_mem_class_register:
	fini_ve_tty_class(ve->tty_class);
out_tty_class_register:
	fini_ve_netclass(ve);
out_nc:
	subsystem_unregister(subsys);
out_register:
	kfree(ve->class_subsys);
out_class_subsys:
	kfree(ve->class_obj_subsys);
out_class_obj:
#ifdef CONFIG_SYSFS
	unregister_ve_fs_type(ve->sysfs_fstype, ve->sysfs_mnt);
	/* sysfs_fstype is freed in real_put_ve -> free_ve_filesystems */
out_fs_type:
	kfree(ve->sysfs_root);
	ve->sysfs_root = NULL;
#endif
	ve->class_subsys = NULL;
	ve->class_obj_subsys = NULL;
out:
	return err;
}

static void fini_ve_sysfs(struct ve_struct *ve)
{
	fini_ve_mem_class(ve->mem_class);
	fini_ve_tty_class(ve->tty_class);
	fini_ve_netclass(ve);
	subsystem_unregister(ve->class_subsys);
	kfree(ve->class_subsys);
	kfree(ve->class_obj_subsys);

	ve->class_subsys = NULL;
	ve->class_obj_subsys = NULL;
#ifdef CONFIG_SYSFS
	unregister_ve_fs_type(ve->sysfs_fstype, ve->sysfs_mnt);
	ve->sysfs_mnt = NULL;
	kfree(ve->sysfs_root);
	ve->sysfs_root = NULL;
	/* sysfs_fstype is freed in real_put_ve -> free_ve_filesystems */
#endif
}

static void free_ve_filesystems(struct ve_struct *ve)
{
#ifdef CONFIG_SYSFS
	kfree(ve->sysfs_fstype);
	ve->sysfs_fstype = NULL;
#endif
	kfree(ve->shmem_fstype);
	ve->shmem_fstype = NULL;

	kfree(ve->devpts_fstype);
	ve->devpts_fstype = NULL;

	free_ve_proc(ve);
}

static int init_printk(struct ve_struct *ve)
{
	struct ve_prep_printk {
		wait_queue_head_t       log_wait;
		unsigned long           log_start;
		unsigned long           log_end;
		unsigned long           logged_chars;
	} *tmp;

	tmp = kzalloc(sizeof(struct ve_prep_printk), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	init_waitqueue_head(&tmp->log_wait);
	ve->_log_wait = &tmp->log_wait;
	ve->_log_start = &tmp->log_start;
	ve->_log_end = &tmp->log_end;
	ve->_logged_chars = &tmp->logged_chars;
	/* ve->log_buf will be initialized later by ve_log_init() */
	return 0;
}

static void fini_printk(struct ve_struct *ve)
{
	/* 
	 * there is no spinlock protection here because nobody can use
	 * log_buf at the moments when this code is called. 
	 */
	kfree(ve->log_buf);
	kfree(ve->_log_wait);
}

static void fini_venet(struct ve_struct *ve)
{
#ifdef CONFIG_INET
	tcp_v4_kill_ve_sockets(ve);
	ve_mapped_devs_cleanup(ve);
	synchronize_net();
#endif
}

static int init_ve_sched(struct ve_struct *ve, unsigned int vcpus)
{
#ifdef CONFIG_FAIRSCHED
	int err;

	/*
	 * We refuse to switch to an already existing node since nodes
	 * keep a pointer to their ve_struct...
	 */
	err = sys_fairsched_mknod(0, 1, ve->veid);
	if (err < 0) {
		printk(KERN_WARNING "Can't create fairsched node %d\n",
				ve->veid);
		return err;
	}
	err = sys_fairsched_vcpus(ve->veid, vcpus);
	if (err) {
		printk(KERN_WARNING "Can't set fairsched vcpus on node %d\n",
				ve->veid);
		goto cleanup;
	}
	err = sys_fairsched_mvpr(current->pid, ve->veid);
	if (err) {
		printk(KERN_WARNING "Can't switch to fairsched node %d\n",
				ve->veid);
		goto cleanup;
	}
#endif
	ve_sched_attach(ve);
	return 0;

#ifdef CONFIG_FAIRSCHED
cleanup:
	if (sys_fairsched_rmnod(ve->veid))
		printk(KERN_ERR "Can't clean fairsched node %d\n",
				ve->veid);
	return err;
#endif
}

static void fini_ve_sched(struct ve_struct *ve)
{
#ifdef CONFIG_FAIRSCHED
	if (task_vsched_id(current) == ve->veid)
		if (sys_fairsched_mvpr(current->pid, fairsched_init_node.id))
			printk(KERN_WARNING "Can't leave fairsched node %d\n",
					ve->veid);
	if (sys_fairsched_rmnod(ve->veid))
		printk(KERN_ERR "Can't remove fairsched node %d\n",
				ve->veid);
#endif
}

/*
 * Namespaces
 */

static inline int init_ve_namespaces(struct ve_struct *ve,
		struct nsproxy **old)
{
	int err;
	struct task_struct *tsk;
	struct nsproxy *cur;

	tsk = current;
	cur = get_nsproxy(tsk->nsproxy);

	err = copy_namespaces(CLONE_NAMESPACES_MASK, tsk);
	if (err < 0) {
		put_nsproxy(cur);
		return err;
	}

	ve->ve_ns = get_nsproxy(tsk->nsproxy);
	memcpy(ve->ve_ns->uts_ns->name.release, virt_utsname.release,
			sizeof(virt_utsname.release));
	*old = cur;
	return 0;
}

static inline void fini_ve_namespaces(struct ve_struct *ve,
		struct nsproxy *old)
{
	struct task_struct *tsk;
	struct nsproxy *cur;

	if (old) {
		tsk = current;
		cur = tsk->nsproxy;
		tsk->nsproxy = old;
		put_nsproxy(cur);
	}

	if (ve->ve_ns->ipc_ns)
		shm_clean_ns(ve->ve_ns->ipc_ns);

	put_nsproxy(ve->ve_ns);
	ve->ve_ns = NULL;
}

static inline void switch_ve_namespaces(struct ve_struct *ve,
		struct task_struct *tsk)
{
	struct nsproxy *old_ns;
	struct nsproxy *new_ns;

	BUG_ON(tsk != current);
	old_ns = tsk->nsproxy;
	new_ns = ve->ve_ns;

	if (old_ns != new_ns) {
		tsk->nsproxy = get_nsproxy(new_ns);
		put_nsproxy(old_ns);
	}
}

static __u64 get_ve_features(env_create_param_t *data, int datalen)
{
	__u64 known_features;

	if (datalen < sizeof(struct env_create_param3))
		/* this version of vzctl is aware of VE_FEATURES_OLD only */
		known_features = VE_FEATURES_OLD;
	else
		known_features = data->known_features;

	/*
	 * known features are set as required
	 * yet unknown features are set as in VE_FEATURES_DEF
	 */
	return (data->feature_mask & known_features) |
		(VE_FEATURES_DEF & ~known_features);
}

static int init_ve_struct(struct ve_struct *ve, envid_t veid,
		u32 class_id, env_create_param_t *data, int datalen,
		struct task_struct *init_tsk)
{
	int n;

	(void)get_ve(ve);
	ve->veid = veid;
	ve->class_id = class_id;
	ve->init_entry = init_tsk;
	ve->features = get_ve_features(data, datalen);
	INIT_LIST_HEAD(&ve->vetask_lh);
	init_rwsem(&ve->op_sem);
#ifdef CONFIG_NET
	ve->ifindex = -1;
#endif

	for(n = 0; n < UIDHASH_SZ_VE; ++n)
		INIT_LIST_HEAD(&ve->uidhash_table[n]);

	ve->start_timespec = ve->init_entry->start_time;
	/* The value is wrong, but it is never compared to process
	 * start times */
	ve->start_jiffies = get_jiffies_64();
	ve->start_cycles = get_cycles();
	ve->virt_pids = glob_virt_pids;

	return 0;
}

/**********************************************************************
 **********************************************************************
 *
 * /proc/meminfo virtualization
 *
 **********************************************************************
 **********************************************************************/
static int ve_set_meminfo(envid_t veid, unsigned long val)
{
#ifdef CONFIG_USER_RESOURCE
	struct ve_struct *ve;

	ve = get_ve_by_id(veid);
	if (!ve)
		return -EINVAL;

	ve->meminfo_val = val;
	real_put_ve(ve);
	return 0;
#else
	return -ENOTTY;
#endif
}

static int init_ve_meminfo(struct ve_struct *ve)
{
	ve->meminfo_val = 0;
	return 0;
}

static inline void fini_ve_meminfo(struct ve_struct *ve)
{
}

static void set_ve_root(struct ve_struct *ve, struct task_struct *tsk)
{
	read_lock(&tsk->fs->lock);
	ve->fs_rootmnt = tsk->fs->rootmnt;
	ve->fs_root = tsk->fs->root;
	read_unlock(&tsk->fs->lock);
	mark_tree_virtual(ve->fs_rootmnt, ve->fs_root);
}

static void set_ve_caps(struct ve_struct *ve, struct task_struct *tsk)
{
	/* required for real_setdevperms from register_ve_<fs> above */
	memcpy(&ve->ve_cap_bset, &tsk->cap_effective, sizeof(kernel_cap_t));
	cap_lower(ve->ve_cap_bset, CAP_SETVEID);
}

static int ve_list_add(struct ve_struct *ve)
{
	write_lock_irq(&ve_list_lock);
	if (__find_ve_by_id(ve->veid) != NULL)
		goto err_exists;

	list_add(&ve->ve_list, &ve_list_head);
	nr_ve++;
	write_unlock_irq(&ve_list_lock);
	return 0;

err_exists:
	write_unlock_irq(&ve_list_lock);
	return -EEXIST;
}

static void ve_list_del(struct ve_struct *ve)
{
	write_lock_irq(&ve_list_lock);
	list_del(&ve->ve_list);
	nr_ve--;
	write_unlock_irq(&ve_list_lock);
}

static void set_task_ve_caps(struct task_struct *tsk, struct ve_struct *ve)
{
	spin_lock(&task_capability_lock);
	cap_mask(tsk->cap_effective, ve->ve_cap_bset);
	cap_mask(tsk->cap_inheritable, ve->ve_cap_bset);
	cap_mask(tsk->cap_permitted, ve->ve_cap_bset);
	spin_unlock(&task_capability_lock);
}

void ve_move_task(struct task_struct *tsk, struct ve_struct *new)
{
	struct ve_struct *old;

	might_sleep();
	BUG_ON(tsk != current);
	BUG_ON(!(thread_group_leader(tsk) && thread_group_empty(tsk)));

	/* this probihibts ptracing of task entered to VE from host system */
	tsk->mm->vps_dumpable = 0;
	/* setup capabilities before enter */
	set_task_ve_caps(tsk, new);

	old = tsk->ve_task_info.owner_env;
	tsk->ve_task_info.owner_env = new;
	tsk->ve_task_info.exec_env = new;

	write_lock_irq(&tasklist_lock);
	list_del_rcu(&tsk->ve_task_info.vetask_list);
	write_unlock_irq(&tasklist_lock);

	synchronize_rcu();

	write_lock_irq(&tasklist_lock);
	list_add_tail_rcu(&tsk->ve_task_info.vetask_list,
			&new->vetask_lh);
	write_unlock_irq(&tasklist_lock);

	atomic_dec(&old->pcounter);
	real_put_ve(old);

	atomic_inc(&new->pcounter);
	get_ve(new);
}

EXPORT_SYMBOL(ve_move_task);

#ifdef CONFIG_VE_IPTABLES
extern int init_netfilter(void);
extern void fini_netfilter(void);
#define init_ve_netfilter()	init_netfilter()
#define fini_ve_netfilter()	fini_netfilter()

#define KSYMIPTINIT(mask, ve, full_mask, mod, name, args)	\
({								\
	int ret = 0;						\
	if (VE_IPT_CMP(mask, full_mask) &&			\
		VE_IPT_CMP((ve)->_iptables_modules, 		\
			full_mask & ~(full_mask##_MOD))) {	\
		ret = KSYMERRCALL(1, mod, name, args);		\
		if (ret == 0)					\
			(ve)->_iptables_modules |=		\
					full_mask##_MOD;	\
		if (ret == 1)					\
			ret = 0;				\
	}							\
	ret;							\
})

#define KSYMIPTFINI(mask, full_mask, mod, name, args)		\
({								\
 	if (VE_IPT_CMP(mask, full_mask##_MOD))			\
		KSYMSAFECALL_VOID(mod, name, args);		\
})


static int do_ve_iptables(struct ve_struct *ve, __u64 init_mask,
		int init_or_cleanup)
{
	int err;

	/* Remove when userspace will start supplying IPv6-related bits. */
	init_mask &= ~VE_IP_IPTABLES6;
	init_mask &= ~VE_IP_FILTER6;
	init_mask &= ~VE_IP_MANGLE6;
	init_mask &= ~VE_IP_IPTABLE_NAT_MOD;
	if ((init_mask & VE_IP_IPTABLES) == VE_IP_IPTABLES)
		init_mask |= VE_IP_IPTABLES6;
	if ((init_mask & VE_IP_FILTER) == VE_IP_FILTER)
		init_mask |= VE_IP_FILTER6;
	if ((init_mask & VE_IP_MANGLE) == VE_IP_MANGLE)
		init_mask |= VE_IP_MANGLE6;
	if ((init_mask & VE_IP_NAT) == VE_IP_NAT)
		init_mask |= VE_IP_IPTABLE_NAT;

	err = 0;
	if (!init_or_cleanup)
		goto cleanup;

	/* init part */
#if defined(CONFIG_IP_NF_IPTABLES) || \
    defined(CONFIG_IP_NF_IPTABLES_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_IPTABLES,
			ip_tables, init_iptables, ());
	if (err < 0)
		goto err_iptables;
#endif
#if defined(CONFIG_IP6_NF_IPTABLES) || \
    defined(CONFIG_IP6_NF_IPTABLES_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_IPTABLES6,
			ip6_tables, init_ip6tables, ());
	if (err < 0)
		goto err_ip6tables;
#endif
#if defined(CONFIG_IP_NF_CONNTRACK) || \
    defined(CONFIG_IP_NF_CONNTRACK_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_CONNTRACK,
			ip_conntrack, init_iptable_conntrack, ());
	if (err < 0)
		goto err_iptable_conntrack;
#endif
#if defined(CONFIG_IP_NF_FTP) || \
    defined(CONFIG_IP_NF_FTP_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_CONNTRACK_FTP,
			ip_conntrack_ftp, init_ip_ct_ftp, ());
	if (err < 0)
		goto err_iptable_ftp;
#endif
#if defined(CONFIG_IP_NF_IRC) || \
    defined(CONFIG_IP_NF_IRC_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_CONNTRACK_IRC,
			ip_conntrack_irc, init_ip_ct_irc, ());
	if (err < 0)
		goto err_iptable_irc;
#endif
#if defined(CONFIG_IP_NF_NAT) || \
    defined(CONFIG_IP_NF_NAT_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_NAT,
			ip_nat, ip_nat_init, ());
	if (err < 0)
		goto err_iptable_nat;
	err = KSYMIPTINIT(init_mask, ve, VE_IP_IPTABLE_NAT,
			iptable_nat, init_iptable_nat, ());
	if (err < 0)
		goto err_iptable_nat2;
#endif
#if defined(CONFIG_IP_NF_NAT_FTP) || \
    defined(CONFIG_IP_NF_NAT_FTP_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_NAT_FTP,
			ip_nat_ftp, init_iptable_nat_ftp, ());
	if (err < 0)
		goto err_iptable_nat_ftp;
#endif
#if defined(CONFIG_IP_NF_NAT_IRC) || \
    defined(CONFIG_IP_NF_NAT_IRC_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_NAT_IRC,
			ip_nat_irc, init_iptable_nat_irc, ());
	if (err < 0)
		goto err_iptable_nat_irc;
#endif
#if defined(CONFIG_IP_NF_FILTER) || \
    defined(CONFIG_IP_NF_FILTER_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_FILTER,
			iptable_filter,	init_iptable_filter, ());
	if (err < 0)
		goto err_iptable_filter;
#endif
#if defined(CONFIG_IP6_NF_FILTER) || \
    defined(CONFIG_IP6_NF_FILTER_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_FILTER6,
			ip6table_filter, init_ip6table_filter, ());
	if (err < 0)
		goto err_ip6table_filter;
#endif
#if defined(CONFIG_IP_NF_MANGLE) || \
    defined(CONFIG_IP_NF_MANGLE_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_MANGLE,
			iptable_mangle,	init_iptable_mangle, ());
	if (err < 0)
		goto err_iptable_mangle;
#endif
#if defined(CONFIG_IP6_NF_MANGLE) || \
    defined(CONFIG_IP6_NF_MANGLE_MODULE)
	err = KSYMIPTINIT(init_mask, ve, VE_IP_MANGLE6,
			ip6table_mangle, init_ip6table_mangle, ());
	if (err < 0)
		goto err_ip6table_mangle;
#endif
	return 0;

/* ------------------------------------------------------------------------- */

cleanup:
#if defined(CONFIG_IP6_NF_MANGLE) || \
    defined(CONFIG_IP6_NF_MANGLE_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_MANGLE6,
			ip6table_mangle, fini_ip6table_mangle, ());
err_ip6table_mangle:
#endif
#if defined(CONFIG_IP_NF_MANGLE) || \
    defined(CONFIG_IP_NF_MANGLE_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_MANGLE,
			iptable_mangle,	fini_iptable_mangle, ());
err_iptable_mangle:
#endif
#if defined(CONFIG_IP6_NF_FILTER) || \
    defined(CONFIG_IP6_NF_FILTER_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_FILTER6,
			ip6table_filter, fini_ip6table_filter, ());
err_ip6table_filter:
#endif
#if defined(CONFIG_IP_NF_FILTER) || \
    defined(CONFIG_IP_NF_FILTER_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_FILTER,
			iptable_filter,	fini_iptable_filter, ());
err_iptable_filter:
#endif
#if defined(CONFIG_IP_NF_NAT_IRC) || \
    defined(CONFIG_IP_NF_NAT_IRC_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_NAT_IRC,
			ip_nat_irc, fini_iptable_nat_irc, ());
err_iptable_nat_irc:
#endif
#if defined(CONFIG_IP_NF_NAT_FTP) || \
    defined(CONFIG_IP_NF_NAT_FTP_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_NAT_FTP,
			ip_nat_ftp, fini_iptable_nat_ftp, ());
err_iptable_nat_ftp:
#endif
#if defined(CONFIG_IP_NF_NAT) || \
    defined(CONFIG_IP_NF_NAT_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_IPTABLE_NAT,
			iptable_nat, fini_iptable_nat, ());
err_iptable_nat2:
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_NAT,
			ip_nat, ip_nat_cleanup, ());
err_iptable_nat:
#endif
#if defined(CONFIG_IP_NF_IRC) || \
    defined(CONFIG_IP_NF_IRC_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_CONNTRACK_IRC,
			ip_conntrack_irc, fini_ip_ct_irc, ());
err_iptable_irc:
#endif
#if defined(CONFIG_IP_NF_FTP) || \
    defined(CONFIG_IP_NF_FTP_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_CONNTRACK_FTP,
			ip_conntrack_ftp, fini_ip_ct_ftp, ());
err_iptable_ftp:
#endif
#if defined(CONFIG_IP_NF_CONNTRACK) || \
    defined(CONFIG_IP_NF_CONNTRACK_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_CONNTRACK,
			ip_conntrack, fini_iptable_conntrack, ());
err_iptable_conntrack:
#endif
#if defined(CONFIG_IP6_NF_IPTABLES) || \
    defined(CONFIG_IP6_NF_IPTABLES_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_IPTABLES6,
			ip6_tables, fini_ip6tables, ());
err_ip6tables:
#endif
#if defined(CONFIG_IP_NF_IPTABLES) || \
    defined(CONFIG_IP_NF_IPTABLES_MODULE)
	KSYMIPTFINI(ve->_iptables_modules, VE_IP_IPTABLES,
			ip_tables, fini_iptables, ());
err_iptables:
#endif
	ve->_iptables_modules = 0;

	return err;
}

static inline int init_ve_iptables(struct ve_struct *ve, __u64 init_mask)
{
	return do_ve_iptables(ve, init_mask, 1);
}

static inline void fini_ve_iptables(struct ve_struct *ve, __u64 init_mask)
{
	(void)do_ve_iptables(ve, init_mask, 0);
}

#else
#define init_ve_iptables(x, y)	(0)
#define fini_ve_iptables(x, y)	do { } while (0)
#define init_ve_netfilter()	(0)
#define fini_ve_netfilter()	do { } while (0)
#endif

static inline int init_ve_cpustats(struct ve_struct *ve)
{
	ve->cpu_stats = alloc_percpu(struct ve_cpu_stats);
	if (ve->cpu_stats == NULL)
		return -ENOMEM;
	ve->sched_lat_ve.cur = alloc_percpu(struct kstat_lat_pcpu_snap_struct);
	if (ve == NULL)
		goto fail;
	return 0;

fail:
	free_percpu(ve->cpu_stats);
	return -ENOMEM;
}

static inline void free_ve_cpustats(struct ve_struct *ve)
{
	free_percpu(ve->cpu_stats);
	ve->cpu_stats = NULL;
	free_percpu(ve->sched_lat_ve.cur);
	ve->sched_lat_ve.cur = NULL;
}

static int alone_in_pgrp(struct task_struct *tsk)
{
	struct task_struct *p;
	int alone = 0;

	read_lock(&tasklist_lock);
	do_each_task_pid_all(tsk->pid, PIDTYPE_PGID, p) {
		if (p != tsk)
			goto out;
	} while_each_task_pid_all(tsk->pid, PIDTYPE_PGID, p);
	do_each_task_pid_all(tsk->pid, PIDTYPE_SID, p) {
		if (p != tsk)
			goto out;
	} while_each_task_pid_all(tsk->pid, PIDTYPE_SID, p);
	alone = 1;
out:
	read_unlock(&tasklist_lock);
	return alone;
}

static int do_env_create(envid_t veid, unsigned int flags, u32 class_id,
			 env_create_param_t *data, int datalen)
{
	struct task_struct *tsk;
	struct ve_struct *old;
	struct ve_struct *old_exec;
	struct ve_struct *ve;
 	__u64 init_mask;
	int err;
	struct nsproxy *old_ns;

	tsk = current;
	old = VE_TASK_INFO(tsk)->owner_env;

	if (!thread_group_leader(tsk) || !thread_group_empty(tsk))
		return -EINVAL;

	if (tsk->signal->tty) {
		printk("ERR: CT init has controlling terminal\n");
		return -EINVAL;
	}
	if (tsk->signal->pgrp != tsk->pid ||
	    tsk->signal->session != tsk->pid) {
		int may_setsid;

		read_lock(&tasklist_lock);
		may_setsid = !tsk->signal->leader &&
			!find_task_by_pid_type_all(PIDTYPE_PGID, tsk->pid);
		read_unlock(&tasklist_lock);

		if (!may_setsid) {
			printk("ERR: CT init is process group leader\n");
			return -EINVAL;
		}
	}
	/* Check that the process is not a leader of non-empty group/session.
	 * If it is, we cannot virtualize its PID and must fail. */
	if (!alone_in_pgrp(tsk)) {
		printk("ERR: CT init is not alone in process group\n");
		return -EINVAL;
	}


	VZTRACE("%s: veid=%d classid=%d pid=%d\n",
		__FUNCTION__, veid, class_id, current->pid);

	err = -ENOMEM;
	ve = kzalloc(sizeof(struct ve_struct), GFP_KERNEL);
	if (ve == NULL)
		goto err_struct;

	init_ve_struct(ve, veid, class_id, data, datalen, tsk);
	__module_get(THIS_MODULE);
	down_write(&ve->op_sem);
	if (flags & VE_LOCK)
		ve->is_locked = 1;

	/*
	 * this should be done before adding to list
	 * because if calc_load_ve finds this ve in
	 * list it will be very surprised
	 */
	if ((err = init_ve_cpustats(ve)) < 0)
		goto err_cpu_stats;

	if ((err = ve_list_add(ve)) < 0)
		goto err_exist;

	/* this should be done before context switching */
	if ((err = init_printk(ve)) < 0)
		goto err_log_wait;

	old_exec = set_exec_env(ve);

	if ((err = init_ve_sched(ve, data->total_vcpus)) < 0)
		goto err_sched;

	/* move user to VE */
	if ((err = set_user(0, 0)) < 0)
		goto err_set_user;

	set_ve_root(ve, tsk);

	if ((err = init_ve_namespaces(ve, &old_ns)))
		goto err_ns;

	if ((err = init_ve_mibs(ve)))
		goto err_mibs;

	if ((err = init_ve_proc(ve)))
		goto err_proc;

	if ((err = init_ve_sysctl(ve)))
		goto err_sysctl;

	if ((err = init_ve_sysfs(ve)))
		goto err_sysfs;

	if ((err = ve_arp_init(ve)) < 0)
		goto err_route;
	if ((err = ve_ndisc_init(ve)) < 0)
		goto err_route;

	if ((err = init_ve_route(ve)) < 0)
		goto err_route;

	if ((err = init_ve_route6(ve)) < 0)
		goto err_route;

	if ((err = init_ve_netdev()))
		goto err_dev;

	if ((err = init_ve_tty_drivers(ve)) < 0)
		goto err_tty;

	if ((err = init_ve_shmem(ve)))
		goto err_shmem;

	if ((err = init_ve_devpts(ve)))
		goto err_devpts;

	if((err = init_ve_meminfo(ve)))
		goto err_meminf;

	set_ve_caps(ve, tsk);

	/* It is safe to initialize netfilter here as routing initialization and
	   interface setup will be done below. This means that NO skb can be
	   passed inside. Den */
	/* iptables ve initialization for non ve0;
	   ve0 init is in module_init */
	if ((err = init_ve_netfilter()) < 0)
		goto err_netfilter;

	init_mask = data ? data->iptables_mask : VE_IP_DEFAULT;
	if ((err = init_ve_iptables(ve, init_mask)) < 0)
		goto err_iptables;

	if ((err = alloc_vpid(tsk->pids[PIDTYPE_PID].pid, 1)) < 0)
		goto err_vpid;

	if ((err = ve_hook_iterate_init(VE_SS_CHAIN, ve)) < 0)
		goto err_ve_hook;

	put_nsproxy(old_ns);

	/* finally: set vpids and move inside */
	ve_move_task(tsk, ve);

	set_virt_pid(tsk, 1);
	set_virt_tgid(tsk, 1);

	set_special_pids(tsk->pid, tsk->pid);
	current->signal->tty_old_pgrp = 0;
	set_virt_pgid(tsk, 1);
	set_virt_sid(tsk, 1);
	grsecurity_setup();

	ve->is_running = 1;
	up_write(&ve->op_sem);

	printk(KERN_INFO "CT: %d: started\n", veid);
	return veid;

err_ve_hook:
	free_vpid(tsk->pids[PIDTYPE_PID].pid);
err_vpid:
	fini_venet(ve);
	fini_ve_iptables(ve, init_mask);
err_iptables:
	fini_ve_netfilter();
err_netfilter:
	fini_ve_meminfo(ve);
err_meminf:
	fini_ve_devpts(ve);
err_devpts:
	fini_ve_shmem(ve);
err_shmem:
	fini_ve_tty_drivers(ve);
err_tty:
	fini_ve_netdev();
err_dev:
	fini_ve_route(ve);
	fini_ve_route6(ve);
err_route:
	ve_ndisc_fini(ve);
	ve_arp_fini(ve);
	fini_ve_sysfs(ve);
err_sysfs:
	fini_ve_sysctl(ve);
err_sysctl:
	fini_ve_proc(ve);
err_proc:
	clean_device_perms_ve(ve->veid);
	fini_ve_mibs(ve);
err_mibs:
	/* free_ve_utsname() is called inside real_put_ve() */ ;
	fini_ve_namespaces(ve, old_ns);
err_ns:
	/* It is safe to restore current->envid here because
	 * ve_fairsched_detach does not use current->envid. */
	/* Really fairsched code uses current->envid in sys_fairsched_mknod 
	 * only.  It is correct if sys_fairsched_mknod is called from
	 * userspace.  If sys_fairsched_mknod is called from
	 * ve_fairsched_attach, then node->envid and node->parent_node->envid
	 * are explicitly set to valid value after the call. */
	/* FIXME */
	VE_TASK_INFO(tsk)->owner_env = old;
	VE_TASK_INFO(tsk)->exec_env = old_exec;
	/* move user back */
	if (set_user(0, 0) < 0)
		printk(KERN_WARNING"Can't restore UID\n");

err_set_user:
	fini_ve_sched(ve);
err_sched:
	(void)set_exec_env(old_exec);

	/* we can jump here having incorrect envid */
	VE_TASK_INFO(tsk)->owner_env = old;
	fini_printk(ve);
err_log_wait:
	/* cpustats will be freed in do_env_free */
	ve_list_del(ve);
	up_write(&ve->op_sem);

	real_put_ve(ve);
err_struct:
	printk(KERN_INFO "CT: %d: failed to start with err=%d\n", veid, err);
	return err;

err_exist:
	free_ve_cpustats(ve);
err_cpu_stats:
	kfree(ve);
	goto err_struct;
}


/**********************************************************************
 **********************************************************************
 *
 * VE start/stop callbacks
 *
 **********************************************************************
 **********************************************************************/

int real_env_create(envid_t veid, unsigned flags, u32 class_id,
			env_create_param_t *data, int datalen)
{
	int status;
	struct ve_struct *ve;

	if (!flags) {
		status = get_exec_env()->veid;
		goto out;
	}

	status = -EPERM;
	if (!capable(CAP_SETVEID))
		goto out;

	status = -EINVAL;
	if ((flags & VE_TEST) && (flags & (VE_ENTER|VE_CREATE)))
		goto out;

	status = -EINVAL;
	ve = get_ve_by_id(veid);
	if (ve) {
		if (flags & VE_TEST) {
			status = 0;
			goto out_put;
		}
		if (flags & VE_EXCLUSIVE) {
			status = -EACCES;
			goto out_put;
		}
		if (flags & VE_CREATE) {
			flags &= ~VE_CREATE;
			flags |= VE_ENTER;
		}
	} else {
		if (flags & (VE_TEST|VE_ENTER)) {
			status = -ESRCH;
			goto out;
		}
	}

	if (flags & VE_CREATE) {
		status = do_env_create(veid, flags, class_id, data, datalen);
		goto out;
	} else if (flags & VE_ENTER)
		status = do_env_enter(ve, flags);

	/* else: returning EINVAL */

out_put:
	real_put_ve(ve);
out:
	return status;
}
EXPORT_SYMBOL(real_env_create);

static int do_env_enter(struct ve_struct *ve, unsigned int flags)
{
	struct task_struct *tsk = current;
	int err;

	VZTRACE("%s: veid=%d\n", __FUNCTION__, ve->veid);

	err = -EBUSY;
	down_read(&ve->op_sem);
	if (!ve->is_running)
		goto out_up;
	if (ve->is_locked && !(flags & VE_SKIPLOCK))
		goto out_up;
	err = -EINVAL;
	if (!thread_group_leader(tsk) || !thread_group_empty(tsk))
		goto out_up;

#ifdef CONFIG_FAIRSCHED
	err = sys_fairsched_mvpr(current->pid, ve->veid);
	if (err)
		goto out_up;
#endif
	ve_sched_attach(ve);
	switch_ve_namespaces(ve, tsk);
	ve_move_task(current, ve);

	/* Check that the process is not a leader of non-empty group/session.
	 * If it is, we cannot virtualize its PID. Do not fail, just leave
	 * it non-virtual.
	 */
	if (!is_virtual_pid(virt_pid(tsk)) && alone_in_pgrp(tsk)) {
		pid_t vpid = alloc_vpid(tsk->pids[PIDTYPE_PID].pid, -1);
		if (vpid > 0) {
			set_virt_pid(tsk, vpid);
			set_virt_tgid(tsk, vpid);
			if (tsk->signal->pgrp == tsk->pid)
				set_virt_pgid(tsk, vpid);
			if (tsk->signal->session == tsk->pid)
				set_virt_sid(tsk, vpid);
		}
	}
	/* Unlike VE_CREATE, we do not setsid() in VE_ENTER.
	 * Process is allowed to be in an external group/session.
	 * If user space callers wants, it will do setsid() after
	 * VE_ENTER.
	 */
	err = VE_TASK_INFO(tsk)->owner_env->veid;

out_up:
	up_read(&ve->op_sem);
	return err;
}

static void env_cleanup(struct ve_struct *ve)
{
	struct ve_struct *old_ve;

	VZTRACE("real_do_env_cleanup\n");

	down_read(&ve->op_sem);
	old_ve = set_exec_env(ve);

	ve_hook_iterate_fini(VE_SS_CHAIN, ve);

	fini_venet(ve);

	/* no new packets in flight beyond this point */
	/* skb hold dst_entry, and in turn lies in the ip fragment queue */
	ip_fragment_cleanup(ve);
	ip6_frag_cleanup(ve);

	fini_ve_netdev();
	fini_ve_route(ve);
	fini_ve_route6(ve);

	ve_arp_fini(ve);
	ve_ndisc_fini(ve);

	fini_ve_namespaces(ve, NULL);

	/* kill iptables */
	/* No skb belonging to VE can exist at this point as unregister_netdev
	   is an operation awaiting until ALL skb's gone */
	fini_ve_iptables(ve, ve->_iptables_modules);
	fini_ve_netfilter();

	fini_ve_sched(ve);
	clean_device_perms_ve(ve->veid);

	fini_ve_devpts(ve);
	fini_ve_shmem(ve);
	fini_ve_sysfs(ve);
	unregister_ve_tty_drivers(ve);
	fini_ve_sysctl(ve);
	fini_ve_proc(ve);
	fini_ve_meminfo(ve);

	fini_ve_mibs(ve);

	(void)set_exec_env(old_ve);
	fini_printk(ve);	/* no printk can happen in ve context anymore */

	ve_list_del(ve);
	up_read(&ve->op_sem);

	real_put_ve(ve);
}

static DECLARE_COMPLETION(vzmond_complete);
static volatile int stop_vzmond;

static int vzmond_helper(void *arg)
{
	char name[18];
	struct ve_struct *ve;

	ve = (struct ve_struct *)arg;
	snprintf(name, sizeof(name), "vzmond/%d", ve->veid);
	daemonize(name);
	env_cleanup(ve);
	module_put_and_exit(0);
}

static void do_pending_env_cleanups(void)
{
	int err;
	struct ve_struct *ve;

	spin_lock(&ve_cleanup_lock);
	while (1) {
		if (list_empty(&ve_cleanup_list) || need_resched())
			break;

		ve = list_first_entry(&ve_cleanup_list,
				struct ve_struct, cleanup_list);
		list_del(&ve->cleanup_list);
		spin_unlock(&ve_cleanup_lock);

		__module_get(THIS_MODULE);
		err = kernel_thread(vzmond_helper, (void *)ve, 0);
		if (err < 0) {
			env_cleanup(ve);
			module_put(THIS_MODULE);
		}

		spin_lock(&ve_cleanup_lock);
	}
	spin_unlock(&ve_cleanup_lock);
}

static inline int have_pending_cleanups(void)
{
	return !list_empty(&ve_cleanup_list);
}

static int vzmond(void *arg)
{
	daemonize("vzmond");
	set_current_state(TASK_INTERRUPTIBLE);

	while (!stop_vzmond || have_pending_cleanups()) {
		schedule();
		try_to_freeze();
		if (signal_pending(current))
			flush_signals(current);

		do_pending_env_cleanups();
		set_current_state(TASK_INTERRUPTIBLE);
		if (have_pending_cleanups())
			__set_current_state(TASK_RUNNING);
	}

	__set_task_state(current, TASK_RUNNING);
	complete_and_exit(&vzmond_complete, 0);
}

static int __init init_vzmond(void)
{
	int pid;
	struct task_struct *tsk;

	pid = kernel_thread(vzmond, NULL, 0);
	if (pid > 0) {
		tsk = find_task_by_pid_all(pid);
		BUG_ON(tsk == NULL);
		ve_cleanup_thread = tsk;
	}
	return pid;
}

static void fini_vzmond(void)
{
	stop_vzmond = 1;
	wake_up_process(ve_cleanup_thread);
	wait_for_completion(&vzmond_complete);
	ve_cleanup_thread = NULL;
	WARN_ON(!list_empty(&ve_cleanup_list));
}

void real_do_env_free(struct ve_struct *ve)
{
	VZTRACE("real_do_env_free\n");

	free_ve_tty_drivers(ve);
	free_ve_sysctl(ve); /* free per ve sysctl data */
	free_ve_filesystems(ve);
	free_ve_cpustats(ve);
	printk(KERN_INFO "CT: %d: stopped\n", VEID(ve));
	kfree(ve);

	module_put(THIS_MODULE);
}
EXPORT_SYMBOL(real_do_env_free);


/**********************************************************************
 **********************************************************************
 *
 * VE TTY handling
 *
 **********************************************************************
 **********************************************************************/

static struct tty_driver *alloc_ve_tty_driver(struct tty_driver *base,
					   struct ve_struct *ve)
{
	size_t size;
	struct tty_driver *driver;

	driver = ub_kmalloc(sizeof(struct tty_driver), GFP_KERNEL);
	if (!driver)
		goto out;

	memcpy(driver, base, sizeof(struct tty_driver));

	driver->driver_state = NULL;

	size = base->num * 3 * sizeof(void *);
	if (!(driver->flags & TTY_DRIVER_DEVPTS_MEM)) {
		void **p;
		p = ub_kzalloc(size, GFP_KERNEL);
		if (!p)
			goto out_free;

		driver->ttys = (struct tty_struct **)p;
		driver->termios = (struct termios **)(p + driver->num);
		driver->termios_locked = (struct termios **)
			(p + driver->num * 2);
	} else {
		driver->ttys = NULL;
		driver->termios = NULL;
		driver->termios_locked = NULL;
	}

	driver->owner_env = ve;
	driver->flags |= TTY_DRIVER_INSTALLED;
	driver->refcount = 0;

	return driver;

out_free:
	kfree(driver);
out:
	return NULL;
}

static void free_ve_tty_driver(struct tty_driver *driver)
{
	if (!driver)
		return;

	clear_termios(driver);
	kfree(driver->ttys);
	kfree(driver);
}

static int alloc_ve_tty_drivers(struct ve_struct* ve)
{
#ifdef CONFIG_LEGACY_PTYS
	/* Traditional BSD devices */
	ve->pty_driver = alloc_ve_tty_driver(pty_driver, ve);
	if (!ve->pty_driver)
		goto out_mem;

	ve->pty_slave_driver = alloc_ve_tty_driver(pty_slave_driver, ve);
	if (!ve->pty_slave_driver)
		goto out_mem;

	ve->pty_driver->other       = ve->pty_slave_driver;
	ve->pty_slave_driver->other = ve->pty_driver;
#endif	

#ifdef CONFIG_UNIX98_PTYS
	ve->ptm_driver = alloc_ve_tty_driver(ptm_driver, ve);
	if (!ve->ptm_driver)
		goto out_mem;

	ve->pts_driver = alloc_ve_tty_driver(pts_driver, ve);
	if (!ve->pts_driver)
		goto out_mem;

	ve->ptm_driver->other = ve->pts_driver;
	ve->pts_driver->other = ve->ptm_driver;

	ve->allocated_ptys = ub_kmalloc(sizeof(*ve->allocated_ptys),
			GFP_KERNEL);
	if (!ve->allocated_ptys)
		goto out_mem;
	idr_init(ve->allocated_ptys);
#endif
	return 0;

out_mem:
	free_ve_tty_drivers(ve);
	return -ENOMEM;
}

static void free_ve_tty_drivers(struct ve_struct* ve)
{
#ifdef CONFIG_LEGACY_PTYS
	free_ve_tty_driver(ve->pty_driver);
	free_ve_tty_driver(ve->pty_slave_driver);
	ve->pty_driver = ve->pty_slave_driver = NULL;
#endif	
#ifdef CONFIG_UNIX98_PTYS
	free_ve_tty_driver(ve->ptm_driver);
	free_ve_tty_driver(ve->pts_driver);
	kfree(ve->allocated_ptys);
	ve->ptm_driver = ve->pts_driver = NULL;
	ve->allocated_ptys = NULL;
#endif
}

static inline void __register_tty_driver(struct tty_driver *driver)
{
	list_add(&driver->tty_drivers, &tty_drivers);
}

static inline void __unregister_tty_driver(struct tty_driver *driver)
{
	if (!driver)
		return;
	list_del(&driver->tty_drivers);
}

static int register_ve_tty_drivers(struct ve_struct* ve)
{
	write_lock_irq(&tty_driver_guard);
#ifdef CONFIG_UNIX98_PTYS
	__register_tty_driver(ve->ptm_driver);
	__register_tty_driver(ve->pts_driver);
#endif
#ifdef CONFIG_LEGACY_PTYS
	__register_tty_driver(ve->pty_driver);
	__register_tty_driver(ve->pty_slave_driver);
#endif	
	write_unlock_irq(&tty_driver_guard);

	return 0;
}

static void unregister_ve_tty_drivers(struct ve_struct* ve)
{
	VZTRACE("unregister_ve_tty_drivers\n");

	write_lock_irq(&tty_driver_guard);
#ifdef CONFIG_LEGACY_PTYS
	__unregister_tty_driver(ve->pty_driver);
	__unregister_tty_driver(ve->pty_slave_driver);
#endif
#ifdef CONFIG_UNIX98_PTYS
	__unregister_tty_driver(ve->ptm_driver);
	__unregister_tty_driver(ve->pts_driver);
#endif
	write_unlock_irq(&tty_driver_guard);
}

static int init_ve_tty_drivers(struct ve_struct *ve)
{
	int err;

	if ((err = alloc_ve_tty_drivers(ve)))
		goto err_ttyalloc;
	if ((err = register_ve_tty_drivers(ve)))
		goto err_ttyreg;
	return 0;

err_ttyreg:
	free_ve_tty_drivers(ve);
err_ttyalloc:
	return err;
}

static void fini_ve_tty_drivers(struct ve_struct *ve)
{
	unregister_ve_tty_drivers(ve);
	free_ve_tty_drivers(ve);
}

/*
 * Free the termios and termios_locked structures because
 * we don't want to get memory leaks when modular tty
 * drivers are removed from the kernel.
 */
static void clear_termios(struct tty_driver *driver)
{
	int i;
	struct termios *tp;

	if (driver->termios == NULL)
		return;
	for (i = 0; i < driver->num; i++) {
		tp = driver->termios[i];
		if (tp) {
			driver->termios[i] = NULL;
			kfree(tp);
		}
		tp = driver->termios_locked[i];
		if (tp) {
			driver->termios_locked[i] = NULL;
			kfree(tp);
		}
	}
}


/**********************************************************************
 **********************************************************************
 *
 * Pieces of VE network
 *
 **********************************************************************
 **********************************************************************/

#ifdef CONFIG_NET
#include <asm/uaccess.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/route.h>
#include <net/ip_fib.h>
#endif

#ifdef CONFIG_INET
static void ve_del_ip_addrs(struct net_device *dev)
{
	struct in_device *in_dev;

	in_dev = in_dev_get(dev);
	if (in_dev == NULL)
		return;

	while (in_dev->ifa_list != NULL) {
		inet_del_ifa(in_dev, &in_dev->ifa_list, 1);
	}
	in_dev_put(in_dev);
}

static void ve_del_ipv6_addrs(struct net_device *dev)
{
#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
	addrconf_ifdown(dev, 2);
#endif
}

static int ve_netdev_cleanup(struct net_device *dev, int to_ve)
{
	int err;

	err = 0;
	ve_del_ip_addrs(dev);
	ve_del_ipv6_addrs(dev);
	if ((dev->flags & IFF_UP) != 0)
		err = dev_close(dev);
	synchronize_net();
	dev_shutdown(dev);
	dev_mc_discard(dev);
	free_divert_blk(dev);
	synchronize_net();
	return err;
}

static void __ve_dev_move(struct net_device *dev, struct ve_struct *ve_src,
	struct ve_struct *ve_dst, struct user_beancounter *exec_ub)
{
	struct net_device **dp, *d;
	struct user_beancounter *ub;
	struct ve_struct *exec_ve;

	for (d = ve_src->_net_dev_base, dp = NULL; d != NULL; 
	     dp = &d->next, d = d->next) {
		if (d == dev) {
			hlist_del(&dev->name_hlist);
			hlist_del(&dev->index_hlist);
			if (ve_src->_net_dev_tail == &dev->next)
				ve_src->_net_dev_tail = dp;
			if (dp)
				*dp = dev->next;
			dev->next = NULL;
			break;
		}
	}
	*ve_dst->_net_dev_tail = dev;
	ve_dst->_net_dev_tail = &dev->next;
	hlist_add_head(&dev->name_hlist, dev_name_hash(dev->name, ve_dst));
	hlist_add_head(&dev->index_hlist, dev_index_hash(dev->ifindex, ve_dst));
	dev->owner_env = ve_dst;

	ub = netdev_bc(dev)->exec_ub;
	netdev_bc(dev)->exec_ub = get_beancounter(exec_ub);
	put_beancounter(ub);

	write_unlock_bh(&dev_base_lock);

	exec_ve = set_exec_env(ve_src);
	call_netdevice_notifiers(NETDEV_UNREGISTER, dev);

	__rtnl_unlock();

	netdev_unregister_sysfs(dev);
	(void)set_exec_env(exec_ve);

	rtnl_lock();
	exec_ve = set_exec_env(ve_dst);
	call_netdevice_notifiers(NETDEV_REGISTER, dev);
	if (netdev_register_sysfs(dev))
		printk(KERN_ERR "Can't register %s dev in CT%d sysfs\n",
				dev->name, ve_dst->veid);
	(void)set_exec_env(exec_ve);

	write_lock_bh(&dev_base_lock);
}

static int ve_dev_add(envid_t veid, char *dev_name)
{
	int err;
	struct net_device *dev;
	struct ve_struct *ve;
	struct hlist_node *p;
	struct hlist_head *head;

	dev = NULL;
	err = -ESRCH;

	ve = get_ve_by_id(veid);
	if (ve == NULL)
		goto out;

	rtnl_lock();

	read_lock(&dev_base_lock);
	hlist_for_each(p, dev_name_hash(dev_name, get_ve0())) {
		struct net_device *d = hlist_entry(p, struct net_device, 
						   name_hlist);
		if (strncmp(d->name, dev_name, IFNAMSIZ) == 0) {
			dev = d;
			break;
		}
	}
	read_unlock(&dev_base_lock);
	if (dev == NULL)
		goto out_unlock;

	err = -EPERM;
	if (!ve_is_dev_movable(dev))
		goto out_unlock;

	err = -EINVAL;
	if (dev->flags & (IFF_SLAVE|IFF_MASTER))
		goto out_unlock;

	/* Check for existence of name */
	head = dev_name_hash(dev->name, ve);
	hlist_for_each(p, head) {
		struct net_device *d
			= hlist_entry(p, struct net_device, name_hlist);
		if (!strncmp(d->name, dev->name, IFNAMSIZ)) {
			err = -EEXIST;
 			goto out_unlock;
		}
 	}

	ve_netdev_cleanup(dev, 1);

	write_lock_bh(&dev_base_lock);
	__ve_dev_move(dev, get_ve0(), ve, get_exec_ub());
	write_unlock_bh(&dev_base_lock);

	err = 0;

out_unlock:
	rtnl_unlock();
	real_put_ve(ve);

	if (dev == NULL)
		printk(KERN_WARNING "Device %s not found\n", dev_name);

out:
	return err;
}

static int ve_dev_del(envid_t veid, char *dev_name)
{
	int err;
	struct net_device *dev;
	struct ve_struct *ve, *old_exec;
	struct hlist_node *p;

	dev = NULL;
	err = -ESRCH;

	ve = get_ve_by_id(veid);
	if (ve == NULL)
		goto out;

	rtnl_lock();

	read_lock(&dev_base_lock);
	hlist_for_each(p, dev_name_hash(dev_name, ve)) {
		struct net_device *d = hlist_entry(p, struct net_device, 
						   name_hlist);
		if (strncmp(d->name, dev_name, IFNAMSIZ) == 0) {
			dev = d;
			break;
		}
	}
	read_unlock(&dev_base_lock);
	if (dev == NULL)
		goto out_unlock;

	err = -EPERM;
	if (!ve_is_dev_movable(dev))
		goto out_unlock;

	old_exec = set_exec_env(ve);
	ve_netdev_cleanup(dev, 0);
	(void)set_exec_env(old_exec);

	write_lock_bh(&dev_base_lock);
	__ve_dev_move(dev, ve, get_ve0(), netdev_bc(dev)->owner_ub);
	write_unlock_bh(&dev_base_lock);

	err = 0;

out_unlock:
	rtnl_unlock();
	real_put_ve(ve);

	if (dev == NULL)
		printk(KERN_WARNING "Device %s not found\n", dev_name);

out:
	return err;
}

int real_ve_dev_map(envid_t veid, int op, char *dev_name)
{
	int err;
	err = -EPERM;
	if (!capable(CAP_SETVEID))
		goto out;
	switch (op)
	{
		case VE_NETDEV_ADD:
			err = ve_dev_add(veid, dev_name);
			break;
		case VE_NETDEV_DEL:
			err = ve_dev_del(veid, dev_name);
			break;
		default:
			err = -EINVAL;
			break;
	}
out:
	return err;
}

static void ve_mapped_devs_cleanup(struct ve_struct *ve)
{
	struct net_device *dev;

	rtnl_lock();
	write_lock_bh(&dev_base_lock);
restart:
	for (dev = ve->_net_dev_base; dev != NULL; dev = dev->next)
	{
		if ((dev->features & NETIF_F_VENET) ||
		    (dev == ve->_loopback_dev)) /* Skip loopback dev */
			continue;
		write_unlock_bh(&dev_base_lock);
		ve_netdev_cleanup(dev, 0);
		write_lock_bh(&dev_base_lock);
		__ve_dev_move(dev, ve, get_ve0(), netdev_bc(dev)->owner_ub);
		goto restart;
	}
	write_unlock_bh(&dev_base_lock);
	rtnl_unlock();
}
#endif


/**********************************************************************
 **********************************************************************
 *
 * VE information via /proc
 *
 **********************************************************************
 **********************************************************************/
#ifdef CONFIG_PROC_FS
#if BITS_PER_LONG == 32
#define VESTAT_LINE_WIDTH (6 * 11 + 6 * 21)
#define VESTAT_LINE_FMT "%10u %10lu %10lu %10lu %10Lu %20Lu %20Lu %20Lu %20Lu %20Lu %20Lu %10lu\n"
#define VESTAT_HEAD_FMT "%10s %10s %10s %10s %10s %20s %20s %20s %20s %20s %20s %10s\n"
#else
#define VESTAT_LINE_WIDTH (12 * 21)
#define VESTAT_LINE_FMT "%20u %20lu %20lu %20lu %20Lu %20Lu %20Lu %20Lu %20Lu %20Lu %20Lu %20lu\n"
#define VESTAT_HEAD_FMT "%20s %20s %20s %20s %20s %20s %20s %20s %20s %20s %20s %20s\n"
#endif

static int vestat_seq_show(struct seq_file *m, void *v)
{
	struct list_head *entry;
	struct ve_struct *ve;
	struct ve_struct *curve;
	int cpu;
	unsigned long user_ve, nice_ve, system_ve;
	unsigned long long uptime;
	cycles_t uptime_cycles, idle_time, strv_time, used;

	entry = (struct list_head *)v;
	ve = list_entry(entry, struct ve_struct, ve_list);

	curve = get_exec_env();
	if (entry == ve_list_head.next ||
	    (!ve_is_super(curve) && ve == curve)) {
		/* print header */
		seq_printf(m, "%-*s\n",
			VESTAT_LINE_WIDTH - 1,
			"Version: 2.2");
		seq_printf(m, VESTAT_HEAD_FMT, "VEID",
					"user", "nice", "system",
					"uptime", "idle",
					"strv", "uptime", "used",
					"maxlat", "totlat", "numsched");
	}

	if (ve == get_ve0())
		return 0;

	user_ve = nice_ve = system_ve = 0;
	idle_time = strv_time = used = 0;

	for_each_online_cpu(cpu) {
		struct ve_cpu_stats *st;

		st = VE_CPU_STATS(ve, cpu);
		user_ve += st->user;
		nice_ve += st->nice;
		system_ve += st->system;
		used += st->used_time;
		idle_time += __ve_sched_get_idle_time(ve, cpu);
	}
	uptime_cycles = get_cycles() - ve->start_cycles;
	uptime = get_jiffies_64() - ve->start_jiffies;

	seq_printf(m, VESTAT_LINE_FMT, ve->veid,
				user_ve, nice_ve, system_ve,
				(unsigned long long)uptime,
				(unsigned long long)idle_time, 
				(unsigned long long)strv_time,
				(unsigned long long)uptime_cycles,
				(unsigned long long)used,
				(unsigned long long)ve->sched_lat_ve.last.maxlat,
				(unsigned long long)ve->sched_lat_ve.last.totlat,
				ve->sched_lat_ve.last.count);
	return 0;
}

static void *ve_seq_start(struct seq_file *m, loff_t *pos)
{
	struct ve_struct *curve;
	struct list_head *entry;
	loff_t l;

	curve = get_exec_env();
	read_lock(&ve_list_lock);
	if (!ve_is_super(curve)) {
		if (*pos != 0)
			return NULL;
		return curve;
	}

	l = *pos;
	list_for_each(entry, &ve_list_head) {
		if (l == 0)
			return entry;
		l--;
	}
	return NULL;
}

static void *ve_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	struct list_head *entry;

	entry = (struct list_head *)v;
	if (!ve_is_super(get_exec_env()))
		return NULL;
	(*pos)++;
	return entry->next == &ve_list_head ? NULL : entry->next;
}

static void ve_seq_stop(struct seq_file *m, void *v)
{
	read_unlock(&ve_list_lock);
}

static struct seq_operations vestat_seq_op = {
        .start	= ve_seq_start,
        .next	= ve_seq_next,
        .stop	= ve_seq_stop,
        .show	= vestat_seq_show
};

static int vestat_open(struct inode *inode, struct file *file)
{
        return seq_open(file, &vestat_seq_op);
}

static struct file_operations proc_vestat_operations = {
        .open	 = vestat_open,
        .read	 = seq_read,
        .llseek	 = seq_lseek,
        .release = seq_release
};

static int vz_version_show(struct seq_file *file, void* v)
{
	static const char ver[] = VZVERSION "\n";

	return seq_puts(file, ver);
}

static int vz_version_open(struct inode *inode, struct file *file)
{
	return single_open(file, vz_version_show, NULL);
}

static struct file_operations proc_vz_version_oparations = {
	.open    = vz_version_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static inline unsigned long ve_used_mem(struct user_beancounter *ub)
{
	extern int glob_ve_meminfo;
	return glob_ve_meminfo ? ub->ub_parms[UB_OOMGUARPAGES].held :
				 ub->ub_parms[UB_PRIVVMPAGES].held ;
}

static inline void ve_mi_replace(struct meminfo *mi)
{
#ifdef CONFIG_USER_RESOURCE
	struct user_beancounter *ub;
	unsigned long meminfo_val;
	unsigned long nodettram;
	unsigned long usedmem;

	meminfo_val = get_exec_env()->meminfo_val;

	if(!meminfo_val)
		return; /* No virtualization */

	nodettram = mi->si.totalram;
	ub = current->mm->mm_ub;
	usedmem = ve_used_mem(ub);

	memset(mi, 0, sizeof(*mi));

	mi->si.totalram = (meminfo_val > nodettram) ?
			nodettram : meminfo_val;
	mi->si.freeram = (mi->si.totalram > usedmem) ?
			(mi->si.totalram - usedmem) : 0;
#else
	return;
#endif
}

static int meminfo_call(struct vnotifier_block *self,
                unsigned long event, void *arg, int old_ret)
{
	if (event != VIRTINFO_MEMINFO)
		return old_ret;

	ve_mi_replace((struct meminfo *)arg);

	return NOTIFY_OK;
}


static struct vnotifier_block meminfo_notifier_block = {
	.notifier_call = meminfo_call
};

/* /proc/vz/veinfo */

static ve_seq_print_t veaddr_seq_print_cb;

void vzmon_register_veaddr_print_cb(ve_seq_print_t cb)
{
	rcu_assign_pointer(veaddr_seq_print_cb, cb);
}
EXPORT_SYMBOL(vzmon_register_veaddr_print_cb);

void vzmon_unregister_veaddr_print_cb(ve_seq_print_t cb)
{
	rcu_assign_pointer(veaddr_seq_print_cb, NULL);
	synchronize_rcu();
}
EXPORT_SYMBOL(vzmon_unregister_veaddr_print_cb);

static int veinfo_seq_show(struct seq_file *m, void *v)
{
	struct ve_struct *ve;
	ve_seq_print_t veaddr_seq_print;

	ve = list_entry((struct list_head *)v, struct ve_struct, ve_list);

	seq_printf(m, "%10u %5u %5u", ve->veid,
			ve->class_id, atomic_read(&ve->pcounter));

	veaddr_seq_print = m->private;
	if (veaddr_seq_print)
		veaddr_seq_print(m, ve);

	seq_putc(m, '\n');
	return 0;
}

static void *veinfo_seq_start(struct seq_file *m, loff_t *pos)
{
	struct ve_struct *curve;
	struct list_head *entry;
	loff_t l;

	rcu_read_lock();
	m->private = rcu_dereference(veaddr_seq_print_cb);
	curve = get_exec_env();
	read_lock(&ve_list_lock);
	if (!ve_is_super(curve)) {
		if (*pos != 0)
			return NULL;
		return curve;
	}

	l = *pos;
	list_for_each(entry, &ve_list_head) {
		if (l == 0)
			return entry;
		l--;
	}
	return NULL;
}

static void *veinfo_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	struct list_head *entry;

	entry = (struct list_head *)v;
	if (!ve_is_super(get_exec_env()))
		return NULL;
	(*pos)++;
	return entry->next == &ve_list_head ? NULL : entry->next;
}

static void veinfo_seq_stop(struct seq_file *m, void *v)
{
	read_unlock(&ve_list_lock);
	rcu_read_unlock();
}


static struct seq_operations veinfo_seq_op = {
	.start	= veinfo_seq_start,
	.next	=  veinfo_seq_next,
	.stop	=  veinfo_seq_stop,
	.show	=  veinfo_seq_show,
};

static int veinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &veinfo_seq_op);
}

static struct file_operations proc_veinfo_operations = {
	.open		= veinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init init_vecalls_proc(void)
{
	struct proc_dir_entry *de;

	de = create_proc_glob_entry_mod("vz/vestat",
			S_IFREG|S_IRUSR, NULL, THIS_MODULE);
	if (de == NULL) {
		/* create "vz" subdirectory, if not exist */
		(void) create_proc_glob_entry("vz",
					      S_IFDIR|S_IRUGO|S_IXUGO, NULL);
		de = create_proc_glob_entry_mod("vz/vestat",
				S_IFREG|S_IRUSR, NULL, THIS_MODULE);
	}
	if (de)
		de->proc_fops = &proc_vestat_operations;
	else
		printk(KERN_WARNING 
				"VZMON: can't make vestat proc entry\n");

	de = create_proc_entry_mod("vz/devperms", S_IFREG | S_IRUSR, NULL,
				THIS_MODULE);
	if (de)
		de->proc_fops = &proc_devperms_ops;
	else
		printk(KERN_WARNING
				"VZMON: can't make devperms proc entry\n");

	
	de = create_proc_entry_mod("vz/version", S_IFREG | 0444, NULL,
				THIS_MODULE);
	if (de)
		de->proc_fops = &proc_vz_version_oparations;
	else
		printk(KERN_WARNING
				"VZMON: can't make version proc entry\n");

	de = create_proc_glob_entry_mod("vz/veinfo", S_IFREG | S_IRUSR, NULL,
				THIS_MODULE);
	if (de)
		de->proc_fops = &proc_veinfo_operations;
	else
		printk(KERN_WARNING "VZMON: can't make veinfo proc entry\n");

	virtinfo_notifier_register(VITYPE_GENERAL, &meminfo_notifier_block);

	return 0;
}

static void fini_vecalls_proc(void)
{
	remove_proc_entry("vz/version", NULL);
	remove_proc_entry("vz/devperms", NULL);
	remove_proc_entry("vz/vestat", NULL);
	remove_proc_entry("vz/veinfo", NULL);
	virtinfo_notifier_unregister(VITYPE_GENERAL, &meminfo_notifier_block);
}
#else
#define init_vecalls_proc()	(0)
#define fini_vecalls_proc()	do { } while (0)
#endif /* CONFIG_PROC_FS */


/**********************************************************************
 **********************************************************************
 *
 * User ctl
 *
 **********************************************************************
 **********************************************************************/

int vzcalls_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err;

	err = -ENOTTY;
	switch(cmd) {
	    case VZCTL_MARK_ENV_TO_DOWN: {
		        /* Compatibility issue */
		        err = 0;
		}
		break;
	    case VZCTL_SETDEVPERMS: {
			/* Device type was mistakenly declared as dev_t
			 * in the old user-kernel interface.
			 * That's wrong, dev_t is a kernel internal type.
			 * I use `unsigned' not having anything better in mind.
			 * 2001/08/11  SAW  */
			struct vzctl_setdevperms s;
			err = -EFAULT;
			if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
				break;
			err = real_setdevperms(s.veid, s.type,
					new_decode_dev(s.dev), s.mask);
		}
		break;
#ifdef CONFIG_INET
	    case VZCTL_VE_NETDEV: {
			struct vzctl_ve_netdev d;
			char *s;
			err = -EFAULT;
			if (copy_from_user(&d, (void __user *)arg, sizeof(d)))
				break;
			err = -ENOMEM;
			s = kmalloc(IFNAMSIZ+1, GFP_KERNEL);
			if (s == NULL)
				break;
			err = -EFAULT;
			if (strncpy_from_user(s, d.dev_name, IFNAMSIZ) > 0) {
				s[IFNAMSIZ] = 0;
				err = real_ve_dev_map(d.veid, d.op, s);
			}
			kfree(s);
		}
		break;
#endif
	    case VZCTL_ENV_CREATE: {
			struct vzctl_env_create s;
			err = -EFAULT;
			if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
				break;
			err = real_env_create(s.veid, s.flags, s.class_id,
				NULL, 0);
		}
		break;
	    case VZCTL_ENV_CREATE_DATA: {
			struct vzctl_env_create_data s;
			env_create_param_t *data;
			err = -EFAULT;
			if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
				break;
			err=-EINVAL;
			if (s.datalen < VZCTL_ENV_CREATE_DATA_MINLEN ||
			    s.datalen > VZCTL_ENV_CREATE_DATA_MAXLEN ||
			    s.data == 0)
				break;
			err = -ENOMEM;
			data = kzalloc(sizeof(*data), GFP_KERNEL);
			if (!data)
				break;

			err = -EFAULT;
			if (copy_from_user(data, (void __user *)s.data,
						s.datalen))
				goto free_data;
			err = real_env_create(s.veid, s.flags, s.class_id,
				data, s.datalen);
free_data:
			kfree(data);
		}
		break;
	    case VZCTL_GET_CPU_STAT: {
			struct vzctl_cpustatctl s;
			err = -EFAULT;
			if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
				break;
			err = ve_get_cpu_stat(s.veid, s.cpustat);
		}
		break;
	    case VZCTL_VE_MEMINFO: {
			struct vzctl_ve_meminfo s;
			err = -EFAULT;
			if (copy_from_user(&s, (void __user *)arg, sizeof(s)))
				break;
			err = ve_set_meminfo(s.veid, s.val);
		}
		break;
	}
	return err;
}

#ifdef CONFIG_COMPAT
int compat_vzcalls_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int err;

	switch(cmd) {
	case VZCTL_GET_CPU_STAT: {
		/* FIXME */
	}
	case VZCTL_COMPAT_ENV_CREATE_DATA: {
		struct compat_vzctl_env_create_data cs;
		struct vzctl_env_create_data __user *s;

		s = compat_alloc_user_space(sizeof(*s));
		err = -EFAULT;
		if (copy_from_user(&cs, (void *)arg, sizeof(cs)))
			break;

		if (put_user(cs.veid, &s->veid) ||
		    put_user(cs.flags, &s->flags) ||
		    put_user(cs.class_id, &s->class_id) ||
		    put_user(compat_ptr(cs.data), &s->data) ||
		    put_user(cs.datalen, &s->datalen))
			break;
		err = vzcalls_ioctl(file, VZCTL_ENV_CREATE_DATA,
						(unsigned long)s);
		break;
	}
#ifdef CONFIG_NET
	case VZCTL_COMPAT_VE_NETDEV: {
		struct compat_vzctl_ve_netdev cs;
		struct vzctl_ve_netdev __user *s;

		s = compat_alloc_user_space(sizeof(*s));
		err = -EFAULT;
		if (copy_from_user(&cs, (void *)arg, sizeof(cs)))
			break;

		if (put_user(cs.veid, &s->veid) ||
		    put_user(cs.op, &s->op) ||
		    put_user(compat_ptr(cs.dev_name), &s->dev_name))
			break;
		err = vzcalls_ioctl(file, VZCTL_VE_NETDEV, (unsigned long)s);
		break;
	}
#endif
	case VZCTL_COMPAT_VE_MEMINFO: {
		struct compat_vzctl_ve_meminfo cs;
		err = -EFAULT;
		if (copy_from_user(&cs, (void *)arg, sizeof(cs)))
			break;
		err = ve_set_meminfo(cs.veid, cs.val);
		break;
	}
	default:
		err = vzcalls_ioctl(file, cmd, arg);
		break;
	}
	return err;
}
#endif

static struct vzioctlinfo vzcalls = {
	.type		= VZCTLTYPE,
	.ioctl		= vzcalls_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= compat_vzcalls_ioctl,
#endif
	.owner		= THIS_MODULE,
};


/**********************************************************************
 **********************************************************************
 *
 * Init/exit stuff
 *
 **********************************************************************
 **********************************************************************/

static int __init init_vecalls_symbols(void)
{
	KSYMRESOLVE(real_do_env_free);
	KSYMMODRESOLVE(vzmon);
	return 0;
}

static void fini_vecalls_symbols(void)
{
	KSYMMODUNRESOLVE(vzmon);
	KSYMUNRESOLVE(real_do_env_free);
}

static inline __init int init_vecalls_ioctls(void)
{
	vzioctl_register(&vzcalls);
	return 0;
}

static inline void fini_vecalls_ioctls(void)
{
	vzioctl_unregister(&vzcalls);
}

#ifdef CONFIG_SYSCTL
static struct ctl_table_header *table_header;

static ctl_table kernel_table[] = {
	{
		.ctl_name	= KERN_VE_ALLOW_KTHREADS,
		.procname	= "ve_allow_kthreads",
		.data		= &ve_allow_kthreads,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= &proc_dointvec,
	},
	{ 0 }
};

static ctl_table root_table[] =  {
	{CTL_KERN, "kernel",  NULL, 0, 0555, kernel_table},
	{ 0 }
};

static int init_vecalls_sysctl(void)
{
	table_header = register_sysctl_table(root_table, 0);
	if (!table_header)
		return -ENOMEM ;
	return 0;
}

static void fini_vecalls_sysctl(void)
{
	unregister_sysctl_table(table_header);
} 
#else
static int init_vecalls_sysctl(void) { return 0; }
static void fini_vecalls_sysctl(void) { ; }
#endif

static int __init vecalls_init(void)
{
	int err;

	err = init_vecalls_sysctl();
	if (err)
		goto out_vzmond;

	err = init_vzmond();
	if (err < 0)
		goto out_sysctl;

	err = init_vecalls_symbols();
	if (err < 0)
		goto out_sym;

	err = init_vecalls_proc();
	if (err < 0)
		goto out_proc;

	err = init_vecalls_ioctls();
	if (err < 0)
		goto out_ioctls;

	return 0;

out_ioctls:
	fini_vecalls_proc();
out_proc:
	fini_vecalls_symbols();
out_sym:
	fini_vzmond();
out_sysctl:
	fini_vecalls_sysctl();
out_vzmond:
	return err;
}

static void vecalls_exit(void)
{
	fini_vecalls_ioctls();
	fini_vecalls_proc();
	fini_vecalls_symbols();
	fini_vzmond();
	fini_vecalls_sysctl();
}

MODULE_AUTHOR("SWsoft <info@sw-soft.com>");
MODULE_DESCRIPTION("Virtuozzo Control");
MODULE_LICENSE("GPL v2");

module_init(vecalls_init)
module_exit(vecalls_exit)
