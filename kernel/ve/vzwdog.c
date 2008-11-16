/*
 *  kernel/ve/vzwdog.c
 *
 *  Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/kobject.h>
#include <linux/genhd.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/suspend.h>
#include <linux/ve.h>
#include <linux/vzstat.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>

/* Staff regading kernel thread polling VE validity */
static int sleep_timeout = 60;
static struct task_struct *wdog_thread_tsk;

extern void show_mem(void);

static struct file *intr_file;
static char page[PAGE_SIZE];

static void parse_irq_list(int len)
{
	int i, k, skip;
	for (i = 0; i < len; ) {
		k = i;
		while (i < len && page[i] != '\n' && page[i] != ':')
			i++;
		skip = 0;
		if (i < len && page[i] != '\n') {
			i++; /* skip ':' */
			while (i < len && (page[i] == ' ' || page[i] == '0'))
				i++;
			skip = (i < len && (page[i] < '0' || page[i] > '9'));
			while (i < len && page[i] != '\n')
				i++;
		}
		if (!skip)
			printk("%.*s\n", i - k, page + k);
		if (i < len)
			i++; /* skip '\n' */
	}
}

extern loff_t vfs_llseek(struct file *file, loff_t, int);
extern ssize_t vfs_read(struct file *file, char __user *, size_t, loff_t *);
extern struct file *filp_open(const char *filename, int flags, int mode);
extern int filp_close(struct file *filp, fl_owner_t id);
static void show_irq_list(void)
{
	mm_segment_t fs;
	int r;

	fs = get_fs();
	set_fs(KERNEL_DS);
	vfs_llseek(intr_file, 0, 0);
	r = vfs_read(intr_file, (void __user *)page, sizeof(page),
			&intr_file->f_pos);
	set_fs(fs);

	if (r > 0)
		parse_irq_list(r);
}

static void show_alloc_latency(void)
{
	static const char *alloc_descr[KSTAT_ALLOCSTAT_NR] = {
		"A0",
		"L0",
		"H0",
		"L1",
		"H1"
	};
	int i;

	printk("lat: ");
	for (i = 0; i < KSTAT_ALLOCSTAT_NR; i++) {
		struct kstat_lat_struct *p;
		cycles_t maxlat, avg0, avg1, avg2;

		p = &kstat_glob.alloc_lat[i];
		spin_lock_irq(&kstat_glb_lock);
		maxlat = p->last.maxlat;
		avg0 = p->avg[0];
		avg1 = p->avg[1];
		avg2 = p->avg[2];
		spin_unlock_irq(&kstat_glb_lock);

		printk("%s %Lu (%Lu %Lu %Lu)",
				alloc_descr[i],
				(unsigned long long)maxlat,
				(unsigned long long)avg0,
				(unsigned long long)avg1,
				(unsigned long long)avg2);
	}
	printk("\n");
}

static void show_schedule_latency(void)
{
	struct kstat_lat_pcpu_struct *p;
	cycles_t maxlat, totlat, avg0, avg1, avg2;
	unsigned long count;

	p = &kstat_glob.sched_lat;
	spin_lock_irq(&kstat_glb_lock);
	maxlat = p->last.maxlat;
	totlat = p->last.totlat;
	count = p->last.count;
	avg0 = p->avg[0];
	avg1 = p->avg[1];
	avg2 = p->avg[2];
	spin_unlock_irq(&kstat_glb_lock);

	printk("sched lat: %Lu/%Lu/%lu (%Lu %Lu %Lu)\n",
			(unsigned long long)maxlat,
			(unsigned long long)totlat,
			count,
			(unsigned long long)avg0,
			(unsigned long long)avg1,
			(unsigned long long)avg2);
}

static void show_header(void)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	preempt_disable();
	printk("*** VZWDOG 1.14: time %lu.%06lu uptime %Lu CPU %d ***\n",
			tv.tv_sec, (long)tv.tv_usec,
			(unsigned long long)get_jiffies_64(),
			smp_processor_id());
#ifdef CONFIG_FAIRSCHED
	printk("*** cycles_per_jiffy %lu jiffies_per_second %u ***\n",
			cycles_per_jiffy, HZ);
#else
	printk("*** jiffies_per_second %u ***\n", HZ);
#endif
	preempt_enable();
}

static void show_pgdatinfo(void)
{
	pg_data_t *pgdat;

	printk("pgdat:");
	for_each_online_pgdat(pgdat) {
		printk(" %d: %lu,%lu,%lu",
				pgdat->node_id,
				pgdat->node_start_pfn,
				pgdat->node_present_pages,
				pgdat->node_spanned_pages);
#ifdef CONFIG_FLAT_NODE_MEM_MAP
		printk(",%p", pgdat->node_mem_map);
#endif
	}
	printk("\n");
}

static void show_diskio(void)
{
	struct gendisk *gd;
	char buf[BDEVNAME_SIZE];

	printk("disk_io: ");

	down_read(&block_subsys.rwsem);
	list_for_each_entry(gd, &block_subsys.kset.list, kobj.entry) {
		char *name;
		name = disk_name(gd, 0, buf);
		if ((strlen(name) > 4) && (strncmp(name, "loop", 4) == 0) &&
		    isdigit(name[4]))
			continue;
		if ((strlen(name) > 3) && (strncmp(name, "ram", 3) == 0) &&
		    isdigit(name[3]))
			continue;
		printk("(%u,%u) %s r(%lu %lu %lu) w(%lu %lu %lu)\n",
			gd->major, gd->first_minor,
			name,
			disk_stat_read(gd, ios[READ]),
			disk_stat_read(gd, sectors[READ]),
			disk_stat_read(gd, merges[READ]),
			disk_stat_read(gd, ios[WRITE]),
			disk_stat_read(gd, sectors[WRITE]),
			disk_stat_read(gd, merges[WRITE]));
	}
	up_read(&block_subsys.rwsem);

	printk("\n");
}

static void show_nrprocs(void)
{
	unsigned long _nr_running, _nr_sleeping,
			_nr_unint, _nr_zombie, _nr_dead, _nr_stopped;

	_nr_running = nr_running();
	_nr_unint = nr_uninterruptible();
	_nr_sleeping = nr_sleeping();
	_nr_zombie = nr_zombie;
	_nr_dead = atomic_read(&nr_dead);
	_nr_stopped = nr_stopped();

	printk("VEnum: %d, proc R %lu, S %lu, D %lu, "
		"Z %lu, X %lu, T %lu (tot %d)\n",
		nr_ve,	_nr_running, _nr_sleeping, _nr_unint,
		_nr_zombie, _nr_dead, _nr_stopped, nr_threads);
}

static void wdog_print(void)
{
	show_header();
	show_irq_list();
	show_pgdatinfo();
	show_mem();
	show_diskio();
	show_schedule_latency();
	show_alloc_latency();
	show_nrprocs();
}

static int wdog_loop(void* data)
{
	while (1) {
		wdog_print();
		try_to_freeze();

		set_current_state(TASK_UNINTERRUPTIBLE);
		if (kthread_should_stop())
			break;
		schedule_timeout(sleep_timeout*HZ);
	}
	return 0;
}

static int __init wdog_init(void)
{
	struct file *file;

	file = filp_open("/proc/interrupts", 0, 0);
	if (IS_ERR(file))
		return PTR_ERR(file);
	intr_file = file;

	wdog_thread_tsk = kthread_run(wdog_loop, NULL, "vzwdog");
	if (IS_ERR(wdog_thread_tsk)) {
		filp_close(intr_file, NULL);
		return -EBUSY;
	}
	return 0;
}

static void __exit wdog_exit(void)
{
	kthread_stop(wdog_thread_tsk);
	filp_close(intr_file, NULL);
}

module_param(sleep_timeout, int, 0666);
MODULE_AUTHOR("SWsoft <info@sw-soft.com>");
MODULE_DESCRIPTION("Virtuozzo WDOG");
MODULE_LICENSE("GPL v2");

module_init(wdog_init)
module_exit(wdog_exit)
