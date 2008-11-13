#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/smp_lock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/percpu.h>
#include <linux/module.h>

#ifdef CONFIG_VE
#include <linux/grinternal.h>
#else
int grsec_enable_tpe;
int grsec_tpe_gid;
int grsec_enable_tpe_all;
int grsec_lock;
#endif

spinlock_t grsec_alert_lock = SPIN_LOCK_UNLOCKED;

unsigned long grsec_alert_wtime = 0;
unsigned long grsec_alert_fyet = 0;

spinlock_t grsec_audit_lock = SPIN_LOCK_UNLOCKED;

char *gr_shared_page[4];

char *gr_alert_log_fmt;
char *gr_audit_log_fmt;

char *gr_alert_log_buf;
char *gr_audit_log_buf;

void grsecurity_setup(void)
{
#if !defined(CONFIG_GRKERNSEC_SYSCTL) || defined(CONFIG_GRKERNSEC_SYSCTL_ON)
#ifndef CONFIG_GRKERNSEC_SYSCTL
	grsec_lock = 1;
#endif
#ifdef CONFIG_GRKERNSEC_TPE
	grsec_enable_tpe = 1;
	grsec_tpe_gid = CONFIG_GRKERNSEC_TPE_GID;
#ifdef CONFIG_GRKERNSEC_TPE_ALL
	grsec_enable_tpe_all = 1;
#endif
#endif
#endif
}
EXPORT_SYMBOL(grsecurity_setup);

void
grsecurity_init(void)
{
	int j;
	/* create the per-cpu shared pages */

	for (j = 0; j < 4; j++) {
		gr_shared_page[j] = (char *)__alloc_percpu(PAGE_SIZE);
		if (gr_shared_page[j] == NULL) {
			panic("Unable to allocate grsecurity shared page");
			return;
		}
	}

	/* allocate log buffers */
	gr_alert_log_fmt = kmalloc(512, GFP_KERNEL);
	if (!gr_alert_log_fmt) {
		panic("Unable to allocate grsecurity alert log format buffer");
		return;
	}
	gr_audit_log_fmt = kmalloc(512, GFP_KERNEL);
	if (!gr_audit_log_fmt) {
		panic("Unable to allocate grsecurity audit log format buffer");
		return;
	}
	gr_alert_log_buf = (char *) get_zeroed_page(GFP_KERNEL);
	if (!gr_alert_log_buf) {
		panic("Unable to allocate grsecurity alert log buffer");
		return;
	}
	gr_audit_log_buf = (char *) get_zeroed_page(GFP_KERNEL);
	if (!gr_audit_log_buf) {
		panic("Unable to allocate grsecurity audit log buffer");
		return;
	}
	grsecurity_setup();

	return;
}
