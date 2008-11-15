#define __KERNEL_SYSCALLS__
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/sysrq.h>
#include <asm/hypervisor.h>
#include <xen/xenbus.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#ifdef HAVE_XEN_PLATFORM_COMPAT_H
#include <xen/platform-compat.h>
#endif

MODULE_LICENSE("Dual BSD/GPL");

#define SHUTDOWN_INVALID  -1
#define SHUTDOWN_POWEROFF  0
#define SHUTDOWN_SUSPEND   2
#define SHUTDOWN_RESUMING  3
#define SHUTDOWN_HALT      4

/* Ignore multiple shutdown requests. */
static int shutting_down = SHUTDOWN_INVALID;

/* Can we leave APs online when we suspend? */
static int fast_suspend;

static void __shutdown_handler(void *unused);
static DECLARE_WORK(shutdown_work, __shutdown_handler, NULL);

int __xen_suspend(int fast_suspend, void (*resume_notifier)(void));

static int shutdown_process(void *__unused)
{
	static char *envp[] = { "HOME=/", "TERM=linux",
				"PATH=/sbin:/usr/sbin:/bin:/usr/bin", NULL };
	static char *poweroff_argv[] = { "/sbin/poweroff", NULL };

	extern asmlinkage long sys_reboot(int magic1, int magic2,
					  unsigned int cmd, void *arg);

	if ((shutting_down == SHUTDOWN_POWEROFF) ||
	    (shutting_down == SHUTDOWN_HALT)) {
		if (call_usermodehelper("/sbin/poweroff", poweroff_argv,
					envp, 0) < 0) {
#ifdef CONFIG_XEN
			sys_reboot(LINUX_REBOOT_MAGIC1,
				   LINUX_REBOOT_MAGIC2,
				   LINUX_REBOOT_CMD_POWER_OFF,
				   NULL);
#endif /* CONFIG_XEN */
		}
	}

	shutting_down = SHUTDOWN_INVALID; /* could try again */

	return 0;
}

static void xen_resume_notifier(void)
{
	int old_state = xchg(&shutting_down, SHUTDOWN_RESUMING);
	BUG_ON(old_state != SHUTDOWN_SUSPEND);
}

static int xen_suspend(void *__unused)
{
	int err, old_state;

	daemonize("suspend");
	err = set_cpus_allowed(current, cpumask_of_cpu(0));
	if (err) {
		printk(KERN_ERR "Xen suspend can't run on CPU0 (%d)\n", err);
		goto fail;
	}

	do {
		err = __xen_suspend(fast_suspend, xen_resume_notifier);
		if (err) {
			printk(KERN_ERR "Xen suspend failed (%d)\n", err);
			goto fail;
		}
		old_state = cmpxchg(
			&shutting_down, SHUTDOWN_RESUMING, SHUTDOWN_INVALID);
	} while (old_state == SHUTDOWN_SUSPEND);

	switch (old_state) {
	case SHUTDOWN_INVALID:
	case SHUTDOWN_SUSPEND:
		BUG();
	case SHUTDOWN_RESUMING:
		break;
	default:
		schedule_work(&shutdown_work);
		break;
	}

	return 0;

 fail:
	old_state = xchg(&shutting_down, SHUTDOWN_INVALID);
	BUG_ON(old_state != SHUTDOWN_SUSPEND);
	return 0;
}

static void __shutdown_handler(void *unused)
{
	int err;

	err = kernel_thread((shutting_down == SHUTDOWN_SUSPEND) ?
			    xen_suspend : shutdown_process,
			    NULL, CLONE_FS | CLONE_FILES);

	if (err < 0) {
		printk(KERN_WARNING "Error creating shutdown process (%d): "
		       "retrying...\n", -err);
		schedule_delayed_work(&shutdown_work, HZ/2);
	}
}

static void shutdown_handler(struct xenbus_watch *watch,
			     const char **vec, unsigned int len)
{
	extern void ctrl_alt_del(void);
	char *str;
	struct xenbus_transaction xbt;
	int err, old_state, new_state = SHUTDOWN_INVALID;

	if ((shutting_down != SHUTDOWN_INVALID) &&
	    (shutting_down != SHUTDOWN_RESUMING))
		return;

 again:
	err = xenbus_transaction_start(&xbt);
	if (err)
		return;

	str = (char *)xenbus_read(xbt, "control", "shutdown", NULL);
	/* Ignore read errors and empty reads. */
	if (XENBUS_IS_ERR_READ(str)) {
		xenbus_transaction_end(xbt, 1);
		return;
	}

	xenbus_write(xbt, "control", "shutdown", "");

	err = xenbus_transaction_end(xbt, 0);
	if (err == -EAGAIN) {
		kfree(str);
		goto again;
	}

	if (strcmp(str, "poweroff") == 0)
		new_state = SHUTDOWN_POWEROFF;
	else if (strcmp(str, "reboot") == 0)
		ctrl_alt_del();
	else if (strcmp(str, "suspend") == 0)
		new_state = SHUTDOWN_SUSPEND;
	else if (strcmp(str, "halt") == 0)
		new_state = SHUTDOWN_HALT;
	else
		printk("Ignoring shutdown request: %s\n", str);

	if (new_state != SHUTDOWN_INVALID) {
		old_state = xchg(&shutting_down, new_state);
		if (old_state == SHUTDOWN_INVALID)
			schedule_work(&shutdown_work);
		else
			BUG_ON(old_state != SHUTDOWN_RESUMING);
	}

	kfree(str);
}

static void sysrq_handler(struct xenbus_watch *watch, const char **vec,
			  unsigned int len)
{
	char sysrq_key = '\0';
	struct xenbus_transaction xbt;
	int err;

 again:
	err = xenbus_transaction_start(&xbt);
	if (err)
		return;
	if (!xenbus_scanf(xbt, "control", "sysrq", "%c", &sysrq_key)) {
		printk(KERN_ERR "Unable to read sysrq code in "
		       "control/sysrq\n");
		xenbus_transaction_end(xbt, 1);
		return;
	}

	if (sysrq_key != '\0')
		xenbus_printf(xbt, "control", "sysrq", "%c", '\0');

	err = xenbus_transaction_end(xbt, 0);
	if (err == -EAGAIN)
		goto again;

#ifdef CONFIG_MAGIC_SYSRQ
	if (sysrq_key != '\0')
		handle_sysrq(sysrq_key, NULL, NULL);
#endif
}

static struct xenbus_watch shutdown_watch = {
	.node = "control/shutdown",
	.callback = shutdown_handler
};

static struct xenbus_watch sysrq_watch = {
	.node = "control/sysrq",
	.callback = sysrq_handler
};

static int setup_shutdown_watcher(void)
{
	int err;

	xenbus_scanf(XBT_NIL, "control",
		     "platform-feature-multiprocessor-suspend",
		     "%d", &fast_suspend);

	err = register_xenbus_watch(&shutdown_watch);
	if (err) {
		printk(KERN_ERR "Failed to set shutdown watcher\n");
		return err;
	}

	err = register_xenbus_watch(&sysrq_watch);
	if (err) {
		printk(KERN_ERR "Failed to set sysrq watcher\n");
		return err;
	}

	return 0;
}

#ifdef CONFIG_XEN

static int shutdown_event(struct notifier_block *notifier,
			  unsigned long event,
			  void *data)
{
	setup_shutdown_watcher();
	return NOTIFY_DONE;
}

static int __init setup_shutdown_event(void)
{
	static struct notifier_block xenstore_notifier = {
		.notifier_call = shutdown_event
	};
	register_xenstore_notifier(&xenstore_notifier);

	return 0;
}

subsys_initcall(setup_shutdown_event);

#else /* !defined(CONFIG_XEN) */

int xen_reboot_init(void)
{
	return setup_shutdown_watcher();
}

#endif /* !defined(CONFIG_XEN) */
