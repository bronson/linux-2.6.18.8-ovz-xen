/* -*- linux-c -*-
 *
 *	$Id: sysrq.c,v 1.15 1998/08/23 14:56:41 mj Exp $
 *
 *	Linux Magic System Request Key Hacks
 *
 *	(c) 1997 Martin Mares <mj@atrey.karlin.mff.cuni.cz>
 *	based on ideas by Pavel Machek <pavel@atrey.karlin.mff.cuni.cz>
 *
 *	(c) 2000 Crutcher Dunnavant <crutcher+kernel@datastacks.com>
 *	overhauled to use key registration
 *	based upon discusions in irc://irc.openprojects.net/#kernelnewbies
 */

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/mount.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/reboot.h>
#include <linux/sysrq.h>
#include <linux/kbd_kern.h>
#include <linux/quotaops.h>
#include <linux/smp_lock.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/writeback.h>
#include <linux/buffer_head.h>		/* for fsync_bdev() */
#include <linux/swap.h>
#include <linux/spinlock.h>
#include <linux/vt_kern.h>
#include <linux/workqueue.h>
#include <linux/kexec.h>
#include <linux/kallsyms.h>
#include <linux/slab.h>

#include <asm/ptrace.h>

/* Whether we react on sysrq keys or just ignore them */
int sysrq_enabled = 1;

static void sysrq_handle_loglevel(int key, struct pt_regs *pt_regs,
				  struct tty_struct *tty)
{
	int i;
	i = key - '0';
	console_loglevel = 7;
	printk("Loglevel set to %d\n", i);
	console_loglevel = i;
}
static struct sysrq_key_op sysrq_loglevel_op = {
	.handler	= sysrq_handle_loglevel,
	.help_msg	= "loglevel0-8",
	.action_msg	= "Changing Loglevel",
	.enable_mask	= SYSRQ_ENABLE_LOG,
};

#ifdef CONFIG_VT
static void sysrq_handle_SAK(int key, struct pt_regs *pt_regs,
			     struct tty_struct *tty)
{
	if (tty)
		do_SAK(tty);
	reset_vc(vc_cons[fg_console].d);
}
static struct sysrq_key_op sysrq_SAK_op = {
	.handler	= sysrq_handle_SAK,
	.help_msg	= "saK",
	.action_msg	= "SAK",
	.enable_mask	= SYSRQ_ENABLE_KEYBOARD,
};
#else
#define sysrq_SAK_op (*(struct sysrq_key_op *)0)
#endif

#ifdef CONFIG_VT
static void sysrq_handle_unraw(int key, struct pt_regs *pt_regs,
			       struct tty_struct *tty)
{
	struct kbd_struct *kbd = &kbd_table[fg_console];

	if (kbd)
		kbd->kbdmode = VC_XLATE;
}
static struct sysrq_key_op sysrq_unraw_op = {
	.handler	= sysrq_handle_unraw,
	.help_msg	= "unRaw",
	.action_msg	= "Keyboard mode set to XLATE",
	.enable_mask	= SYSRQ_ENABLE_KEYBOARD,
};
#else
#define sysrq_unraw_op (*(struct sysrq_key_op *)0)
#endif /* CONFIG_VT */

#ifdef CONFIG_KEXEC
static void sysrq_handle_crashdump(int key, struct pt_regs *pt_regs,
				struct tty_struct *tty)
{
	crash_kexec(pt_regs);
}
static struct sysrq_key_op sysrq_crashdump_op = {
	.handler	= sysrq_handle_crashdump,
	.help_msg	= "Crashdump",
	.action_msg	= "Trigger a crashdump",
	.enable_mask	= SYSRQ_ENABLE_DUMP,
};
#else
#define sysrq_crashdump_op (*(struct sysrq_key_op *)0)
#endif

static void sysrq_handle_reboot(int key, struct pt_regs *pt_regs,
				struct tty_struct *tty)
{
	local_irq_enable();
	emergency_restart();
}
static struct sysrq_key_op sysrq_reboot_op = {
	.handler	= sysrq_handle_reboot,
	.help_msg	= "reBoot",
	.action_msg	= "Resetting",
	.enable_mask	= SYSRQ_ENABLE_BOOT,
};

static void sysrq_handle_sync(int key, struct pt_regs *pt_regs,
			      struct tty_struct *tty)
{
	emergency_sync();
}
static struct sysrq_key_op sysrq_sync_op = {
	.handler	= sysrq_handle_sync,
	.help_msg	= "Sync",
	.action_msg	= "Emergency Sync",
	.enable_mask	= SYSRQ_ENABLE_SYNC,
};

static void sysrq_handle_mountro(int key, struct pt_regs *pt_regs,
				 struct tty_struct *tty)
{
	emergency_remount();
}
static struct sysrq_key_op sysrq_mountro_op = {
	.handler	= sysrq_handle_mountro,
	.help_msg	= "Unmount",
	.action_msg	= "Emergency Remount R/O",
	.enable_mask	= SYSRQ_ENABLE_REMOUNT,
};

#ifdef CONFIG_LOCKDEP
static void sysrq_handle_showlocks(int key, struct pt_regs *pt_regs,
				struct tty_struct *tty)
{
	debug_show_all_locks();
}

static struct sysrq_key_op sysrq_showlocks_op = {
	.handler	= sysrq_handle_showlocks,
	.help_msg	= "show-all-locks(D)",
	.action_msg	= "Show Locks Held",
};
#else
#define sysrq_showlocks_op (*(struct sysrq_key_op *)0)
#endif

static void sysrq_handle_showregs(int key, struct pt_regs *pt_regs,
				  struct tty_struct *tty)
{
	bust_spinlocks(1);
	if (pt_regs)
		show_regs(pt_regs);
	bust_spinlocks(0);
#if defined(__i386__) || defined(__x86_64__)
	smp_nmi_call_function(smp_show_regs, NULL, 1);
#endif
}
static struct sysrq_key_op sysrq_showregs_op = {
	.handler	= sysrq_handle_showregs,
	.help_msg	= "showPc",
	.action_msg	= "Show Regs",
	.enable_mask	= SYSRQ_ENABLE_DUMP,
};

static void sysrq_handle_showstate(int key, struct pt_regs *pt_regs,
				   struct tty_struct *tty)
{
	show_state();
}
static struct sysrq_key_op sysrq_showstate_op = {
	.handler	= sysrq_handle_showstate,
	.help_msg	= "showTasks",
	.action_msg	= "Show State",
	.enable_mask	= SYSRQ_ENABLE_DUMP,
};

static void sysrq_handle_showmem(int key, struct pt_regs *pt_regs,
				 struct tty_struct *tty)
{
	show_mem();
	show_slab_info();
}
static struct sysrq_key_op sysrq_showmem_op = {
	.handler	= sysrq_handle_showmem,
	.help_msg	= "showMem",
	.action_msg	= "Show Memory",
	.enable_mask	= SYSRQ_ENABLE_DUMP,
};

/*
 * Signal sysrq helper function.  Sends a signal to all user processes.
 */
static void send_sig_all(int sig)
{
	struct task_struct *p;

	for_each_process_all(p) {
		if (p->mm && p->pid != 1)
			/* Not swapper, init nor kernel thread */
			force_sig(sig, p);
	}
}

static void sysrq_handle_term(int key, struct pt_regs *pt_regs,
			      struct tty_struct *tty)
{
	send_sig_all(SIGTERM);
	console_loglevel = 8;
}
static struct sysrq_key_op sysrq_term_op = {
	.handler	= sysrq_handle_term,
	.help_msg	= "tErm",
	.action_msg	= "Terminate All Tasks",
	.enable_mask	= SYSRQ_ENABLE_SIGNAL,
};

static void moom_callback(void *ignored)
{
	out_of_memory(&NODE_DATA(0)->node_zonelists[ZONE_NORMAL],
			GFP_KERNEL, 0);
}

static DECLARE_WORK(moom_work, moom_callback, NULL);

static void sysrq_handle_moom(int key, struct pt_regs *pt_regs,
			      struct tty_struct *tty)
{
	schedule_work(&moom_work);
}
static struct sysrq_key_op sysrq_moom_op = {
	.handler	= sysrq_handle_moom,
	.help_msg	= "Full",
	.action_msg	= "Manual OOM execution",
};

static void sysrq_handle_kill(int key, struct pt_regs *pt_regs,
			      struct tty_struct *tty)
{
	send_sig_all(SIGKILL);
	console_loglevel = 8;
}
static struct sysrq_key_op sysrq_kill_op = {
	.handler	= sysrq_handle_kill,
	.help_msg	= "kIll",
	.action_msg	= "Kill All Tasks",
	.enable_mask	= SYSRQ_ENABLE_SIGNAL,
};

#ifdef CONFIG_SCHED_VCPU
static void sysrq_handle_vschedstate(int key, struct pt_regs *pt_regs,
				   struct tty_struct *tty) 
{
	show_vsched();
}
static struct sysrq_key_op sysrq_vschedstate_op = {
	.handler	= sysrq_handle_vschedstate,
	.help_msg	= "vsced_stAte",
	.action_msg	= "Show Vsched",
};
#endif

static void sysrq_handle_unrt(int key, struct pt_regs *pt_regs,
				struct tty_struct *tty)
{
	normalize_rt_tasks();
}
static struct sysrq_key_op sysrq_unrt_op = {
	.handler	= sysrq_handle_unrt,
	.help_msg	= "Nice",
	.action_msg	= "Nice All RT Tasks",
	.enable_mask	= SYSRQ_ENABLE_RTNICE,
};

/* Key Operations table and lock */
static DEFINE_SPINLOCK(sysrq_key_table_lock);

#define SYSRQ_KEY_TABLE_LENGTH 37
static struct sysrq_key_op **sysrq_key_table;
static struct sysrq_key_op *sysrq_default_key_table[];

#ifdef CONFIG_SYSRQ_DEBUG
#define SYSRQ_NAMELEN_MAX	64
#define SYSRQ_DUMP_LINES	32

static struct sysrq_key_op *sysrq_debug_key_table[];
static struct sysrq_key_op *sysrq_input_key_table[];
static unsigned long *dump_address;
static int orig_console_loglevel;
static void (*sysrq_input_return)(char *) = NULL;

static void dump_mem(void)
{
	unsigned long value[4];
	mm_segment_t old_fs;
	int line, err;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	err = 0;

	for (line = 0; line < SYSRQ_DUMP_LINES; line++) {
		err |= __get_user(value[0], dump_address++);
		err |= __get_user(value[1], dump_address++);
		err |= __get_user(value[2], dump_address++);
		err |= __get_user(value[3], dump_address++);
		if (err) {
			printk("Invalid address %p\n", dump_address - 4);
			break;
		}
#if BITS_PER_LONG == 32
		printk("0x%p: %08lx %08lx %08lx %08lx\n",
				dump_address - 4,
				value[0], value[1], value[2], value[3]);
#else
		printk("0x%p: %016lx %016lx %016lx %016lx\n",
				dump_address - 4,
				value[0], value[1], value[2], value[3]);
#endif
	}
	set_fs(old_fs);
}

static void write_mem(unsigned long val)
{
	mm_segment_t old_fs;
	unsigned long old_val;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	if (__get_user(old_val, dump_address)) {
		printk("Invalid address %p\n", dump_address);
		goto out;
	}

#if BITS_PER_LONG == 32
	printk("Changing [%p] from %08lx to %08lx\n",
			dump_address, old_val, val);
#else
	printk("Changing [%p] from %016lx to %016lx\n",
			dump_address, old_val, val);
#endif
	__put_user(val, dump_address);
out:
	set_fs(old_fs);
}

static void handle_read(int key, struct pt_regs *pt_regs,
		struct tty_struct *tty)
{
	static int pos;
	static int upper_case;
	static char str[SYSRQ_NAMELEN_MAX];

	if (key == 0) {
		/* actually 0 is not shift only... */
		upper_case = 1;
		return;
	}

	if (key == 0x0d || pos == SYSRQ_NAMELEN_MAX - 1) {
		/* enter */
		sysrq_key_table = sysrq_debug_key_table;
		str[pos] = '\0';
		pos = upper_case = 0;
		printk("\n");
		if (sysrq_input_return == NULL)
			printk("No return handler!!!\n");
		else
			sysrq_input_return(str);
		return;
	};

	/* check for alowed symbols */
	if (key == '-') {
		if (upper_case)
			key = '_';
		goto correct;
	};
	if (key >= 'a' && key <= 'z') {
		if (upper_case)
			key = key - 'a' + 'A';
		goto correct;
	};
	if (key >= '0' && key <= '9')
		goto correct;

	upper_case = 0;
	return;

correct:
	str[pos] = key;
	printk("%c", (char)key);
	pos++;
	upper_case = 0;
}

static struct sysrq_key_op input_read = {
	.handler	= handle_read,
	.help_msg	= "",
	.action_msg	= NULL,
};

static struct sysrq_key_op *sysrq_input_key_table[SYSRQ_KEY_TABLE_LENGTH] = {
	[0 ... SYSRQ_KEY_TABLE_LENGTH - 1] = &input_read,
};

static void return_dump_mem(char *str)
{
	unsigned long address;
	char *end;

	address = simple_strtoul(str, &end, 0);
	if (*end != '\0') {
		printk("Bad address [%s]\n", str);
		return;
	}

	dump_address = (unsigned long *)address;
	dump_mem();
}

static void handle_dump_mem(int key, struct pt_regs *pt_regs,
		struct tty_struct *tty)
{
	sysrq_input_return = return_dump_mem;
	sysrq_key_table = sysrq_input_key_table;
}

static struct sysrq_key_op debug_dump_mem = {
	.handler	= handle_dump_mem,
	.help_msg	= "Dump",
	.action_msg	= "Enter address:",
};

static void return_resolve(char *str)
{
	unsigned long address;

	address = kallsyms_lookup_name(str);
	printk("%s : %lx\n", str, address);
	if (address) {
		dump_address = (unsigned long *)address;
		printk("Now you can dump it via X\n");
	}
}

static void handle_resolve(int key, struct pt_regs *pt_regs,
		struct tty_struct *tty)
{
	sysrq_input_return = return_resolve;
	sysrq_key_table = sysrq_input_key_table;
}

static struct sysrq_key_op debug_resolve = {
	.handler	= handle_resolve,
	.help_msg	= "Resolve",
	.action_msg	= "Enter symbol name:",
};

static void return_write_mem(char *str)
{
	unsigned long address;
	unsigned long value;
	char *end;

	address = simple_strtoul(str, &end, 0);
	if (*end != '-') {
		printk("Bad address in %s\n", str);
		return;
	}
	value = simple_strtoul(end + 1, &end, 0);
	if (*end != '\0') {
		printk("Bad value in %s\n", str);
		return;
	}

	dump_address = (unsigned long *)address;
	write_mem(value);
}

static void handle_write_mem(int key, struct pt_regs *pt_regs,
		struct tty_struct *tty)
{
	sysrq_input_return = return_write_mem;
	sysrq_key_table = sysrq_input_key_table;
}

static struct sysrq_key_op debug_write_mem = {
	.handler	= handle_write_mem,
	.help_msg	= "Writemem",
	.action_msg	= "Enter address-value:",
};

static void handle_next(int key, struct pt_regs *pt_regs,
		struct tty_struct *tty)
{
	dump_mem();
}

static struct sysrq_key_op debug_next = {
	.handler	= handle_next,
	.help_msg	= "neXt",
	.action_msg	= "continuing",
};

static void handle_quit(int key, struct pt_regs *pt_regs,
		struct tty_struct *tty)
{
	sysrq_key_table = sysrq_default_key_table;
	console_loglevel = orig_console_loglevel;
}

static struct sysrq_key_op debug_quit = {
	.handler	= handle_quit,
	.help_msg	= "Quit",
	.action_msg	= "Tnahk you for using debugger",
};

static struct sysrq_key_op *sysrq_debug_key_table[SYSRQ_KEY_TABLE_LENGTH] = {
	[13] = &debug_dump_mem,		/* d */
	[26] = &debug_quit,		/* q */
	[27] = &debug_resolve,		/* r */
	[32] = &debug_write_mem,	/* w */
	[33] = &debug_next,		/* x */
};

static void sysrq_handle_debug(int key, struct pt_regs *pt_regs,
		struct tty_struct *tty)
{
	orig_console_loglevel = console_loglevel;
	console_loglevel = 8;
	sysrq_key_table = sysrq_debug_key_table;
	printk("Welcome sysrq debugging mode\n"
			"Press H for help\n");
}

static struct sysrq_key_op sysrq_debug_op = {
	.handler        = sysrq_handle_debug,
	.help_msg       = "debuG",
	.action_msg     = "Select desired action",
};
#endif

static struct sysrq_key_op *sysrq_default_key_table[SYSRQ_KEY_TABLE_LENGTH] = {
	&sysrq_loglevel_op,		/* 0 */
	&sysrq_loglevel_op,		/* 1 */
	&sysrq_loglevel_op,		/* 2 */
	&sysrq_loglevel_op,		/* 3 */
	&sysrq_loglevel_op,		/* 4 */
	&sysrq_loglevel_op,		/* 5 */
	&sysrq_loglevel_op,		/* 6 */
	&sysrq_loglevel_op,		/* 7 */
	&sysrq_loglevel_op,		/* 8 */
	&sysrq_loglevel_op,		/* 9 */

	/*
	 * Don't use for system provided sysrqs, it is handled specially on
	 * sparc and will never arrive
	 */
#ifdef CONFIG_SCHED_VCPU
	&sysrq_vschedstate_op,		/* a */
#else
	NULL,				/* a */
#endif
	&sysrq_reboot_op,		/* b */
	&sysrq_crashdump_op,		/* c */
	&sysrq_showlocks_op,		/* d */
	&sysrq_term_op,			/* e */
	&sysrq_moom_op,			/* f */
#ifdef CONFIG_SYSRQ_DEBUG
	&sysrq_debug_op,		/* g */
#else
	NULL,				/* g */
#endif
	NULL,				/* h */
	&sysrq_kill_op,			/* i */
	NULL,				/* j */
	&sysrq_SAK_op,			/* k */
	NULL,				/* l */
	&sysrq_showmem_op,		/* m */
	&sysrq_unrt_op,			/* n */
	/* This will often be registered as 'Off' at init time */
	NULL,				/* o */
	&sysrq_showregs_op,		/* p */
	NULL,				/* q */
	&sysrq_unraw_op,			/* r */
	&sysrq_sync_op,			/* s */
	&sysrq_showstate_op,		/* t */
	&sysrq_mountro_op,		/* u */
	/* May be assigned at init time by SMP VOYAGER */
	NULL,				/* v */
	NULL,				/* w */
	NULL,				/* x */
	NULL,				/* y */
	NULL,				/* z */
	NULL,				/* for debugger */
};

static struct sysrq_key_op **sysrq_key_table = sysrq_default_key_table;

/* key2index calculation, -1 on invalid index */
static int sysrq_key_table_key2index(int key)
{
	int retval;

	if ((key >= '0') && (key <= '9'))
		retval = key - '0';
	else if ((key >= 'a') && (key <= 'z'))
		retval = key + 10 - 'a';
#ifdef CONFIG_SYSRQ_DEBUG
	else if (key == 0 || key == 0x0d || key == '-')
		retval = SYSRQ_KEY_TABLE_LENGTH - 1;
#endif
	else
		retval = -1;
	return retval;
}

/*
 * get and put functions for the table, exposed to modules.
 */
struct sysrq_key_op *__sysrq_get_key_op(int key)
{
        struct sysrq_key_op *op_p = NULL;
        int i;

	i = sysrq_key_table_key2index(key);
	if (i != -1)
	        op_p = sysrq_key_table[i];
        return op_p;
}

static void __sysrq_put_key_op(int key, struct sysrq_key_op *op_p)
{
        int i = sysrq_key_table_key2index(key);

        if (i != -1)
                sysrq_key_table[i] = op_p;
}

/*
 * This is the non-locking version of handle_sysrq.  It must/can only be called
 * by sysrq key handlers, as they are inside of the lock
 */
void __handle_sysrq(int key, struct pt_regs *pt_regs, struct tty_struct *tty,
			int check_mask)
{
	struct sysrq_key_op *op_p;
	int orig_log_level;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&sysrq_key_table_lock, flags);
	orig_log_level = console_loglevel;
	console_loglevel = 7;

        op_p = __sysrq_get_key_op(key);
        if (op_p) {
		/*
		 * Should we check for enabled operations (/proc/sysrq-trigger
		 * should not) and is the invoked operation enabled?
		 */
		if (!check_mask || sysrq_enabled == 1 ||
		    (sysrq_enabled & op_p->enable_mask)) {
		    	if (op_p->action_msg)
				printk("SysRq: %s\n", op_p->action_msg);
			console_loglevel = orig_log_level;
			op_p->handler(key, pt_regs, tty);
		} else {
			printk("This sysrq operation is disabled.\n");
		}
	} else {
		printk("SysRq HELP : ");
		/* Only print the help msg once per handler */
		for (i = 0; i < SYSRQ_KEY_TABLE_LENGTH; i++) {
			if (sysrq_key_table[i]) {
				int j;

				for (j = 0; sysrq_key_table[i] !=
						sysrq_key_table[j]; j++)
					;
				if (j != i)
					continue;
				printk("%s ", sysrq_key_table[i]->help_msg);
			}
		}
		printk("\n");
		console_loglevel = orig_log_level;
	}
	spin_unlock_irqrestore(&sysrq_key_table_lock, flags);
}

/*
 * This function is called by the keyboard handler when SysRq is pressed
 * and any other keycode arrives.
 */
void handle_sysrq(int key, struct pt_regs *pt_regs, struct tty_struct *tty)
{
	if (!sysrq_enabled)
		return;
	__handle_sysrq(key, pt_regs, tty, 1);
}
EXPORT_SYMBOL(handle_sysrq);

static int __sysrq_swap_key_ops(int key, struct sysrq_key_op *insert_op_p,
                                struct sysrq_key_op *remove_op_p)
{

	int retval;
	unsigned long flags;

	spin_lock_irqsave(&sysrq_key_table_lock, flags);
	if (__sysrq_get_key_op(key) == remove_op_p) {
		__sysrq_put_key_op(key, insert_op_p);
		retval = 0;
	} else {
		retval = -1;
	}
	spin_unlock_irqrestore(&sysrq_key_table_lock, flags);
	return retval;
}

int register_sysrq_key(int key, struct sysrq_key_op *op_p)
{
	return __sysrq_swap_key_ops(key, op_p, NULL);
}
EXPORT_SYMBOL(register_sysrq_key);

int unregister_sysrq_key(int key, struct sysrq_key_op *op_p)
{
	return __sysrq_swap_key_ops(key, NULL, op_p);
}
EXPORT_SYMBOL(unregister_sysrq_key);
