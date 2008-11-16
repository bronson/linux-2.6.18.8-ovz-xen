#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/grsecurity.h>
#include <linux/grinternal.h>

int
gr_handle_sysctl_mod(const char *dirname, const char *name, const int op)
{
#ifdef CONFIG_GRKERNSEC_SYSCTL
	if (!strcmp(dirname, "grsecurity") && grsec_lock && (op & 002)) {
		gr_log_str(GR_DONT_AUDIT, GR_SYSCTL_MSG, name);
		return -EACCES;
	}
#endif
	return 0;
}

#ifdef CONFIG_GRKERNSEC_SYSCTL
static int grsec_proc_dointvec(ctl_table *ctl, int write, struct file * filp,
		void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;
#ifdef CONFIG_VE
	struct ctl_table fake_table;
	struct ve_struct *env = get_exec_env();

	if (!ve_is_super(env)) {
		memcpy(&fake_table, ctl, sizeof(struct ctl_table));
		fake_table.data = (char *)((unsigned long)&env->grsec +
			(unsigned long)ctl->data -
			(unsigned long)&get_ve0()->grsec);
		ctl = &fake_table;
	}
#endif
	ret = proc_dointvec(ctl, write, filp, buffer, lenp, ppos);
	return ret;
}

enum {GS_TPE = 1, GS_TPE_GID, GS_TPE_ALL, GS_LOCK};

ctl_table grsecurity_table[] = {
#ifdef CONFIG_GRKERNSEC_TPE
	{
		.ctl_name	= GS_TPE,
		.procname	= "tpe",
		.data		= &ve0.grsec.enable_tpe,
		.maxlen		= sizeof(int),
		.mode		= 0600,
		.proc_handler	= &grsec_proc_dointvec,
		.virt_handler	= 1,
	},
	{
		.ctl_name	= GS_TPE_GID,
		.procname	= "tpe_gid",
		.data		= &ve0.grsec.tpe_gid,
		.maxlen		= sizeof(int),
		.mode		= 0600,
		.proc_handler	= &grsec_proc_dointvec,
		.virt_handler	= 1,
	},
#endif
#ifdef CONFIG_GRKERNSEC_TPE_ALL
	{
		.ctl_name	= GS_TPE_ALL,
		.procname	= "tpe_restrict_all",
		.data		= &ve0.grsec.enable_tpe_all,
		.maxlen		= sizeof(int),
		.mode		= 0600,
		.proc_handler	= &grsec_proc_dointvec,
		.virt_handler	= 1,
	},
#endif
	{
		.ctl_name	= GS_LOCK,
		.procname	= "grsec_lock",
		.data		= &ve0.grsec.lock,
		.maxlen		= sizeof(int),
		.mode		= 0600,
		.proc_handler	= &grsec_proc_dointvec,
		.virt_handler	= 1,
	},
	{ .ctl_name = 0 }
};
#endif
