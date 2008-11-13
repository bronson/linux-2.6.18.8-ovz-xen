#ifndef __GRINTERNAL_H
#define __GRINTERNAL_H

#ifdef CONFIG_GRKERNSEC

#include <linux/grmsg.h>

extern char *gr_to_filename(const struct dentry *dentry,
			    const struct vfsmount *mnt);
extern char *gr_to_filename2(const struct dentry *dentry,
			    const struct vfsmount *mnt);
extern char *gr_to_filename3(const struct dentry *dentry,
			    const struct vfsmount *mnt);

#ifdef CONFIG_VE
#include <linux/ve_task.h>
#define grsec_enable_tpe		(get_exec_env()->grsec.enable_tpe)
#define grsec_tpe_gid			(get_exec_env()->grsec.tpe_gid)
#define grsec_enable_tpe_all		(get_exec_env()->grsec.enable_tpe_all)
#define grsec_lock			(get_exec_env()->grsec.lock)
#else
extern int grsec_enable_tpe;
extern int grsec_tpe_gid;
extern int grsec_enable_tpe_all;
extern int grsec_lock;
#endif

extern spinlock_t grsec_alert_lock;
extern unsigned long grsec_alert_wtime;
extern unsigned long grsec_alert_fyet;

extern spinlock_t grsec_audit_lock;

#define gr_task_fullpath(tsk) ("")

#define gr_parent_task_fullpath(tsk) ("")

#define DEFAULTSECARGS(task) gr_task_fullpath(task), task->comm, \
		       task->pid, task->uid, \
		       task->euid, task->gid, task->egid, \
		       gr_parent_task_fullpath(task), \
		       task->parent->comm, task->parent->pid, \
		       task->parent->uid, task->parent->euid, \
		       task->parent->gid, task->parent->egid

enum {
	GR_DO_AUDIT,
	GR_DONT_AUDIT,
	GR_DONT_AUDIT_GOOD
};

enum {
	GR_TTYSNIFF,
	GR_RBAC,
	GR_RBAC_STR,
	GR_STR_RBAC,
	GR_RBAC_MODE2,
	GR_RBAC_MODE3,
	GR_FILENAME,
	GR_NOARGS,
	GR_ONE_INT,
	GR_ONE_INT_TWO_STR,
	GR_ONE_STR,
	GR_STR_INT,
	GR_TWO_INT,
	GR_THREE_INT,
	GR_FIVE_INT_TWO_STR,
	GR_TWO_STR,
	GR_THREE_STR,
	GR_FOUR_STR,
	GR_STR_FILENAME,
	GR_FILENAME_STR,
	GR_FILENAME_TWO_INT,
	GR_FILENAME_TWO_INT_STR,
	GR_TEXTREL,
	GR_PTRACE,
	GR_RESOURCE,
	GR_CAP,
	GR_SIG,
	GR_CRASH1,
	GR_CRASH2,
	GR_PSACCT
};

#define gr_log_fs_generic(audit, msg, dentry, mnt) gr_log_varargs(audit, msg, GR_FILENAME, dentry, mnt)
#define gr_log_str(audit, msg, str) gr_log_varargs(audit, msg, GR_ONE_STR, str)

extern void gr_log_varargs(int audit, const char *msg, int argtypes, ...);

#endif
#endif
