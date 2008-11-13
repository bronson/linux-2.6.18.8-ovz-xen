#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/tty.h>
#include <linux/fs.h>
#include <linux/grinternal.h>

#define BEGIN_LOCKS(x) \
	if (x != GR_DO_AUDIT) \
		spin_lock(&grsec_alert_lock); \
	else \
		spin_lock(&grsec_audit_lock)

#define END_LOCKS(x) \
	if (x != GR_DO_AUDIT) \
		spin_unlock(&grsec_alert_lock); \
	else \
		spin_unlock(&grsec_audit_lock);

enum {
	FLOODING,
	NO_FLOODING
};

extern char *gr_alert_log_fmt;
extern char *gr_audit_log_fmt;
extern char *gr_alert_log_buf;
extern char *gr_audit_log_buf;

static int gr_log_start(int audit)
{
	char *loglevel = (audit == GR_DO_AUDIT) ? KERN_INFO : KERN_ALERT;
	char *fmt = (audit == GR_DO_AUDIT) ? gr_audit_log_fmt : gr_alert_log_fmt;
	char *buf = (audit == GR_DO_AUDIT) ? gr_audit_log_buf : gr_alert_log_buf;

	if (audit == GR_DO_AUDIT)
		goto set_fmt;

	if (!grsec_alert_wtime || jiffies - grsec_alert_wtime > CONFIG_GRKERNSEC_FLOODTIME * HZ) {
		grsec_alert_wtime = jiffies;
		grsec_alert_fyet = 0;
	} else if ((jiffies - grsec_alert_wtime < CONFIG_GRKERNSEC_FLOODTIME * HZ) && (grsec_alert_fyet < CONFIG_GRKERNSEC_FLOODBURST)) {
		grsec_alert_fyet++;
	} else if (grsec_alert_fyet == CONFIG_GRKERNSEC_FLOODBURST) {
		grsec_alert_wtime = jiffies;
		grsec_alert_fyet++;
		ve_printk(VE_LOG, KERN_ALERT "grsec: more alerts, logging disabled for %d seconds\n", CONFIG_GRKERNSEC_FLOODTIME);
		return FLOODING;
	} else return FLOODING;

set_fmt:
	memset(buf, 0, PAGE_SIZE);
	sprintf(fmt, "%s%s", loglevel, "grsec: ");
	strcpy(buf, fmt);

	return NO_FLOODING;
}

static void gr_log_middle(int audit, const char *msg, va_list ap)
{
	char *buf = (audit == GR_DO_AUDIT) ? gr_audit_log_buf : gr_alert_log_buf;
	unsigned int len = strlen(buf);

	vsnprintf(buf + len, PAGE_SIZE - len - 1, msg, ap);

	return;
}

static void gr_log_middle_varargs(int audit, const char *msg, ...)
{
	char *buf = (audit == GR_DO_AUDIT) ? gr_audit_log_buf : gr_alert_log_buf;
	unsigned int len = strlen(buf);
	va_list ap;

	va_start(ap, msg);
	vsnprintf(buf + len, PAGE_SIZE - len - 1, msg, ap);
	va_end(ap);

	return;
}

static void gr_log_end(int audit)
{
	char *buf = (audit == GR_DO_AUDIT) ? gr_audit_log_buf : gr_alert_log_buf;
	unsigned int len = strlen(buf);

	snprintf(buf + len, PAGE_SIZE - len - 1, DEFAULTSECMSG, DEFAULTSECARGS(current));
	ve_printk(VE_LOG, "%s\n", buf);

	return;
}

void gr_log_varargs(int audit, const char *msg, int argtypes, ...)
{
	int logtype;
	struct dentry *dentry;
	struct vfsmount *mnt;
	va_list ap;

	BEGIN_LOCKS(audit);
	logtype = gr_log_start(audit);
	if (logtype == FLOODING) {
		END_LOCKS(audit);
		return;
	}
	va_start(ap, argtypes);
	switch (argtypes) {
	/* 
	 * Only GR_FILENAME is now supported in VZ
	 */
	case GR_FILENAME:
		dentry = va_arg(ap, struct dentry *);
		mnt = va_arg(ap, struct vfsmount *);
		gr_log_middle_varargs(audit, msg, gr_to_filename(dentry, mnt));
		break;
	default:
		gr_log_middle(audit, msg, ap);
	}
	va_end(ap);
	gr_log_end(audit);
	END_LOCKS(audit);
}
