#include <linux/compiler.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/linkage.h>
#include <linux/namei.h>
#include <linux/sched.h>
#include <linux/stat.h>
#include <linux/utime.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>


asmlinkage long sys_utimensat(int dfd, char __user *filename, struct timespec __user *utimes, int flags)
{
	struct timespec tstimes[2];
	struct timeval time[2];
	if (utimes) {
		if (copy_from_user(&tstimes, utimes, sizeof(tstimes)))
			return -EFAULT;
		if ((tstimes[0].tv_nsec == UTIME_OMIT ||
		     tstimes[0].tv_nsec == UTIME_NOW) &&
		     tstimes[0].tv_sec != 0)
			return -EINVAL;
		if ((tstimes[1].tv_nsec == UTIME_OMIT ||
		     tstimes[1].tv_nsec == UTIME_NOW) &&
		     tstimes[1].tv_sec != 0)
			return -EINVAL;

	/* Nothing to do, we must not even check the path. */
	if (tstimes[0].tv_nsec == UTIME_OMIT &&
	    tstimes[1].tv_nsec == UTIME_OMIT)
		return 0;
	}

/* Note: declaration of lutimes from glibc is:
int lutimes(const char *path, const struct timeval *times);
while 2.6.23 had timespec instead of timeval, but sizeof(timespec)==sizeof(timeval) */
	time[0].tv_sec =tstimes[0].tv_sec;
	time[0].tv_usec=tstimes[0].tv_nsec/1000;
	time[1].tv_sec =tstimes[1].tv_sec;
	time[1].tv_usec=tstimes[1].tv_nsec/1000;
	return do_utimes(dfd, filename, utimes ? time : NULL, flags);
}
