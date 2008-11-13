#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>

void
gr_copy_label(struct task_struct *tsk)
{
	return;
}

int
gr_acl_handle_mmap(const struct file *file, const unsigned long prot,
		   unsigned int *vm_flags)
{
	return 1;
}

void
grsecurity_init(void)
{
	return;
}

void
gr_acl_handle_exit(void)
{
	return;
}

int
gr_acl_handle_mprotect(const struct file *file, const unsigned long prot)
{
	return 1;
}

void grsecurity_setup(void)
{
}
EXPORT_SYMBOL(grsecurity_setup);
