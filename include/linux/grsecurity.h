#ifndef GR_SECURITY_H
#define GR_SECURITY_H
#include <linux/fs.h>

extern int gr_tpe_allow(const struct file *file);
extern void gr_copy_label(struct task_struct *tsk);
extern int gr_acl_handle_mmap(const struct file *file,
			      const unsigned long prot);
extern int gr_acl_handle_mprotect(const struct file *file,
				  const unsigned long prot);
extern void gr_acl_handle_exit(void);

#endif
