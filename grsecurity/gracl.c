#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/tty.h>
#include <linux/proc_fs.h>
#include <linux/smp_lock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/types.h>
#include <linux/capability.h>
#include <linux/sysctl.h>
#include <linux/netdevice.h>
#include <linux/ptrace.h>
#include <linux/grsecurity.h>
#include <linux/grinternal.h>
#include <linux/percpu.h>

#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/mman.h>

extern char *gr_shared_page[4];
	
static char *
gen_full_path(struct dentry *dentry, struct vfsmount *vfsmnt,
              struct dentry *root, struct vfsmount *rootmnt, char *buf, int buflen)
{
	char *end = buf + buflen;
	char *retval;
	int namelen = 0;

	*--end = '\0';

	retval = end - 1;
	*retval = '/';

	if (dentry == root && vfsmnt == rootmnt)
		return retval;
	if (dentry != vfsmnt->mnt_root && !IS_ROOT(dentry)) {
		namelen = strlen(dentry->d_name.name);
		buflen -= namelen;
		if (buflen < 2)
			goto err;
		if (dentry->d_parent != root || vfsmnt != rootmnt)
			buflen--;
	}

	retval = __d_path(dentry->d_parent, vfsmnt, root, rootmnt, buf, buflen);
	if (unlikely(IS_ERR(retval)))
err:
		retval = strcpy(buf, "<path too long>");
	else if (namelen != 0) {
		end = buf + buflen - 1; // accounts for null termination
		if (dentry->d_parent != root || vfsmnt != rootmnt)
			*end++ = '/'; // accounted for above with buflen--
		memcpy(end, dentry->d_name.name, namelen);
	}

	return retval;
}

static char *
d_real_path(const struct dentry *dentry, const struct vfsmount *vfsmnt,
	    char *buf, int buflen)
{
	char *res;
	struct dentry *root;
	struct vfsmount *rootmnt;

	/* we can't use real_root, real_root_mnt, because they belong only to the RBAC system */
#ifdef CONFIG_VE
	/* Don't use child_reaper, because it's VE0 process */ 
	root = dget(get_exec_env()->fs_root);
	rootmnt = mntget(get_exec_env()->fs_rootmnt);
#else
	read_lock(&child_reaper->fs->lock);
	root = dget(child_reaper->fs->root);
	rootmnt = mntget(child_reaper->fs->rootmnt);
	read_unlock(&child_reaper->fs->lock);
#endif

	spin_lock(&dcache_lock);
	res = gen_full_path((struct dentry *)dentry, (struct vfsmount *)vfsmnt, root, rootmnt, buf, buflen);
	spin_unlock(&dcache_lock);

	dput(root);
	mntput(rootmnt);
	return res;
}

char *
gr_to_filename(const struct dentry *dentry, const struct vfsmount *mnt)
{
	return d_real_path(dentry, mnt, per_cpu_ptr(gr_shared_page[0], smp_processor_id()),
			   PAGE_SIZE);
}

char *
gr_to_filename2(const struct dentry *dentry, const struct vfsmount *mnt)
{
	return d_real_path(dentry, mnt, per_cpu_ptr(gr_shared_page[2], smp_processor_id()),
			   PAGE_SIZE);
}

char *
gr_to_filename3(const struct dentry *dentry, const struct vfsmount *mnt)
{
	return d_real_path(dentry, mnt, per_cpu_ptr(gr_shared_page[3], smp_processor_id()),
			   PAGE_SIZE);
}

int
gr_acl_handle_mmap(const struct file *file, const unsigned long prot)
{
	if (unlikely(!file || !(prot & PROT_EXEC)))
		return 1;

	if (!gr_tpe_allow(file))
		return 0;
	return 1;
}

int
gr_acl_handle_mprotect(const struct file *file, const unsigned long prot)
{
	if (unlikely(!file || !(prot & PROT_EXEC)))
		return 1;

	if (!gr_tpe_allow(file))
		return 0;
	return 1;
}
