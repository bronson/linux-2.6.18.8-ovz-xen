/*
 *  linux/fs/filesystems.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  table of configured filesystems
 */

#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kmod.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>	/* for 'current' */
#include <linux/mount.h>
#include <asm/uaccess.h>

/*
 * Handling of filesystem drivers list.
 * Rules:
 *	Inclusion to/removals from/scanning of list are protected by spinlock.
 *	During the unload module must call unregister_filesystem().
 *	We can access the fields of list element if:
 *		1) spinlock is held or
 *		2) we hold the reference to the element.
 *	The latter can be guaranteed by call of try_filesystem(); if it
 *	returned 0 we must skip the element, otherwise we got the reference.
 *	Once the reference is obtained we can drop the spinlock.
 */

static struct file_system_type *file_systems;
static DEFINE_RWLOCK(file_systems_lock);

int try_get_filesystem(struct file_system_type *fs)
{
	if (try_module_get(fs->owner)) {
		get_ve(fs->owner_env);
		return 1;
	}
	return 0;
}

/* WARNING: This can be used only if we _already_ own a reference */
void get_filesystem(struct file_system_type *fs)
{
	get_ve(fs->owner_env);
	__module_get(fs->owner);
}

void put_filesystem(struct file_system_type *fs)
{
	module_put(fs->owner);
	put_ve(fs->owner_env);
}

static inline int check_ve_fstype(struct file_system_type *p,
		struct ve_struct *env)
{
	return ((p->fs_flags & FS_VIRTUALIZED) ||
			ve_accessible_strict(p->owner_env, env));
}

static struct file_system_type **find_filesystem(const char *name,
		struct ve_struct *env)
{
	struct file_system_type **p;
	for (p=&file_systems; *p; p=&(*p)->next) {
		if (!check_ve_fstype(*p, env))
			continue;
		if (strcmp((*p)->name,name) == 0)
			break;
	}
	return p;
}

/**
 *	register_filesystem - register a new filesystem
 *	@fs: the file system structure
 *
 *	Adds the file system passed to the list of file systems the kernel
 *	is aware of for mount and other syscalls. Returns 0 on success,
 *	or a negative errno code on an error.
 *
 *	The &struct file_system_type that is passed is linked into the kernel 
 *	structures and must not be freed until the file system has been
 *	unregistered.
 */
 
int register_filesystem(struct file_system_type * fs)
{
	int res = 0;
	struct file_system_type ** p;

	if (!fs)
		return -EINVAL;
	if (fs->next)
		return -EBUSY;
	INIT_LIST_HEAD(&fs->fs_supers);
	if (fs->owner_env == NULL)
		fs->owner_env = get_ve0();
	if (fs->proto == NULL)
		fs->proto = fs;
	write_lock(&file_systems_lock);
	p = find_filesystem(fs->name, fs->owner_env);
	if (*p)
		res = -EBUSY;
	else
		*p = fs;
	write_unlock(&file_systems_lock);
	return res;
}

EXPORT_SYMBOL(register_filesystem);

/**
 *	unregister_filesystem - unregister a file system
 *	@fs: filesystem to unregister
 *
 *	Remove a file system that was previously successfully registered
 *	with the kernel. An error is returned if the file system is not found.
 *	Zero is returned on a success.
 *	
 *	Once this function has returned the &struct file_system_type structure
 *	may be freed or reused.
 */
 
int unregister_filesystem(struct file_system_type * fs)
{
	struct file_system_type ** tmp;

	write_lock(&file_systems_lock);
	tmp = &file_systems;
	while (*tmp) {
		if (fs == *tmp) {
			*tmp = fs->next;
			fs->next = NULL;
			write_unlock(&file_systems_lock);
			return 0;
		}
		tmp = &(*tmp)->next;
	}
	write_unlock(&file_systems_lock);
	return -EINVAL;
}

EXPORT_SYMBOL(unregister_filesystem);

#ifdef CONFIG_VE
int register_ve_fs_type(struct ve_struct *ve, struct file_system_type *template,
		struct file_system_type **p_fs_type, struct vfsmount **p_mnt)
{
	struct vfsmount *mnt;
	struct file_system_type *local_fs_type;
	int ret;

	local_fs_type = kzalloc(sizeof(*local_fs_type) + sizeof(void *),
					GFP_KERNEL);
	if (local_fs_type == NULL)
		return -ENOMEM;

	local_fs_type->name = template->name;
	local_fs_type->fs_flags = template->fs_flags;
	local_fs_type->get_sb = template->get_sb;
	local_fs_type->kill_sb = template->kill_sb;
	local_fs_type->owner = template->owner;
	local_fs_type->owner_env = ve;
	local_fs_type->proto = template;

	get_filesystem(local_fs_type);	/* get_ve() inside */

	ret = register_filesystem(local_fs_type);
	if (ret)
		goto reg_err;

	if (p_mnt == NULL) 
		goto done; 

	mnt = kern_mount(local_fs_type);
	if (IS_ERR(mnt))
		goto mnt_err;

	*p_mnt = mnt;
done:
	*p_fs_type = local_fs_type;
	return 0;

mnt_err:
	ret = PTR_ERR(mnt);
	unregister_filesystem(local_fs_type); /* does not put */

reg_err:
	put_filesystem(local_fs_type);
	kfree(local_fs_type);
	printk(KERN_DEBUG
	       "register_ve_fs_type(\"%s\") err=%d\n", template->name, ret);
	return ret;
}

EXPORT_SYMBOL(register_ve_fs_type);

void unregister_ve_fs_type(struct file_system_type *local_fs_type,
		struct vfsmount *local_fs_mount)
{
	if (local_fs_mount == NULL && local_fs_type == NULL)
		return;

	unregister_filesystem(local_fs_type);
	umount_ve_fs_type(local_fs_type);
	if (local_fs_mount)
		kern_umount(local_fs_mount); /* alias to mntput, drop our ref */
	put_filesystem(local_fs_type);
}

EXPORT_SYMBOL(unregister_ve_fs_type);
#endif

static int fs_index(const char __user * __name)
{
	struct file_system_type * tmp;
	char * name;
	int err, index;

	name = getname(__name);
	err = PTR_ERR(name);
	if (IS_ERR(name))
		return err;

	err = -EINVAL;
	read_lock(&file_systems_lock);
	for (tmp=file_systems, index=0 ; tmp ; tmp=tmp->next) {
		if (!check_ve_fstype(tmp, get_exec_env()))
			continue;
		if (strcmp(tmp->name,name) == 0) {
			err = index;
			break;
		}
		index++;
	}
	read_unlock(&file_systems_lock);
	putname(name);
	return err;
}

static int fs_name(unsigned int index, char __user * buf)
{
	struct file_system_type * tmp;
	int len, res;

	read_lock(&file_systems_lock);
	for (tmp = file_systems; tmp; tmp = tmp->next) {
		if (!check_ve_fstype(tmp, get_exec_env()))
			continue;
		if (!index) {
			if (try_get_filesystem(tmp))
				break;
		} else
			index--;
	}
	read_unlock(&file_systems_lock);
	if (!tmp)
		return -EINVAL;

	/* OK, we got the reference, so we can safely block */
	len = strlen(tmp->name) + 1;
	res = copy_to_user(buf, tmp->name, len) ? -EFAULT : 0;
	put_filesystem(tmp);
	return res;
}

static int fs_maxindex(void)
{
	struct file_system_type * tmp;
	int index;

	read_lock(&file_systems_lock);
	for (tmp = file_systems, index = 0 ; tmp ; tmp = tmp->next)
		if (check_ve_fstype(tmp, get_exec_env()))
			index++;
	read_unlock(&file_systems_lock);
	return index;
}

/*
 * Whee.. Weird sysv syscall. 
 */
asmlinkage long sys_sysfs(int option, unsigned long arg1, unsigned long arg2)
{
	int retval = -EINVAL;

	switch (option) {
		case 1:
			retval = fs_index((const char __user *) arg1);
			break;

		case 2:
			retval = fs_name(arg1, (char __user *) arg2);
			break;

		case 3:
			retval = fs_maxindex();
			break;
	}
	return retval;
}

int get_filesystem_list(char * buf)
{
	int len = 0;
	struct file_system_type * tmp;

	read_lock(&file_systems_lock);
	tmp = file_systems;
	while (tmp && len < PAGE_SIZE - 80) {
		if (check_ve_fstype(tmp, get_exec_env()))
			len += sprintf(buf+len, "%s\t%s\n",
				(tmp->fs_flags & FS_REQUIRES_DEV) ? "" : "nodev",
				tmp->name);
		tmp = tmp->next;
	}
	read_unlock(&file_systems_lock);
	return len;
}

struct file_system_type *get_fs_type(const char *name)
{
	struct file_system_type *fs;

	read_lock(&file_systems_lock);
	fs = *(find_filesystem(name, get_exec_env()));
	if (fs && !try_get_filesystem(fs))
		fs = NULL;
	read_unlock(&file_systems_lock);
	if (!fs && (request_module("%s", name) == 0)) {
		read_lock(&file_systems_lock);
		fs = *(find_filesystem(name, get_exec_env()));
		if (fs && !try_get_filesystem(fs))
			fs = NULL;
		read_unlock(&file_systems_lock);
	}
	return fs;
}

EXPORT_SYMBOL(get_fs_type);
EXPORT_SYMBOL(get_filesystem);
EXPORT_SYMBOL(put_filesystem);
