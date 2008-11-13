/*
 *
 *  kernel/cpt/rst_inotify.c
 *
 *  Copyright (C) 2000-2007  SWsoft
 *  All rights reserved.
 *
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/pipe_fs_i.h>
#include <linux/mman.h>
#include <linux/namespace.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/smp_lock.h>
#include <asm/uaccess.h>
#include <linux/vzcalluser.h>
#include <linux/inotify.h>
#include <linux/cpt_image.h>

#include "cpt_obj.h"
#include "cpt_context.h"
#include "cpt_mm.h"
#include "cpt_files.h"
#include "cpt_kernel.h"
#include "cpt_fsmagic.h"
#include "cpt_syscalls.h"

extern struct file_operations inotify_fops;

struct file *rst_open_inotify(struct cpt_file_image *fi,
			      unsigned flags,
			      struct cpt_context *ctx)
{
	struct file *file;
	int fd;

	fd = sys_inotify_init();
	if (fd < 0)
		return ERR_PTR(fd);

	file = fget(fd);
	sys_close(fd);
	return file;
}

static int restore_one_inotify(cpt_object_t *obj,
			       loff_t pos,
			       struct cpt_inotify_image *ibuf,
			       cpt_context_t *ctx)
{
	int err = 0;
	loff_t endpos;
	struct file *file = obj->o_obj;
	struct inotify_device *dev;

	if (file->f_op != &inotify_fops) {
		eprintk_ctx("bad inotify file\n");
		return -EINVAL;
	}

	dev = file->private_data;

	if (unlikely(dev == NULL)) {
		eprintk_ctx("bad inotify device\n");
		return -EINVAL;
	}

	endpos = pos + ibuf->cpt_next;
	pos += ibuf->cpt_hdrlen;
	while (pos < endpos) {
		union {
			struct cpt_inotify_wd_image wi;
			struct cpt_inotify_ev_image ei;
		} u;

		err = rst_get_object(-1, pos, &u, ctx);
		if (err) {
			eprintk_ctx("rst_get_object: %d\n", err);
			return err;
		}
		if (u.wi.cpt_object == CPT_OBJ_INOTIFY_WATCH) {
			struct dentry *d;
			struct vfsmount *mnt;
			loff_t fpos = pos + u.wi.cpt_hdrlen;

			err = cpt_get_dentry(&d, &mnt, &fpos, ctx);
			if (err) {
				eprintk_ctx("cpt_get_dentry: %d\n", err);
				return err;
			}

			mutex_lock(&dev->up_mutex);
			dev->ih->last_wd = u.wi.cpt_wd - 1;
			err = inotify_create_watch(dev, d, mnt, u.wi.cpt_mask);
			dev->ih->last_wd = ibuf->cpt_last_wd;
			if (err != u.wi.cpt_wd) {
				eprintk_ctx("wrong inotify descriptor %u %u\n", err, u.wi.cpt_wd);
				if (err >= 0)
					err = -EINVAL;
			} else
				err = 0;
			mutex_unlock(&dev->up_mutex);
			dput(d);
			mntput(mnt);
			if (err)
				break;
		} else if (u.wi.cpt_object == CPT_OBJ_INOTIFY_EVENT) {
			struct inotify_user_watch dummy_watch;
			struct inotify_watch *w;
			char *name = NULL;

			if (u.ei.cpt_namelen) {
				name = kmalloc(u.ei.cpt_namelen+1, GFP_KERNEL);
				if (name == NULL) {
					err = -ENOMEM;
					break;
				}
				name[u.ei.cpt_namelen] = 0;
				err = ctx->pread(name, u.ei.cpt_namelen, ctx, pos + u.ei.cpt_hdrlen);
				if (err) {
					kfree(name);
					break;
				}
			}

			w = &dummy_watch.wdata;
			dummy_watch.dev = dev;
			atomic_set(&w->count, 2);

			/* Trick to avoid destruction due to exit event */
			if (u.ei.cpt_mask & (IN_IGNORED | IN_ONESHOT))
				atomic_inc(&w->count);
			dev->ih->in_ops->handle_event(w, u.ei.cpt_wd, u.ei.cpt_mask,
						      u.ei.cpt_cookie, name, NULL);
			if (name)
				kfree(name);
		} else {
			eprintk_ctx("bad object: %u\n", u.wi.cpt_object);
			err = -EINVAL;
			break;
		}
		pos += u.wi.cpt_next;
	}
	return err;
}

int rst_inotify(cpt_context_t *ctx)
{
	int err;
	loff_t sec = ctx->sections[CPT_SECT_INOTIFY];
	loff_t endsec;
	struct cpt_section_hdr h;

	if (sec == CPT_NULL)
		return 0;

	err = ctx->pread(&h, sizeof(h), ctx, sec);
	if (err)
		return err;
	if (h.cpt_section != CPT_SECT_INOTIFY || h.cpt_hdrlen < sizeof(h))
		return -EINVAL;

	endsec = sec + h.cpt_next;
	sec += h.cpt_hdrlen;
	while (sec < endsec) {
		cpt_object_t *obj;
		struct cpt_inotify_image ibuf;

		err = rst_get_object(CPT_OBJ_INOTIFY, sec, &ibuf, ctx);
		if (err)
			return err;
		obj = lookup_cpt_obj_bypos(CPT_OBJ_FILE, ibuf.cpt_file, ctx);
		if (obj == NULL) {
			eprintk_ctx("cannot find inotify file object\n");
			return -EINVAL;
		}
		err = restore_one_inotify(obj, sec, &ibuf, ctx);
		if (err)
			return err;
		sec += ibuf.cpt_next;
	}

	return 0;
	
}
