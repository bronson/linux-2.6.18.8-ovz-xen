/*
 *
 *  kernel/cpt/cpt_inotify.c
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

int cpt_dump_inotify(cpt_object_t *obj, cpt_context_t *ctx)
{
	int err = 0;
	struct file *file = obj->o_obj;
	struct inotify_device *dev;
	struct inotify_watch *watch;
	struct inotify_kernel_event *kev;
	struct cpt_inotify_image ii;

	if (file->f_op != &inotify_fops) {
		eprintk_ctx("bad inotify file\n");
		return -EINVAL;
	}

	dev = file->private_data;

	/* inotify_user.c does not protect open /proc/N/fd, silly.
	 * Opener will get an invalid file with uninitialized private_data
	 */
	if (unlikely(dev == NULL)) {
		eprintk_ctx("bad inotify dev\n");
		return -EINVAL;
	}

	cpt_open_object(NULL, ctx);

	ii.cpt_next = CPT_NULL;
	ii.cpt_object = CPT_OBJ_INOTIFY;
	ii.cpt_hdrlen = sizeof(ii);
	ii.cpt_content = CPT_CONTENT_ARRAY;
	ii.cpt_file = obj->o_pos;
	ii.cpt_user = dev->user->uid;
	ii.cpt_max_events = dev->max_events;
	ii.cpt_last_wd = dev->ih->last_wd;

	ctx->write(&ii, sizeof(ii), ctx);

	mutex_lock(&dev->ih->mutex);
	list_for_each_entry(watch, &dev->ih->watches, h_list) {
		loff_t saved_obj;
		loff_t saved_obj2;
		struct cpt_inotify_wd_image wi;

		cpt_push_object(&saved_obj, ctx);
		cpt_open_object(NULL, ctx);

		wi.cpt_next = CPT_NULL;
		wi.cpt_object = CPT_OBJ_INOTIFY_WATCH;
		wi.cpt_hdrlen = sizeof(wi);
		wi.cpt_content = CPT_CONTENT_ARRAY;
		wi.cpt_wd = watch->wd;
		wi.cpt_mask = watch->mask;

		ctx->write(&wi, sizeof(wi), ctx);

		cpt_push_object(&saved_obj2, ctx);
		err = cpt_dump_dir(watch->dentry, watch->mnt, ctx);
		cpt_pop_object(&saved_obj2, ctx);
		if (err)
			break;

		cpt_close_object(ctx);
		cpt_pop_object(&saved_obj, ctx);
	}
	mutex_unlock(&dev->ih->mutex);

	if (err)
		return err;

	mutex_lock(&dev->ev_mutex);
	list_for_each_entry(kev, &dev->events, list) {
		loff_t saved_obj;
		struct cpt_inotify_ev_image ei;

		cpt_push_object(&saved_obj, ctx);
		cpt_open_object(NULL, ctx);

		ei.cpt_next = CPT_NULL;
		ei.cpt_object = CPT_OBJ_INOTIFY_EVENT;
		ei.cpt_hdrlen = sizeof(ei);
		ei.cpt_content = CPT_CONTENT_NAME;
		ei.cpt_wd = kev->event.wd;
		ei.cpt_mask = kev->event.mask;
		ei.cpt_cookie = kev->event.cookie;
		ei.cpt_namelen = kev->name ? strlen(kev->name) : 0;

		ctx->write(&ei, sizeof(ei), ctx);

		if (kev->name) {
			ctx->write(kev->name, ei.cpt_namelen+1, ctx);
			ctx->align(ctx);
		}

		cpt_close_object(ctx);
		cpt_pop_object(&saved_obj, ctx);
	}
	mutex_unlock(&dev->ev_mutex);

	cpt_close_object(ctx);

	return err;
}
