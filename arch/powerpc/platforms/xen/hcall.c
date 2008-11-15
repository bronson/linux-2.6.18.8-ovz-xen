/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * Copyright (C) IBM Corp. 2006, 2007
 *
 * Authors: Hollis Blanchard <hollisb@us.ibm.com>
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/module.h>
#include <xen/interface/xen.h>
#include <xen/interface/domctl.h>
#include <xen/interface/sysctl.h>
#include <xen/interface/platform.h>
#include <xen/interface/memory.h>
#include <xen/interface/xencomm.h>
#include <xen/interface/version.h>
#include <xen/interface/sched.h>
#include <xen/interface/event_channel.h>
#include <xen/interface/physdev.h>
#include <xen/interface/vcpu.h>
#include <xen/interface/xsm/acm_ops.h>
#include <xen/interface/kexec.h>
#include <xen/public/privcmd.h>
#include <asm/hypercall.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/hvcall.h>
#include "setup.h"

/* Xencomm notes:
 *
 * For kernel memory, we assume that virtually contiguous pages are also
 * physically contiguous. This allows us to avoid creating descriptors for
 * kernel hypercalls, such as console and event channel operations.
 *
 * In general, we need a xencomm descriptor to cover the top-level data
 * structure (e.g. the domctl op), plus another for every embedded pointer to
 * another data structure (i.e. for every GUEST_HANDLE).
 */

int HYPERVISOR_console_io(int cmd, int count, char *str)
{
	struct xencomm_handle *desc;
	int rc;

	desc = xencomm_map_no_alloc(str, count);
	if (desc == NULL)
		return -EINVAL;

	rc = plpar_hcall_norets(XEN_MARK(__HYPERVISOR_console_io),
				  cmd, count, desc);

	xencomm_free(desc);

	return rc;
}
EXPORT_SYMBOL(HYPERVISOR_console_io);

int HYPERVISOR_event_channel_op(int cmd, void *op)
{
	int rc;

	struct xencomm_handle *desc =
		xencomm_map_no_alloc(op, sizeof(evtchn_op_t));
	if (desc == NULL)
		return -EINVAL;

	rc = plpar_hcall_norets(XEN_MARK(__HYPERVISOR_event_channel_op),
				cmd, desc);

	xencomm_free(desc);

	return rc;

}
EXPORT_SYMBOL(HYPERVISOR_event_channel_op);

int HYPERVISOR_xen_version(int cmd, void *arg)
{
	struct xencomm_handle *desc;
	const unsigned long hcall = __HYPERVISOR_xen_version;
	int argsize;
	int rc;

	switch (cmd) {
	case XENVER_version:
		/* do not actually pass an argument */
		return plpar_hcall_norets(XEN_MARK(hcall), cmd, 0);
	case XENVER_extraversion:
		argsize = sizeof(xen_extraversion_t);
		break;
	case XENVER_compile_info:
		argsize = sizeof(xen_compile_info_t);
		break;
	case XENVER_capabilities:
		argsize = sizeof(xen_capabilities_info_t);
		break;
	case XENVER_changeset:
		argsize = sizeof(xen_changeset_info_t);
		break;
	case XENVER_platform_parameters:
		argsize = sizeof(xen_platform_parameters_t);
		break;
	case XENVER_pagesize:
		if (arg == NULL)
			argsize = 0;
		else
			argsize = sizeof(void *);
		break;
	case XENVER_get_features:
		argsize = sizeof(xen_feature_info_t);
		break;
	default:
		printk(KERN_ERR "%s: unknown version cmd %d\n", __func__, cmd);
		return -ENOSYS;
	}

	/* desc could be NULL in the case of XENVER_pagesize with NULL arg */
	desc = xencomm_map(arg, argsize);

	rc = plpar_hcall_norets(XEN_MARK(hcall), cmd, desc);

	xencomm_free(desc);

	return rc;
}
EXPORT_SYMBOL(HYPERVISOR_xen_version);


int HYPERVISOR_physdev_op(int cmd, void *op)
{
	struct xencomm_handle *desc =
		xencomm_map_no_alloc(op, sizeof(physdev_op_t));
	int rc;

	if (desc == NULL)
		return -EINVAL;

	rc = plpar_hcall_norets(XEN_MARK(__HYPERVISOR_physdev_op),
				cmd, desc);

	xencomm_free(desc);

	return rc;
}
EXPORT_SYMBOL(HYPERVISOR_physdev_op);

int HYPERVISOR_sched_op(int cmd, void *arg)
{
	int argsize = 0;
	int rc = -EINVAL;
	struct xencomm_handle *desc;
	struct xencomm_handle *ports = NULL;

	switch (cmd) {
	case SCHEDOP_yield:
	case SCHEDOP_block:
		return plpar_hcall_norets(XEN_MARK(__HYPERVISOR_sched_op),
					  cmd, 0);
		break;

	case SCHEDOP_poll: {
		struct sched_poll sched_poll;

		argsize = sizeof(struct sched_poll);

		memcpy(&sched_poll, arg, sizeof(sched_poll));

		ports = xencomm_map(
				xen_guest_handle(sched_poll.ports),
				(sizeof(evtchn_port_t) * sched_poll.nr_ports));

		if (ports == NULL)
			return -ENOMEM;

		set_xen_guest_handle(sched_poll.ports, (evtchn_port_t *)ports);
		memcpy(arg, &sched_poll, sizeof(sched_poll));

		}
		break;
	case SCHEDOP_shutdown:
		argsize = sizeof(struct sched_shutdown);
		break;
	case SCHEDOP_remote_shutdown:
		argsize = sizeof(struct sched_remote_shutdown);
		break;
	default:
		printk(KERN_ERR "%s: unknown sched op %d\n", __func__, cmd);
		return -ENOSYS;
	}

	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc) {
		rc = plpar_hcall_norets(XEN_MARK(__HYPERVISOR_sched_op),
					cmd, desc);
		xencomm_free(desc);
	}

	xencomm_free(ports);

	return rc;
}
EXPORT_SYMBOL(HYPERVISOR_sched_op);

int HYPERVISOR_suspend(unsigned long srec)
{
	int cmd = SCHEDOP_shutdown;
	struct sched_shutdown sched_shutdown = {
		.reason = SHUTDOWN_suspend,
	};
	struct xencomm_handle *desc;

	desc = xencomm_map_no_alloc(&sched_shutdown, sizeof(struct sched_shutdown));

	return plpar_hcall_norets(XEN_MARK(__HYPERVISOR_sched_op),
				  cmd, desc, srec);
}
EXPORT_SYMBOL(HYPERVISOR_suspend);

int HYPERVISOR_kexec_op(unsigned long op, void *args)
{
	unsigned long argsize;
	struct xencomm_handle *desc;

	switch (op) {
		case KEXEC_CMD_kexec_get_range:
			argsize = sizeof(struct xen_kexec_range);
			break;
		case KEXEC_CMD_kexec_load:
			argsize = sizeof(struct xen_kexec_load);
			break;
		case KEXEC_CMD_kexec_unload:
			argsize = sizeof(struct xen_kexec_load);
			break;
		case KEXEC_CMD_kexec:
			argsize = sizeof(struct xen_kexec_exec);
			break;
		default:
			return -ENOSYS;
	}
	desc = xencomm_map_no_alloc(args, argsize);

	return plpar_hcall_norets(XEN_MARK(__HYPERVISOR_kexec_op),
				  op, desc);
}
EXPORT_SYMBOL(HYPERVISOR_kexec_op);

int HYPERVISOR_poll(
	evtchn_port_t *ports, unsigned int nr_ports, u64 timeout)
{
	struct sched_poll sched_poll = {
		.nr_ports = nr_ports,
		.timeout = jiffies_to_ns(timeout)
	};
	set_xen_guest_handle(sched_poll.ports, ports);

	return HYPERVISOR_sched_op(SCHEDOP_poll, &sched_poll);
}
EXPORT_SYMBOL(HYPERVISOR_poll);

typedef ulong (mf_t)(ulong arg0, ...);

static mf_t *multicall_funcs[] = {
	[__HYPERVISOR_grant_table_op] = (mf_t *)HYPERVISOR_grant_table_op,
};

int HYPERVISOR_multicall(void *call_list, int nr_calls)
{
	/* we blow out the multicall because the xencomm stuff is jsut
	 * too tricky */
	multicall_entry_t *mcl = (multicall_entry_t *)call_list;
	multicall_entry_t *c;
	int i;
	mf_t *mf;
	int res;
	ulong flags;

	/* let make sure all the calls are supported */
	for (i = 0; i < nr_calls; i++) {
		mf = multicall_funcs[mcl[i].op];
		BUG_ON(mf == NULL);
	}
	/* disable interrupts until we are done all calls */
	local_irq_save(flags);
	for (i = 0; i < nr_calls; i++) {
		/* lookup supported multicalls */
		c = &mcl[i];
		mf = multicall_funcs[c->op];
		res = mf(c->args[0], c->args[1], c->args[2],
			 c->args[3], c->args[4], c->args[5]);
		c->result = res;
	}
	local_irq_restore(flags);
	return 0;
}
EXPORT_SYMBOL(HYPERVISOR_multicall);


/* privcmd operations: */

static int xenppc_privcmd_domctl(privcmd_hypercall_t *hypercall)
{
	xen_domctl_t kern_op;
	xen_domctl_t __user *user_op = (xen_domctl_t __user *)hypercall->arg[0];
	struct xencomm_handle *op_desc;
	struct xencomm_handle *desc = NULL;
	int ret = 0;

	if (copy_from_user(&kern_op, user_op, sizeof(xen_domctl_t)))
		return -EFAULT;

	if (kern_op.interface_version != XEN_DOMCTL_INTERFACE_VERSION) {
		printk(KERN_WARNING "%s: %s %x != %x\n", __func__, current->comm,
				kern_op.interface_version, XEN_DOMCTL_INTERFACE_VERSION);
		return -EACCES;
	}

	op_desc = xencomm_map(&kern_op, sizeof(xen_domctl_t));
	if (op_desc == NULL)
		return -ENOMEM;

	switch (kern_op.cmd) {
	case XEN_DOMCTL_createdomain:
	case XEN_DOMCTL_destroydomain:
	case XEN_DOMCTL_pausedomain:
	case XEN_DOMCTL_unpausedomain:
	case XEN_DOMCTL_getdomaininfo:
		break;
	case XEN_DOMCTL_getmemlist:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.getmemlist.buffer),
			kern_op.u.getmemlist.max_pfns * sizeof(unsigned long));

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.getmemlist.buffer,
				     (void *)desc);
		break;
	case XEN_DOMCTL_getpageframeinfo:
		break;
	case XEN_DOMCTL_getpageframeinfo2:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.getpageframeinfo2.array),
			kern_op.u.getpageframeinfo2.num);

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.getpageframeinfo2.array,
				     (void *)desc);
		break;
	case XEN_DOMCTL_shadow_op:

		if (xen_guest_handle(kern_op.u.shadow_op.dirty_bitmap))
		{
			desc = xencomm_map(
				xen_guest_handle(kern_op.u.shadow_op.dirty_bitmap),
				kern_op.u.shadow_op.pages * sizeof(unsigned long));

			if (desc == NULL)
				ret = -ENOMEM;

			set_xen_guest_handle(kern_op.u.shadow_op.dirty_bitmap,
				    	 (void *)desc);
		}
		break;
	case XEN_DOMCTL_max_mem:
		break;
	case XEN_DOMCTL_setvcpucontext:
	case XEN_DOMCTL_getvcpucontext:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.vcpucontext.ctxt),
			sizeof(vcpu_guest_context_t));

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.vcpucontext.ctxt,
				     (void *)desc);
		break;
	case XEN_DOMCTL_getvcpuinfo:
		break;
	case XEN_DOMCTL_setvcpuaffinity:
	case XEN_DOMCTL_getvcpuaffinity:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.vcpuaffinity.cpumap.bitmap),
			(kern_op.u.vcpuaffinity.cpumap.nr_cpus + 7) / 8);

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.vcpuaffinity.cpumap.bitmap,
				     (void *)desc);
		break;
	case XEN_DOMCTL_max_vcpus:
	case XEN_DOMCTL_scheduler_op:
	case XEN_DOMCTL_setdomainhandle:
	case XEN_DOMCTL_setdebugging:
	case XEN_DOMCTL_irq_permission:
	case XEN_DOMCTL_iomem_permission:
	case XEN_DOMCTL_ioport_permission:
	case XEN_DOMCTL_hypercall_init:
	case XEN_DOMCTL_arch_setup:
	case XEN_DOMCTL_settimeoffset:
	case XEN_DOMCTL_real_mode_area:
		break;
	default:
		printk(KERN_ERR "%s: unknown domctl cmd %d\n", __func__, kern_op.cmd);
		return -ENOSYS;
	}

	if (ret)
		goto out; /* error mapping the nested pointer */

	ret = plpar_hcall_norets(XEN_MARK(hypercall->op),op_desc);

	if (copy_to_user(user_op, &kern_op, sizeof(xen_domctl_t)))
		ret = -EFAULT;

out:
	xencomm_free(desc);
	xencomm_free(op_desc);
	return ret;
}

static int xenppc_privcmd_sysctl(privcmd_hypercall_t *hypercall)
{
	xen_sysctl_t kern_op;
	xen_sysctl_t __user *user_op = (xen_sysctl_t __user *)hypercall->arg[0];
	struct xencomm_handle *op_desc;
	struct xencomm_handle *desc = NULL;
	int ret = 0;

	if (copy_from_user(&kern_op, user_op, sizeof(xen_sysctl_t)))
		return -EFAULT;

	if (kern_op.interface_version != XEN_SYSCTL_INTERFACE_VERSION) {
		printk(KERN_WARNING "%s: %s %x != %x\n", __func__, current->comm,
				kern_op.interface_version, XEN_SYSCTL_INTERFACE_VERSION);
		return -EACCES;
	}

	op_desc = xencomm_map(&kern_op, sizeof(xen_sysctl_t));

	if (op_desc == NULL)
		return -ENOMEM;

	switch (kern_op.cmd) {
	case XEN_SYSCTL_readconsole:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.readconsole.buffer),
			kern_op.u.readconsole.count);

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.readconsole.buffer,
				     (void *)desc);
		break;
	case XEN_SYSCTL_tbuf_op:
	case XEN_SYSCTL_physinfo:
	case XEN_SYSCTL_sched_id:
		break;
	case XEN_SYSCTL_perfc_op:
		/* XXX this requires *two* embedded xencomm mappings (desc and val),
		 * and I don't feel like it right now. */
		printk(KERN_ERR "%s: unknown sysctl cmd %d\n", __func__, kern_op.cmd);
		return -ENOSYS;
	case XEN_SYSCTL_getdomaininfolist:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.getdomaininfolist.buffer),
			kern_op.u.getdomaininfolist.max_domains *
					sizeof(xen_domctl_getdomaininfo_t));

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.getdomaininfolist.buffer,
				     (void *)desc);
		break;
	default:
		printk(KERN_ERR "%s: unknown sysctl cmd %d\n", __func__, kern_op.cmd);
		return -ENOSYS;
	}

	if (ret)
		goto out; /* error mapping the nested pointer */

	ret = plpar_hcall_norets(XEN_MARK(hypercall->op), op_desc);

	if (copy_to_user(user_op, &kern_op, sizeof(xen_sysctl_t)))
		ret = -EFAULT;

out:
	xencomm_free(desc);
	xencomm_free(op_desc);
	return ret;
}

static int xenppc_privcmd_platform_op(privcmd_hypercall_t *hypercall)
{
	xen_platform_op_t kern_op;
	xen_platform_op_t __user *user_op =
			(xen_platform_op_t __user *)hypercall->arg[0];
	struct xencomm_handle *op_desc;
	struct xencomm_handle *desc = NULL;
	int ret = 0;

	if (copy_from_user(&kern_op, user_op, sizeof(xen_platform_op_t)))
		return -EFAULT;

	if (kern_op.interface_version != XENPF_INTERFACE_VERSION) {
		printk(KERN_WARNING "%s: %s %x != %x\n", __func__, current->comm,
				kern_op.interface_version, XENPF_INTERFACE_VERSION);
		return -EACCES;
	}

	op_desc = xencomm_map(&kern_op, sizeof(xen_platform_op_t));

	if (op_desc == NULL)
		return -ENOMEM;

	switch (kern_op.cmd) {
	case XENPF_settime:
	case XENPF_add_memtype:
	case XENPF_del_memtype:
	case XENPF_read_memtype:
	case XENPF_microcode_update:
	case XENPF_platform_quirk:
		break;
	default:
		printk(KERN_ERR "%s: unknown platform_op cmd %d\n", __func__,
				kern_op.cmd);
		return -ENOSYS;
	}

	if (ret)
		goto out; /* error mapping the nested pointer */

	ret = plpar_hcall_norets(XEN_MARK(hypercall->op), op_desc);

	if (copy_to_user(user_op, &kern_op, sizeof(xen_platform_op_t)))
		ret = -EFAULT;

out:
	xencomm_free(desc);
	xencomm_free(op_desc);
	return ret;
}

int HYPERVISOR_memory_op(unsigned int cmd, void *arg)
{
	int ret;
	struct xencomm_handle *op_desc;
	xen_memory_reservation_t *mop;


	mop = (xen_memory_reservation_t *)arg;

	op_desc = xencomm_map(mop, sizeof(xen_memory_reservation_t));

	if (op_desc == NULL)
		return -ENOMEM;

	switch (cmd) {
	case XENMEM_increase_reservation:
	case XENMEM_decrease_reservation:
	case XENMEM_populate_physmap: {
		struct xencomm_handle *desc = NULL;

		if (xen_guest_handle(mop->extent_start)) {
			desc = xencomm_map(
				xen_guest_handle(mop->extent_start),
				mop->nr_extents *
				sizeof(*xen_guest_handle(mop->extent_start)));

			if (desc == NULL) {
				ret = -ENOMEM;
				goto out;
			}

			set_xen_guest_handle(mop->extent_start,
					     (void *)desc);
		}

		ret = plpar_hcall_norets(XEN_MARK(__HYPERVISOR_memory_op),
					cmd, op_desc);

		xencomm_free(desc);
		}
		break;

	case XENMEM_maximum_ram_page:
		/* arg is NULL so we can call thru here */
		ret = plpar_hcall_norets(XEN_MARK(__HYPERVISOR_memory_op),
					cmd, NULL);
		break;
	default:
		printk(KERN_ERR "%s: unknown memory op %d\n", __func__, cmd);
		ret = -ENOSYS;
	}

out:
	xencomm_free(op_desc);
	return ret;
}
EXPORT_SYMBOL(HYPERVISOR_memory_op);

static int xenppc_privcmd_memory_op(privcmd_hypercall_t *hypercall)
{
	xen_memory_reservation_t kern_op;
	xen_memory_reservation_t __user *user_op;
	const unsigned long cmd = hypercall->arg[0];
	int ret = 0;

	user_op = (xen_memory_reservation_t __user *)hypercall->arg[1];
	if (copy_from_user(&kern_op, user_op,
			   sizeof(xen_memory_reservation_t)))
		return -EFAULT;

	ret = HYPERVISOR_memory_op(cmd, &kern_op);
	if (ret >= 0) {
		if (copy_to_user(user_op, &kern_op,
				 sizeof(xen_memory_reservation_t)))
			return -EFAULT;
	}
	return ret;
}

static int xenppc_privcmd_version(privcmd_hypercall_t *hypercall)
{
	return HYPERVISOR_xen_version(hypercall->arg[0],
			(void *)hypercall->arg[1]);
}

static int xenppc_privcmd_event_channel_op(privcmd_hypercall_t *hypercall)
{
	struct xencomm_handle *desc;
	unsigned int argsize;
	int ret;

	switch (hypercall->arg[0]) {
	case EVTCHNOP_alloc_unbound:
		argsize = sizeof(evtchn_alloc_unbound_t);
		break;

	case EVTCHNOP_status:
		argsize = sizeof(evtchn_status_t);
		break;

	default:
		printk(KERN_ERR "%s: unknown EVTCHNOP (%ld)\n",
		       __func__, hypercall->arg[0]);
		return -EINVAL;
	}

	desc = xencomm_map((void *)hypercall->arg[1], argsize);

	if (desc == NULL)
		return -ENOMEM;

	ret = plpar_hcall_norets(XEN_MARK(hypercall->op), hypercall->arg[0],
				desc);

	xencomm_free(desc);
	return ret;
}

static int xenppc_acmcmd_op(privcmd_hypercall_t *hypercall)
{
	xen_acmctl_t kern_op;
	xen_acmctl_t __user *user_op = (xen_acmctl_t __user *)hypercall->arg[0];
	void *op_desc;
	void *desc = NULL, *desc2 = NULL, *desc3 = NULL, *desc4 = NULL;
	int ret = 0;

	if (copy_from_user(&kern_op, user_op, sizeof(xen_acmctl_t)))
		return -EFAULT;

	if (kern_op.interface_version != ACM_INTERFACE_VERSION) {
		printk(KERN_WARNING "%s: %s %x != %x\n", __func__, current->comm,
				kern_op.interface_version, ACM_INTERFACE_VERSION);
		return -EACCES;
	}

	op_desc = xencomm_map(&kern_op, sizeof(xen_acmctl_t));
	if (op_desc == NULL)
		return -ENOMEM;

	switch (kern_op.cmd) {
	case ACMOP_setpolicy:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.setpolicy.pushcache),
			kern_op.u.setpolicy.pushcache_size);

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.setpolicy.pushcache,
		                     desc);
		break;
	case ACMOP_getpolicy:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.getpolicy.pullcache),
			kern_op.u.getpolicy.pullcache_size);

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.getpolicy.pullcache,
		                     desc);
		break;
	case ACMOP_dumpstats:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.dumpstats.pullcache),
			kern_op.u.dumpstats.pullcache_size);

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.dumpstats.pullcache,
		                     desc);
		break;
	case ACMOP_getssid:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.getssid.ssidbuf),
			kern_op.u.getssid.ssidbuf_size);

		if (desc == NULL)
			ret = -ENOMEM;

		set_xen_guest_handle(kern_op.u.getssid.ssidbuf,
		                     desc);
		break;
	case ACMOP_getdecision:
		break;
	case ACMOP_chgpolicy:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.change_policy.policy_pushcache),
			kern_op.u.change_policy.policy_pushcache_size);
		desc2 = xencomm_map(
		 	 xen_guest_handle(kern_op.u.change_policy.del_array),
		 	 kern_op.u.change_policy.delarray_size);
		desc3 = xencomm_map(
		 	 xen_guest_handle(kern_op.u.change_policy.chg_array),
		 	 kern_op.u.change_policy.chgarray_size);
		desc4 = xencomm_map(
		 	 xen_guest_handle(kern_op.u.change_policy.err_array),
		 	 kern_op.u.change_policy.errarray_size);

		if (desc  == NULL || desc2 == NULL ||
			desc3 == NULL || desc4 == NULL) {
			ret = -ENOMEM;
			goto out;
		}

		set_xen_guest_handle(kern_op.u.change_policy.policy_pushcache,
		                     desc);
		set_xen_guest_handle(kern_op.u.change_policy.del_array,
		                     desc2);
		set_xen_guest_handle(kern_op.u.change_policy.chg_array,
		                     desc3);
		set_xen_guest_handle(kern_op.u.change_policy.err_array,
		                     desc4);
		break;
	case ACMOP_relabeldoms:
		desc = xencomm_map(
			xen_guest_handle(kern_op.u.relabel_doms.relabel_map),
			kern_op.u.relabel_doms.relabel_map_size);
		desc2 = xencomm_map(
			xen_guest_handle(kern_op.u.relabel_doms.err_array),
			kern_op.u.relabel_doms.errarray_size);

		if (desc  == NULL || desc2 == NULL) {
			ret = -ENOMEM;
			goto out;
		}

		set_xen_guest_handle(kern_op.u.relabel_doms.relabel_map,
		                     desc);
		set_xen_guest_handle(kern_op.u.relabel_doms.err_array,
		                     desc2);
		break;
	default:
		printk(KERN_ERR "%s: unknown/unsupported acmctl cmd %d\n",
		       __func__, kern_op.cmd);
		return -ENOSYS;
	}

	if (ret)
		goto out; /* error mapping the nested pointer */

	ret = plpar_hcall_norets(XEN_MARK(hypercall->op),op_desc);

	if (copy_to_user(user_op, &kern_op, sizeof(xen_acmctl_t)))
		ret = -EFAULT;

out:
	xencomm_free(desc);
	xencomm_free(desc2);
	xencomm_free(desc3);
	xencomm_free(desc4);
	xencomm_free(op_desc);
	return ret;
}


/* The PowerPC hypervisor runs in a separate address space from Linux
 * kernel/userspace, i.e. real mode. We must therefore translate userspace
 * pointers to something the hypervisor can make sense of. */
int privcmd_hypercall(privcmd_hypercall_t *hypercall)
{
	switch (hypercall->op) {
	case __HYPERVISOR_domctl:
		return xenppc_privcmd_domctl(hypercall);
	case __HYPERVISOR_sysctl:
		return xenppc_privcmd_sysctl(hypercall);
	case __HYPERVISOR_platform_op:
		return xenppc_privcmd_platform_op(hypercall);
	case __HYPERVISOR_memory_op:
		return xenppc_privcmd_memory_op(hypercall);
	case __HYPERVISOR_xen_version:
		return xenppc_privcmd_version(hypercall);
	case __HYPERVISOR_event_channel_op:
		return xenppc_privcmd_event_channel_op(hypercall);
	case __HYPERVISOR_acm_op:
		return xenppc_acmcmd_op(hypercall);
	default:
		printk(KERN_ERR "%s: unknown hcall (%ld)\n", __func__, hypercall->op);
		/* maybe we'll get lucky and the hcall needs no translation. */
		return plpar_hcall_norets(XEN_MARK(hypercall->op),
				hypercall->arg[0],
				hypercall->arg[1],
				hypercall->arg[2],
				hypercall->arg[3],
				hypercall->arg[4]);
	}
}

int HYPERVISOR_vcpu_op(int cmd, int vcpuid, void *extra_args)
{
	int argsize;
	const unsigned long hcall = __HYPERVISOR_vcpu_op;
	struct xencomm_handle *desc;
	int rc;

	switch (cmd) {
	case  VCPUOP_initialise:
		argsize = sizeof(vcpu_guest_context_t);
		break;
	case VCPUOP_up:
	case VCPUOP_down:
	case VCPUOP_is_up:
		return plpar_hcall_norets(XEN_MARK(hcall), cmd, vcpuid, 0);

	case VCPUOP_get_runstate_info:
		argsize = sizeof (vcpu_runstate_info_t);
		break;
	default:
		printk(KERN_ERR "%s: unknown version cmd %d\n", __func__, cmd);
		return -ENOSYS;
	}

	desc = xencomm_map_no_alloc(extra_args, argsize);

	if (desc == NULL)
		return -EINVAL;

	rc = plpar_hcall_norets(XEN_MARK(hcall), cmd, vcpuid, desc);

	xencomm_free(desc);

	return rc;
}
