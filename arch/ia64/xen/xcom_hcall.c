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
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 *          Tristan Gingold <tristan.gingold@bull.net>
 *
 *          Copyright (c) 2007
 *          Isaku Yamahata <yamahata at valinux co jp>
 *                          VA Linux Systems Japan K.K.
 *          consolidate mini and inline version.
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/module.h>
#include <xen/interface/xen.h>
#include <xen/interface/platform.h>
#include <xen/interface/memory.h>
#include <xen/interface/xencomm.h>
#include <xen/interface/version.h>
#include <xen/interface/sched.h>
#include <xen/interface/event_channel.h>
#include <xen/interface/physdev.h>
#include <xen/interface/grant_table.h>
#include <xen/interface/callback.h>
#include <xen/interface/xsm/acm_ops.h>
#include <xen/interface/hvm/params.h>
#include <xen/interface/xenoprof.h>
#include <xen/interface/vcpu.h>
#include <xen/interface/kexec.h>
#include <asm/hypervisor.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/xen/xencomm.h>
#include <asm/perfmon.h>

/* Xencomm notes:
 * This file defines hypercalls to be used by xencomm.  The hypercalls simply
 * create inlines or mini descriptors for pointers and then call the raw arch
 * hypercall xencomm_arch_hypercall_XXX
 *
 * If the arch wants to directly use these hypercalls, simply define macros
 * in asm/hypercall.h, eg:
 *  #define HYPERVISOR_sched_op xencomm_hypercall_sched_op
 * 
 * The arch may also define HYPERVISOR_xxx as a function and do more operations
 * before/after doing the hypercall.
 *
 * Note: because only inline or mini descriptors are created these functions
 * must only be called with in kernel memory parameters.
 */

int
xencomm_hypercall_console_io(int cmd, int count, char *str)
{
	return xencomm_arch_hypercall_console_io
		(cmd, count, xencomm_map_no_alloc(str, count));
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_console_io);

int
xencomm_hypercall_event_channel_op(int cmd, void *op)
{
	struct xencomm_handle *desc;
	desc = xencomm_map_no_alloc(op, sizeof(evtchn_op_t));
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_event_channel_op(cmd, desc);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_event_channel_op);

int
xencomm_hypercall_xen_version(int cmd, void *arg)
{
	struct xencomm_handle *desc;
	unsigned int argsize;

	switch (cmd) {
	case XENVER_version:
		/* do not actually pass an argument */
		return xencomm_arch_hypercall_xen_version(cmd, 0);
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
		argsize = (arg == NULL) ? 0 : sizeof(void *);
		break;
	case XENVER_get_features:
		argsize = (arg == NULL) ? 0 : sizeof(xen_feature_info_t);
		break;

	default:
		printk("%s: unknown version op %d\n", __func__, cmd);
		return -ENOSYS;
	}

	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_xen_version(cmd, desc);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_xen_version);

int
xencomm_hypercall_physdev_op(int cmd, void *op)
{
	unsigned int argsize;
	
	switch (cmd) {
	case PHYSDEVOP_apic_read:
	case PHYSDEVOP_apic_write:
		argsize = sizeof(physdev_apic_t);
		break;
	case PHYSDEVOP_alloc_irq_vector:
	case PHYSDEVOP_free_irq_vector:
		argsize = sizeof(physdev_irq_t);
		break;
	case PHYSDEVOP_irq_status_query:
		argsize = sizeof(physdev_irq_status_query_t);
		break;

	default:
		printk("%s: unknown physdev op %d\n", __func__, cmd);
		return -ENOSYS;
	}
	
	return xencomm_arch_hypercall_physdev_op
		(cmd, xencomm_map_no_alloc(op, argsize));
}

static int
xencommize_grant_table_op(struct xencomm_mini **xc_area,
			  unsigned int cmd, void *op, unsigned int count,
			  struct xencomm_handle **desc)
{
	struct xencomm_handle *desc1;
	unsigned int argsize;

	switch (cmd) {
	case GNTTABOP_map_grant_ref:
		argsize = sizeof(struct gnttab_map_grant_ref);
		break;
	case GNTTABOP_unmap_grant_ref:
		argsize = sizeof(struct gnttab_unmap_grant_ref);
		break;
	case GNTTABOP_unmap_and_replace:
		argsize = sizeof(struct gnttab_unmap_and_replace);
		break;
	case GNTTABOP_setup_table:
	{
		struct gnttab_setup_table *setup = op;

		argsize = sizeof(*setup);

		if (count != 1)
			return -EINVAL;
		desc1 = __xencomm_map_no_alloc
			(xen_guest_handle(setup->frame_list),
			 setup->nr_frames *
			 sizeof(*xen_guest_handle(setup->frame_list)),
			 *xc_area);
		if (desc1 == NULL)
			return -EINVAL;
		(*xc_area)++;
		set_xen_guest_handle(setup->frame_list, (void *)desc1);
		break;
	}
	case GNTTABOP_dump_table:
		argsize = sizeof(struct gnttab_dump_table);
		break;
	case GNTTABOP_transfer:
		argsize = sizeof(struct gnttab_transfer);
		break;
	case GNTTABOP_copy:
		argsize = sizeof(struct gnttab_copy);
		break;
	case GNTTABOP_query_size:
		argsize = sizeof(struct gnttab_query_size);
		break;
	default:
		printk("%s: unknown hypercall grant table op %d\n",
		       __func__, cmd);
		BUG();
	}

	*desc = __xencomm_map_no_alloc(op, count * argsize, *xc_area);
	if (*desc == NULL)
		return -EINVAL;
	(*xc_area)++;

	return 0;
}

int
xencomm_hypercall_grant_table_op(unsigned int cmd, void *op,
                                      unsigned int count)
{
	int rc;
	struct xencomm_handle *desc;
	XENCOMM_MINI_ALIGNED(xc_area, 2);

	rc = xencommize_grant_table_op(&xc_area, cmd, op, count, &desc);
	if (rc)
		return rc;

	return xencomm_arch_hypercall_grant_table_op(cmd, desc, count);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_grant_table_op);

int
xencomm_hypercall_sched_op(int cmd, void *arg)
{
	struct xencomm_handle *desc;
	unsigned int argsize;

	switch (cmd) {
	case SCHEDOP_yield:
	case SCHEDOP_block:
		argsize = 0;
		break;
	case SCHEDOP_shutdown:
		argsize = sizeof(sched_shutdown_t);
		break;
	case SCHEDOP_remote_shutdown:
		argsize = sizeof(sched_remote_shutdown_t);
		break;
	case SCHEDOP_poll:
	{
		sched_poll_t *poll = arg;
		struct xencomm_handle *ports;

		argsize = sizeof(sched_poll_t);
		ports = xencomm_map_no_alloc(xen_guest_handle(poll->ports),
				     sizeof(*xen_guest_handle(poll->ports)));

		set_xen_guest_handle(poll->ports, (void *)ports);
		break;
	}
	default:
		printk("%s: unknown sched op %d\n", __func__, cmd);
		return -ENOSYS;
	}
	
	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_sched_op(cmd, desc);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_sched_op);

int
xencomm_hypercall_multicall(void *call_list, int nr_calls)
{
	int rc;
	int i;
	multicall_entry_t *mce;
	struct xencomm_handle *desc;
	XENCOMM_MINI_ALIGNED(xc_area, nr_calls * 2);

	for (i = 0; i < nr_calls; i++) {
		mce = (multicall_entry_t *)call_list + i;

		switch (mce->op) {
		case __HYPERVISOR_update_va_mapping:
		case __HYPERVISOR_mmu_update:
			/* No-op on ia64.  */
			break;
		case __HYPERVISOR_grant_table_op:
			rc = xencommize_grant_table_op
				(&xc_area,
				 mce->args[0], (void *)mce->args[1],
				 mce->args[2], &desc);
			if (rc)
				return rc;
			mce->args[1] = (unsigned long)desc;
			break;
		case __HYPERVISOR_memory_op:
		default:
			printk("%s: unhandled multicall op entry op %lu\n",
			       __func__, mce->op);
			return -ENOSYS;
		}
	}

	desc = xencomm_map_no_alloc(call_list,
				    nr_calls * sizeof(multicall_entry_t));
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_multicall(desc, nr_calls);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_multicall);

int
xencomm_hypercall_callback_op(int cmd, void *arg)
{
	unsigned int argsize;
	switch (cmd)
	{
	case CALLBACKOP_register:
		argsize = sizeof(struct callback_register);
		break;
	case CALLBACKOP_unregister:
		argsize = sizeof(struct callback_unregister);
		break;
	default:
		printk("%s: unknown callback op %d\n", __func__, cmd);
		return -ENOSYS;
	}

	return xencomm_arch_hypercall_callback_op
		(cmd, xencomm_map_no_alloc(arg, argsize));
}

static int
xencommize_memory_reservation(struct xencomm_mini *xc_area,
			      xen_memory_reservation_t *mop)
{
	struct xencomm_handle *desc;

	desc = __xencomm_map_no_alloc(xen_guest_handle(mop->extent_start),
			mop->nr_extents *
			sizeof(*xen_guest_handle(mop->extent_start)),
			xc_area);
	if (desc == NULL)
		return -EINVAL;

	set_xen_guest_handle(mop->extent_start, (void *)desc);
	return 0;
}

int
xencomm_hypercall_memory_op(unsigned int cmd, void *arg)
{
	XEN_GUEST_HANDLE(xen_pfn_t) extent_start_va[2];
	xen_memory_reservation_t *xmr = NULL, *xme_in = NULL, *xme_out = NULL;
	xen_memory_map_t *memmap = NULL;
	XEN_GUEST_HANDLE(void) buffer;
	int rc;
	struct xencomm_handle *desc;
	unsigned int argsize;
	XENCOMM_MINI_ALIGNED(xc_area, 2);

	switch (cmd) {
	case XENMEM_increase_reservation:
	case XENMEM_decrease_reservation:
	case XENMEM_populate_physmap:
		xmr = (xen_memory_reservation_t *)arg;
		set_xen_guest_handle(extent_start_va[0],
				     xen_guest_handle(xmr->extent_start));

		argsize = sizeof(*xmr);
		rc = xencommize_memory_reservation(xc_area, xmr);
		if (rc)
			return rc;
		xc_area++;
		break;

	case XENMEM_maximum_gpfn:
		argsize = 0;
		break;

	case XENMEM_maximum_ram_page:
		argsize = 0;
		break;

	case XENMEM_exchange:
		xme_in  = &((xen_memory_exchange_t *)arg)->in;
		xme_out = &((xen_memory_exchange_t *)arg)->out;
		set_xen_guest_handle(extent_start_va[0],
				     xen_guest_handle(xme_in->extent_start));
		set_xen_guest_handle(extent_start_va[1],
				     xen_guest_handle(xme_out->extent_start));

		argsize = sizeof(xen_memory_exchange_t);
		rc = xencommize_memory_reservation(xc_area, xme_in);
		if (rc)
			return rc;
		xc_area++;
		rc = xencommize_memory_reservation(xc_area, xme_out);
		if (rc)
			return rc;
		xc_area++;
		break;

	case XENMEM_add_to_physmap:
		argsize = sizeof(xen_add_to_physmap_t);
		break;

	case XENMEM_machine_memory_map:
		argsize = sizeof(*memmap);
		memmap = (xen_memory_map_t *)arg;
		set_xen_guest_handle(buffer, xen_guest_handle(memmap->buffer));
		desc = xencomm_map_no_alloc(xen_guest_handle(memmap->buffer),
					      memmap->nr_entries);
		if (desc == NULL)
			return -EINVAL;
		set_xen_guest_handle(memmap->buffer, (void *)desc);
		break;

	default:
		printk("%s: unknown memory op %d\n", __func__, cmd);
		return -ENOSYS;
	}

	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc == NULL)
		return -EINVAL;

	rc = xencomm_arch_hypercall_memory_op(cmd, desc);

	switch (cmd) {
	case XENMEM_increase_reservation:
	case XENMEM_decrease_reservation:
	case XENMEM_populate_physmap:
		set_xen_guest_handle(xmr->extent_start,
				     xen_guest_handle(extent_start_va[0]));
		break;

	case XENMEM_exchange:
		set_xen_guest_handle(xme_in->extent_start,
				     xen_guest_handle(extent_start_va[0]));
		set_xen_guest_handle(xme_out->extent_start,
				     xen_guest_handle(extent_start_va[1]));
		break;

	case XENMEM_machine_memory_map:
		set_xen_guest_handle(memmap->buffer, xen_guest_handle(buffer));
		break;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_memory_op);

unsigned long
xencomm_hypercall_hvm_op(int cmd, void *arg)
{
	struct xencomm_handle *desc;
	unsigned int argsize;

	switch (cmd) {
	case HVMOP_get_param:
	case HVMOP_set_param:
		argsize = sizeof(xen_hvm_param_t);
		break;
	default:
		printk("%s: unknown HVMOP %d\n", __func__, cmd);
		return -EINVAL;
	}

	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_hvm_op(cmd, desc);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_hvm_op);

int
xencomm_hypercall_suspend(unsigned long srec)
{
	struct sched_shutdown arg;

	arg.reason = SHUTDOWN_suspend;

	return xencomm_arch_hypercall_suspend(
		xencomm_map_no_alloc(&arg, sizeof(arg)));
}

int
xencomm_hypercall_xenoprof_op(int op, void *arg)
{
	unsigned int argsize;
	struct xencomm_handle *desc;

	switch (op) {
	case XENOPROF_init:
		argsize = sizeof(xenoprof_init_t);
		break;
	case XENOPROF_set_active:
		argsize = sizeof(domid_t);
		break;
	case XENOPROF_set_passive:
		argsize = sizeof(xenoprof_passive_t);
		break;
	case XENOPROF_counter:
		argsize = sizeof(xenoprof_counter_t);
		break;
	case XENOPROF_get_buffer:
		argsize = sizeof(xenoprof_get_buffer_t);
		break;

	case XENOPROF_reset_active_list:
	case XENOPROF_reset_passive_list:
	case XENOPROF_reserve_counters:
	case XENOPROF_setup_events:
	case XENOPROF_enable_virq:
	case XENOPROF_start:
	case XENOPROF_stop:
	case XENOPROF_disable_virq:
	case XENOPROF_release_counters:
	case XENOPROF_shutdown:
		return xencomm_arch_hypercall_xenoprof_op(op, arg);

	default:
		printk("%s: op %d isn't supported\n", __func__, op);
		return -ENOSYS;
	}

	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_xenoprof_op(op, desc);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_xenoprof_op);

int
xencomm_hypercall_perfmon_op(unsigned long cmd, void* arg,
                                  unsigned long count)
{
	unsigned int argsize;
	struct xencomm_handle *desc;

	switch (cmd) {
	case PFM_GET_FEATURES:
		argsize = sizeof(pfarg_features_t);
		break;
	case PFM_CREATE_CONTEXT:
		argsize = sizeof(pfarg_context_t);
		break;
	case PFM_LOAD_CONTEXT:
		argsize = sizeof(pfarg_load_t);
		break;
	case PFM_WRITE_PMCS:
	case PFM_WRITE_PMDS:
		argsize = sizeof(pfarg_reg_t) * count;
		break;

	case PFM_DESTROY_CONTEXT:
	case PFM_UNLOAD_CONTEXT:
	case PFM_START:
	case PFM_STOP:
		return xencomm_arch_hypercall_perfmon_op(cmd, arg, count);

	default:
		printk("%s:%d cmd %ld isn't supported\n",
		       __func__, __LINE__, cmd);
		BUG();
	}

	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_perfmon_op(cmd, desc, count);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_perfmon_op);

long
xencomm_hypercall_vcpu_op(int cmd, int cpu, void *arg)
{
	unsigned int argsize;
	switch (cmd) {
	case VCPUOP_register_runstate_memory_area: {
		vcpu_register_runstate_memory_area_t *area =
			(vcpu_register_runstate_memory_area_t *)arg;
		argsize = sizeof(*arg);
		set_xen_guest_handle(area->addr.h,
		     (void *)xencomm_map_no_alloc(area->addr.v,
						  sizeof(area->addr.v)));
		break;
	}

	default:
		printk("%s: unknown vcpu op %d\n", __func__, cmd);
		return -ENOSYS;
	}

	return xencomm_arch_hypercall_vcpu_op(cmd, cpu,
					xencomm_map_no_alloc(arg, argsize));
}

long
xencomm_hypercall_opt_feature(void *arg)
{
	return xencomm_arch_hypercall_opt_feature(
		xencomm_map_no_alloc(arg,
				     sizeof(struct xen_ia64_opt_feature)));
}

int
xencomm_hypercall_fpswa_revision(unsigned int *revision)
{
	struct xencomm_handle *desc;

	desc = xencomm_map_no_alloc(revision, sizeof(*revision));
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_fpswa_revision(desc);
}
EXPORT_SYMBOL_GPL(xencomm_hypercall_fpswa_revision);

int
xencomm_hypercall_kexec_op(int cmd, void *arg)
{
	unsigned int argsize;
	struct xencomm_handle *desc;

	switch (cmd) {
	case KEXEC_CMD_kexec_get_range:
		argsize = sizeof(xen_kexec_range_t);
		break;
	case KEXEC_CMD_kexec_load:
	case KEXEC_CMD_kexec_unload:
		argsize = sizeof(xen_kexec_load_t);
		break;
	case KEXEC_CMD_kexec:
		argsize = sizeof(xen_kexec_exec_t);
		break;
	default:
		printk("%s:%d cmd %d isn't supported\n",
		       __func__, __LINE__, cmd);
		BUG();
	}

	desc = xencomm_map_no_alloc(arg, argsize);
	if (desc == NULL)
		return -EINVAL;

	return xencomm_arch_hypercall_kexec_op(cmd, desc);
}
