/*
 * processor_extcntl_xen.c - interface to notify Xen
 *
 *  Copyright (C) 2008, Intel corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/acpi.h>
#include <linux/pm.h>
#include <linux/cpu.h>

#include <linux/cpufreq.h>
#include <acpi/processor.h>
#include <asm/hypercall.h>

static int xen_processor_pmbits;
static int __init set_xen_processor_pmbits(char *str)
{
	get_option(&str, &xen_processor_pmbits);

	return 1;
}
__setup("xen_processor_pmbits=", set_xen_processor_pmbits);
EXPORT_SYMBOL(xen_processor_pmbits);

static int xen_cx_notifier(struct acpi_processor *pr, int action)
{
	int ret, count = 0, i;
	xen_platform_op_t op = {
		.cmd			= XENPF_set_processor_pminfo,
		.interface_version	= XENPF_INTERFACE_VERSION,
		.u.set_pminfo.id	= pr->acpi_id,
		.u.set_pminfo.type	= XEN_PM_CX,
	};
	struct xen_processor_cx *data, *buf;
	struct acpi_processor_cx *cx;

	if (action == PROCESSOR_PM_CHANGE)
		return -EINVAL;

	/* Convert to Xen defined structure and hypercall */
	buf = kzalloc(pr->power.count * sizeof(struct xen_processor_cx),
			GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	data = buf;
	for (i = 1; i <= pr->power.count; i++) {
		cx = &pr->power.states[i];
		/* Skip invalid cstate entry */
		if (!cx->valid)
			continue;

		data->type = cx->type;
		data->latency = cx->latency;
		data->power = cx->power;
		data->reg.space_id = cx->reg.space_id;
		data->reg.bit_width = cx->reg.bit_width;
		data->reg.bit_offset = cx->reg.bit_offset;
		data->reg.access_size = cx->reg.reserved;
		data->reg.address = cx->reg.address;

		/* Get dependency relationships */
		if (cx->csd_count) {
			printk("Wow! _CSD is found. Not support for now!\n");
			kfree(buf);
			return -EINVAL;
		} else {
			data->dpcnt = 0;
			set_xen_guest_handle(data->dp, NULL);
		}

		data++;
		count++;
	}

	if (!count) {
		printk("No available Cx info for cpu %d\n", pr->acpi_id);
		kfree(buf);
		return -EINVAL;
	}

	op.u.set_pminfo.power.count = count;
	op.u.set_pminfo.power.flags.bm_control = pr->flags.bm_control;
	op.u.set_pminfo.power.flags.bm_check = pr->flags.bm_check;
	op.u.set_pminfo.power.flags.has_cst = pr->flags.has_cst;
	op.u.set_pminfo.power.flags.power_setup_done = pr->flags.power_setup_done;

	set_xen_guest_handle(op.u.set_pminfo.power.states, buf);
	ret = HYPERVISOR_platform_op(&op);
	kfree(buf);
	return ret;
}

static void convert_pct_reg(struct xen_pct_register *xpct,
	struct acpi_pct_register *apct)
{
	xpct->descriptor = apct->descriptor;
	xpct->length     = apct->length;
	xpct->space_id   = apct->space_id;
	xpct->bit_width  = apct->bit_width;
	xpct->bit_offset = apct->bit_offset;
	xpct->reserved   = apct->reserved;
	xpct->address    = apct->address;
}

static void convert_pss_states(struct xen_processor_px *xpss, 
	struct acpi_processor_px *apss, int state_count)
{
	int i;
	for(i=0; i<state_count; i++) {
		xpss->core_frequency     = apss->core_frequency;
		xpss->power              = apss->power;
		xpss->transition_latency = apss->transition_latency;
		xpss->bus_master_latency = apss->bus_master_latency;
		xpss->control            = apss->control;
		xpss->status             = apss->status;
		xpss++;
		apss++;
	}
}

static void convert_psd_pack(struct xen_psd_package *xpsd,
	struct acpi_psd_package *apsd)
{
	xpsd->num_entries    = apsd->num_entries;
	xpsd->revision       = apsd->revision;
	xpsd->domain         = apsd->domain;
	xpsd->coord_type     = apsd->coord_type;
	xpsd->num_processors = apsd->num_processors;
}

static int xen_px_notifier(struct acpi_processor *pr, int action)
{
	int ret;
	xen_platform_op_t op = {
		.cmd			= XENPF_set_processor_pminfo,
		.interface_version	= XENPF_INTERFACE_VERSION,
		.u.set_pminfo.id	= pr->acpi_id,
		.u.set_pminfo.type	= XEN_PM_PX,
	};
	struct xen_processor_performance *perf;
	struct xen_processor_px *states = NULL;
	struct acpi_processor_performance *px;
	struct acpi_psd_package *pdomain;

	/* leave dynamic ppc handle in the future */
	if (action == PROCESSOR_PM_CHANGE)
		return 0;

	perf = &op.u.set_pminfo.perf;
	px = pr->performance;

	perf->flags = XEN_PX_PPC | 
		      XEN_PX_PCT | 
		      XEN_PX_PSS | 
		      XEN_PX_PSD;

	/* ppc */
	perf->ppc = pr->performance_platform_limit;

	/* pct */
	convert_pct_reg(&perf->control_register, &px->control_register);
	convert_pct_reg(&perf->status_register, &px->status_register);

	/* pss */
	perf->state_count = px->state_count;
	states = kzalloc(px->state_count*sizeof(xen_processor_px_t),GFP_KERNEL);
	if (!states)
		return -ENOMEM;
	convert_pss_states(states, px->states, px->state_count);
	set_xen_guest_handle(perf->states, states);

	/* psd */
	pdomain = &px->domain_info;
	convert_psd_pack(&perf->domain_info, pdomain);
	if (perf->domain_info.num_processors) {
		if (pdomain->coord_type == DOMAIN_COORD_TYPE_SW_ALL)
			perf->shared_type = CPUFREQ_SHARED_TYPE_ALL;
		else if (pdomain->coord_type == DOMAIN_COORD_TYPE_SW_ANY)
			perf->shared_type = CPUFREQ_SHARED_TYPE_ANY;
		else if (pdomain->coord_type == DOMAIN_COORD_TYPE_HW_ALL)
			perf->shared_type = CPUFREQ_SHARED_TYPE_HW;
	} else
		perf->shared_type = CPUFREQ_SHARED_TYPE_NONE;

	ret = HYPERVISOR_platform_op(&op);
	kfree(states);
	return ret;
}

static int xen_tx_notifier(struct acpi_processor *pr, int action)
{
	return -EINVAL;
}
static int xen_hotplug_notifier(struct acpi_processor *pr, int event)
{
	return -EINVAL;
}

static struct processor_extcntl_ops xen_extcntl_ops = {
	.hotplug		= xen_hotplug_notifier,
};

static int __cpuinit xen_init_processor_extcntl(void)
{
	if (xen_processor_pmbits & XEN_PROCESSOR_PM_CX)
		xen_extcntl_ops.pm_ops[PM_TYPE_IDLE] = xen_cx_notifier;
	if (xen_processor_pmbits & XEN_PROCESSOR_PM_PX)
		xen_extcntl_ops.pm_ops[PM_TYPE_PERF] = xen_px_notifier;
	if (xen_processor_pmbits & XEN_PROCESSOR_PM_TX)
		xen_extcntl_ops.pm_ops[PM_TYPE_THR] = xen_tx_notifier;

	return processor_register_extcntl(&xen_extcntl_ops);
}
core_initcall(xen_init_processor_extcntl);
