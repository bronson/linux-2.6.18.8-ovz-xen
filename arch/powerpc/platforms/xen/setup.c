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
 * Copyright (C) IBM Corp. 2006
 *
 * Authors: Jimi Xenidis <jimix@watson.ibm.com>
 */

#define DEBUG
#define CONFIG_SHARE_MPIC

#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <xen/interface/xen.h>
#include <xen/interface/sched.h>
#include <xen/evtchn.h>
#include <xen/features.h>
#include <xen/xencons.h>
#include <asm/udbg.h>
#include <asm/pgtable.h>
#include <asm/prom.h>
#include <asm/iommu.h>
#include <asm/mmu.h>
#include <asm/abs_addr.h>
#include <asm/machdep.h>
#include <asm/hypervisor.h>
#include <asm/time.h>
#include <asm/pmc.h>
#include "setup.h"

#ifdef DEBUG
#define DBG(fmt...) udbg_printf(fmt)
#else
#define DBG(fmt...)
#endif

/* Apperently on other arches this could be used before its defined,
 * this should not be the case in PPC */
shared_info_t *HYPERVISOR_shared_info = (shared_info_t *)NULL;
EXPORT_SYMBOL(HYPERVISOR_shared_info);

/* Raw start-of-day parameters from the hypervisor. */
static start_info_t xsi;
start_info_t *xen_start_info;
EXPORT_SYMBOL(xen_start_info);

extern struct machdep_calls mach_maple_md;
extern void maple_pci_init(void);

static unsigned long foreign_mfn_flag;

/* Must be called with &vma->vm_mm->mmap_sem locked for write */
int direct_remap_pfn_range(struct vm_area_struct *vma,
		unsigned long address, 
		unsigned long mfn,
		unsigned long size, 
		pgprot_t prot,
		domid_t  domid)
{
	int rc;

	/* Set the MFN flag to tell Xen that this is not a PFN. */
	printk("%s: mapping mfn 0x%lx (size 0x%lx) -> 0x%lx\n", __func__,
			mfn, size, mfn | foreign_mfn_flag);
	mfn = mfn | foreign_mfn_flag;

	WARN_ON(!rwsem_is_locked(&vma->vm_mm->mmap_sem));
	rc = remap_pfn_range(vma, address, mfn, size, prot);

	return rc;
}

static void __init xen_fw_feature_init(void)
{
	DBG(" -> %s\n", __func__);

	powerpc_firmware_features = 0;

	powerpc_firmware_features |= FW_FEATURE_LPAR;
	powerpc_firmware_features |= FW_FEATURE_TCE | FW_FEATURE_DABR;
		
	printk(KERN_INFO "firmware_features = 0x%lx\n", 
			powerpc_firmware_features);

	DBG(" <- %s\n", __func__);
}

/* if these were global then I could get them from the pseries/setup.c */
static int pseries_set_dabr(unsigned long dabr)
{
	return plpar_hcall_norets(H_SET_DABR, dabr);
}

static int pseries_set_xdabr(unsigned long dabr)
{
	/* We want to catch accesses from kernel and userspace */
	return plpar_hcall_norets(H_SET_XDABR, dabr,
			H_DABRX_KERNEL | H_DABRX_USER);
}

/* 
 * Early initialization.
 */
static void __init xenppc_init_early(void)
{
	struct device_node *xen;

	DBG(" -> %s\n", __func__);

	xen = of_find_node_by_path("/xen");

	xen_start_info = &xsi;

	/* fill out start_info_t from devtree */
	if ((char *)get_property(xen, "privileged", NULL))
		xen_start_info->flags |= SIF_PRIVILEGED;
	if ((char *)get_property(xen, "initdomain", NULL))
		xen_start_info->flags |= SIF_INITDOMAIN;
	xen_start_info->shared_info = *((u64 *)get_property(xen, 
	   "shared-info", NULL));

	/* only look for store and console for guest domains */
	if (xen_start_info->flags == 0) {
		struct device_node *console = of_find_node_by_path("/xen/console");
		struct device_node *store = of_find_node_by_path("/xen/store");

		xen_start_info->store_mfn = (*((u64 *)get_property(store,
		   "reg", NULL))) >> PAGE_SHIFT;
		xen_start_info->store_evtchn = *((u32 *)get_property(store,
		   "interrupts", NULL));
		xen_start_info->console.domU.mfn = (*((u64 *)get_property(console,
		   "reg", NULL))) >> PAGE_SHIFT;
		xen_start_info->console.domU.evtchn = *((u32 *)get_property(console,
		   "interrupts", NULL));
	}

	HYPERVISOR_shared_info = __va(xen_start_info->shared_info);

	udbg_init_xen();

	DBG("xen_start_info at %p\n", xen_start_info);
	DBG("    magic          %s\n", xen_start_info->magic);
	DBG("    flags          %x\n", xen_start_info->flags);
	DBG("    shared_info    %lx, %p\n",
	    xen_start_info->shared_info, HYPERVISOR_shared_info);
	DBG("    store_mfn      %llx\n", xen_start_info->store_mfn);
	DBG("    store_evtchn   %x\n", xen_start_info->store_evtchn);
	DBG("    console_mfn    %llx\n", xen_start_info->console.domU.mfn);
	DBG("    console_evtchn %x\n", xen_start_info->console.domU.evtchn);

	xen_setup_time(&mach_maple_md);

	add_preferred_console("xvc", 0, NULL);

	if (get_property(xen, "power-control", NULL))
		xen_reboot_init(&mach_maple_md);
	else
		xen_reboot_init(NULL);

	if (is_initial_xendomain()) {
		u64 *mfnflag = (u64 *)get_property(xen, "mfn-flag", NULL);
		if (mfnflag) {
			foreign_mfn_flag = (1UL << mfnflag[0]);
			printk("OF: using 0x%lx as foreign mfn flag\n", foreign_mfn_flag);
		} else
			printk("OF: /xen/mfn-base must be present it build guests\n");
	}

	/* get the domain features */
	setup_xen_features();

	DBG("Hello World I'm Maple Xen-LPAR!\n");

	if (firmware_has_feature(FW_FEATURE_DABR))
		ppc_md.set_dabr = pseries_set_dabr;
	else if (firmware_has_feature(FW_FEATURE_XDABR))
		ppc_md.set_dabr = pseries_set_xdabr;

	iommu_init_early_pSeries();

	DBG(" <- %s\n", __func__);
}

/*
 * this interface is limiting
 */
static int running_on_xen;
int is_running_on_xen(void)
{
	return running_on_xen;
}
EXPORT_SYMBOL(is_running_on_xen);

static void xenppc_power_save(void)
{
	/* SCHEDOP_yield could immediately return. Instead, we
	 * want to idle in the Xen idle domain, so use
	 * SCHEDOP_block with a one-shot timer. */
	/* XXX do tickless stuff here. See
	 * linux-2.6-xen-sparse/arch/xen/i386/kernel/time.c */
	u64 now_ns = tb_to_ns(get_tb());
	u64 offset_ns = jiffies_to_ns(1);
	int rc;

	rc = HYPERVISOR_set_timer_op(now_ns + offset_ns);
	BUG_ON(rc != 0);

	HYPERVISOR_sched_op(SCHEDOP_block, NULL);
}

void __init xenppc_setup_arch(void)
{
	/* init to some ~sane value until calibrate_delay() runs */
	loops_per_jiffy = 50000000;

	/* Lookup PCI hosts */
	if (is_initial_xendomain())
		maple_pci_init();

#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif
#ifdef CONFIG_SMP
	/* let them fly */
	xen_setup_smp();
#endif

	printk(KERN_INFO "Using Xen idle loop\n");
}

static int __init xen_probe_flat_dt(unsigned long node,
				    const char *uname, int depth,
				    void *data)
{
	if (depth != 1)
		return 0;
	if (strcmp(uname, "xen") != 0)
 		return 0;

	running_on_xen = 1;

	return 1;
}

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
/* forward ref */
struct machdep_calls __initdata xen_md;
static int __init xenppc_probe(void)
{
	of_scan_flat_dt(xen_probe_flat_dt, NULL);

	if (!running_on_xen)
		return 0;

	xen_fw_feature_init();

	hpte_init_lpar();

	return 1;
}

static void __init xenppc_progress(char *s, unsigned short hex)
{
	printk("*** %04x : %s\n", hex, s ? s : "");
}

unsigned int xenppc_get_irq(struct pt_regs *regs)
{
	evtchn_do_upcall(regs);
	/* evtchn_do_upcall() handles all pending event channels directly, so there
	 * is nothing for do_IRQ() to do.
	 * XXX This means we aren't using IRQ stacks. */
	return NO_IRQ;
}

static void xenppc_enable_pmcs(void)
{
	unsigned long set, reset;

	power4_enable_pmcs();

	set = 1UL << 63;
	reset = 0;
	plpar_hcall_norets(H_PERFMON, set, reset);
}

#ifdef CONFIG_KEXEC
void xen_machine_kexec(struct kimage *image)
{
	panic("%s(%p): called\n", __func__, image);
}

int xen_machine_kexec_prepare(struct kimage *image)
{
	panic("%s(%p): called\n", __func__, image);
}

void xen_machine_crash_shutdown(struct pt_regs *regs)
{
	panic("%s(%p): called\n", __func__, regs);
}       
#endif

define_machine(xen) {
	.name			= "Xen-Maple",
	.probe			= xenppc_probe,
	.setup_arch		= xenppc_setup_arch,
	.init_early		= xenppc_init_early,
	.init_IRQ		= xen_init_IRQ,
	.get_irq		= xenppc_get_irq,
	.calibrate_decr		= generic_calibrate_decr,
	.progress		= xenppc_progress,
	.power_save		= xenppc_power_save,
	.enable_pmcs	= xenppc_enable_pmcs,
#ifdef CONFIG_KEXEC
	.machine_kexec		= xen_machine_kexec,
	.machine_kexec_prepare	= xen_machine_kexec_prepare,
	.machine_crash_shutdown	= xen_machine_crash_shutdown,
#endif
};
