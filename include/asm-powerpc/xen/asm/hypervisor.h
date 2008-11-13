/******************************************************************************
 * hypervisor.h
 * 
 * Linux-specific hypervisor handling.
 * 
 * Copyright (c) 2002-2004, K A Fraser
 * 
 * This file may be distributed separately from the Linux kernel, or
 * incorporated into other software packages, subject to the following license:
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef __HYPERVISOR_H__
#define __HYPERVISOR_H__

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <xen/interface/xen.h>
#include <asm/ptrace.h>
#include <asm/page.h>
#include <asm/irq.h>

extern shared_info_t *HYPERVISOR_shared_info;

/* arch/xen/i386/kernel/setup.c */
extern start_info_t *xen_start_info;

#ifdef CONFIG_XEN_PRIVILEGED_GUEST
#define is_initial_xendomain() (xen_start_info && \
				(xen_start_info->flags & SIF_INITDOMAIN))
#else
#define is_initial_xendomain() 0
#endif

/* arch/xen/kernel/evtchn.c */
/* Force a proper event-channel callback from Xen. */
void force_evtchn_callback(void);

/* arch/xen/kernel/process.c */
void xen_cpu_idle (void);

/* arch/xen/i386/kernel/hypervisor.c */
void do_hypervisor_callback(struct pt_regs *regs);

/* arch/xen/i386/kernel/head.S */
void lgdt_finish(void);

/* arch/xen/i386/mm/hypervisor.c */
/*
 * NB. ptr values should be PHYSICAL, not MACHINE. 'vals' should be already
 * be MACHINE addresses.
 */

void xen_pt_switch(unsigned long ptr);
void xen_new_user_pt(unsigned long ptr); /* x86_64 only */
void xen_load_gs(unsigned int selector); /* x86_64 only */
void xen_tlb_flush(void);
void xen_invlpg(unsigned long ptr);

#ifndef CONFIG_XEN_SHADOW_MODE
void xen_l1_entry_update(pte_t *ptr, pte_t val);
void xen_l2_entry_update(pmd_t *ptr, pmd_t val);
void xen_l3_entry_update(pud_t *ptr, pud_t val); /* x86_64/PAE */
void xen_l4_entry_update(pgd_t *ptr, pgd_t val); /* x86_64 only */
void xen_pgd_pin(unsigned long ptr);
void xen_pgd_unpin(unsigned long ptr);
void xen_pud_pin(unsigned long ptr); /* x86_64 only */
void xen_pud_unpin(unsigned long ptr); /* x86_64 only */
void xen_pmd_pin(unsigned long ptr); /* x86_64 only */
void xen_pmd_unpin(unsigned long ptr); /* x86_64 only */
void xen_pte_pin(unsigned long ptr);
void xen_pte_unpin(unsigned long ptr);
#else
#define xen_l1_entry_update(_p, _v) set_pte((_p), (_v))
#define xen_l2_entry_update(_p, _v) set_pgd((_p), (_v))
#define xen_pgd_pin(_p)   ((void)0)
#define xen_pgd_unpin(_p) ((void)0)
#define xen_pte_pin(_p)   ((void)0)
#define xen_pte_unpin(_p) ((void)0)
#endif

void xen_set_ldt(unsigned long ptr, unsigned long bytes);
void xen_machphys_update(unsigned long mfn, unsigned long pfn);

#ifdef CONFIG_SMP
#include <linux/cpumask.h>
void xen_tlb_flush_all(void);
void xen_invlpg_all(unsigned long ptr);
void xen_tlb_flush_mask(cpumask_t *mask);
void xen_invlpg_mask(cpumask_t *mask, unsigned long ptr);
#endif

/* Returns zero on success else negative errno. */
static inline int xen_create_contiguous_region(
    unsigned long vstart, unsigned int order, unsigned int address_bits)
{
	return 0;
}
static inline void xen_destroy_contiguous_region(
    unsigned long vstart, unsigned int order)
{
	return;
}

#include <asm/hypercall.h>

/* BEGIN: all of these need a new home */
struct vm_area_struct;
int direct_remap_pfn_range(struct vm_area_struct *vma,  unsigned long address,
			   unsigned long mfn, unsigned long size,
			   pgprot_t prot, domid_t  domid);
#define	pfn_to_mfn(x)	(x)
#define	mfn_to_pfn(x)	(x)
#define phys_to_machine(phys) ((maddr_t)(phys))
#define phys_to_machine_mapping_valid(pfn) (1)

/* VIRT <-> MACHINE conversion */
#define virt_to_machine(v)	(phys_to_machine(__pa(v)))
#define machine_to_virt(m)	(__va(m))
#define virt_to_mfn(v)		(pfn_to_mfn(__pa(v) >> PAGE_SHIFT))
#define mfn_to_virt(m)		(__va(mfn_to_pfn(m) << PAGE_SHIFT))


#define PIRQ_BASE		0
#define NR_PIRQS		256

#define DYNIRQ_BASE		(PIRQ_BASE + NR_PIRQS)
#define NR_DYNIRQS		256

#define NR_IPIS 4		/* PPC_MSG_DEBUGGER_BREAK + 1 */

#if NR_IRQS < (NR_PIRQS + NR_DYNIRQS)
#error to many Xen IRQs
#endif

#define NR_IRQ_VECTORS		NR_IRQS

#define pirq_to_irq(_x)		((_x) + PIRQ_BASE)
#define irq_to_pirq(_x)		((_x) - PIRQ_BASE)

#define dynirq_to_irq(_x)	((_x) + DYNIRQ_BASE)
#define irq_to_dynirq(_x)	((_x) - DYNIRQ_BASE)


/* END:  all of these need a new home */

#if defined(CONFIG_X86_64)
#define MULTI_UVMFLAGS_INDEX 2
#define MULTI_UVMDOMID_INDEX 3
#else
#define MULTI_UVMFLAGS_INDEX 3
#define MULTI_UVMDOMID_INDEX 4
#endif

extern int is_running_on_xen(void);

static inline void
MULTI_update_va_mapping(
    multicall_entry_t *mcl, unsigned long va,
    pte_t new_val, unsigned long flags)
{
    mcl->op = __HYPERVISOR_update_va_mapping;
    mcl->args[0] = va;
#if defined(CONFIG_X86_64)
    mcl->args[1] = new_val.pte;
    mcl->args[2] = flags;
#elif defined(CONFIG_X86_PAE)
    mcl->args[1] = new_val.pte_low;
    mcl->args[2] = new_val.pte_high;
    mcl->args[3] = flags;
#elif defined(CONFIG_PPC64)
    mcl->args[1] = pte_val(new_val);
    mcl->args[2] = 0;
    mcl->args[3] = flags;
#else
    mcl->args[1] = new_val.pte_low;
    mcl->args[2] = 0;
    mcl->args[3] = flags;
#endif
}

static inline void
MULTI_update_va_mapping_otherdomain(
    multicall_entry_t *mcl, unsigned long va,
    pte_t new_val, unsigned long flags, domid_t domid)
{
    mcl->op = __HYPERVISOR_update_va_mapping_otherdomain;
    mcl->args[0] = va;
#if defined(CONFIG_X86_64)
    mcl->args[1] = new_val.pte;
    mcl->args[2] = flags;
    mcl->args[3] = domid;
#elif defined(CONFIG_X86_PAE)
    mcl->args[1] = new_val.pte_low;
    mcl->args[2] = new_val.pte_high;
    mcl->args[3] = flags;
    mcl->args[4] = domid;
#elif defined(CONFIG_PPC64)
    mcl->args[1] = pte_val(new_val);
    mcl->args[2] = 0;
    mcl->args[3] = flags;
    mcl->args[4] = domid;
#else
    mcl->args[1] = new_val.pte_low;
    mcl->args[2] = 0;
    mcl->args[3] = flags;
    mcl->args[4] = domid;
#endif
}

#define INVALID_P2M_ENTRY (~0UL)
#define FOREIGN_FRAME(m) (INVALID_P2M_ENTRY)
static inline void set_phys_to_machine(unsigned long pfn, unsigned long mfn)
{
	if (pfn != mfn && mfn != INVALID_P2M_ENTRY)
		printk(KERN_EMERG "%s: pfn: 0x%lx mfn: 0x%lx\n",
		       __func__, pfn, mfn);
	
	return;
}
#define pfn_pte_ma(pfn, prot)	__pte(((pfn) << PAGE_SHIFT) | pgprot_val(prot))

typedef unsigned long maddr_t;
typedef unsigned long paddr_t;

#ifdef CONFIG_XEN_SCRUB_PAGES

static inline void scrub_pages(void *p, unsigned n)
{
	unsigned i;

	for (i = 0; i < n; i++) {
		clear_page(p);
		p += PAGE_SIZE;
	}
}
#else
#define scrub_pages(_p,_n) ((void)0)
#endif

/*
 * for blktap.c
 * int create_lookup_pte_addr(struct mm_struct *mm, 
 *                            unsigned long address,
 *                            uint64_t *ptep);
 */
#define create_lookup_pte_addr(mm, address, ptep)			\
	({								\
		printk(KERN_EMERG					\
		       "%s:%d "						\
		       "create_lookup_pte_addr() isn't supported.\n",	\
		       __func__, __LINE__);				\
		BUG();							\
		(-ENOSYS);						\
	})

#endif /* __HYPERVISOR_H__ */
