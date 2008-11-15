/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file provides compatibility layer for various Linux kernel versions
 * (starting from 2.6.9 RHEL kernel).
 *
 * Copyright 2005-2007: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed and maintained by Solarflare Communications:
 *                      <linux-xen-drivers@solarflare.com>
 *                      <onload-dev@solarflare.com>
 *
 * Certain parts of the driver were implemented by
 *          Alexandra Kossovsky <Alexandra.Kossovsky@oktetlabs.ru>
 *          OKTET Labs Ltd, Russia,
 *          http://oktetlabs.ru, <info@oktetlabs.ru>
 *          by request of Solarflare Communications
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation, incorporated herein by reference.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 ****************************************************************************
 */

#define IN_KERNEL_COMPAT_C
#include <linux/types.h>
#include <ci/efrm/debug.h>
#include "kernel_compat.h"

/* Set this to 1 to enable very basic counting of iopage(s) allocations, then
 * call dump_iopage_counts() to show the number of current allocations of
 * orders 0-7.
 */
#define EFRM_IOPAGE_COUNTS_ENABLED 0



/* I admit that it's a bit ugly going straight to the field, but it
 * seems easiest given that get_page followed by put_page on a page
 * with PG_reserved set will increment the ref count on 2.6.14 and
 * below, but not 2.6.15.  Also, RedHat have hidden put_page_testzero
 * in a header file which produces warnings when compiled.  This
 * doesn't agree with our use of -Werror.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)
# define page_count_field(pg)  ((pg)->count)
#else
# define page_count_field(pg)  ((pg)->_count)
#endif

#define inc_page_count(page)   atomic_inc(&page_count_field(page))
#define dec_page_count(page)   atomic_dec(&page_count_field(page))

/* Bug 5531: set_page_count doesn't work if the new page count is an
 * expression. */
#define ci_set_page_count(page, n) set_page_count(page, (n))

  /* Bug 3965: Gak!  Reference counts just don't work on memory
   * allocated through pci_alloc_consistent.  Different versions and
   * architectures do different things.  There are several interacting
   * bugs/features which have been described below and then summarised
   * in a table for each kernel version.  For each feature, there is a
   * question, a short description, a hint at how to examine the
   * kernel code for this feature and a description of the keys in the
   * table.
   *
   * A. Is PG_compound set on multi-page allocations?
   *
   *    When a multi-page allocation succeeds, the kernel sets the
   *    reference count of the first page to one and the count of the
   *    remaining pages to zero.  This is an immediate problem because
   *    if these pages are mapped into user space, the VM will do
   *    get_page followed by put_page, at which point the reference
   *    count will return to zero and the page will be freed.
   *    PG_compound was introduced in 2.6.0 and back-ported to rhel3
   *    kernels.  When it is set, all the pages have a pointer to the
   *    first page so that they can share the reference count.  If
   *    PG_compound is set, calling get_page(pg+1) can change
   *    page_count(pg).  It was originally set on all multi-page
   *    allocations, but later only set if the __GFP_COMP flag was
   *    provided to the allocator.
   *
   *    See mm/page_alloc.c
   *      Does prep_compound_page get called when __GFP_COMP not set?
   *
   * Keys:
   *    NotDef - prep_compound_page and PG_compound are not defined.
   *    Comp   - prep_compound_page is called for any multi-page allocation.
   *    Opt    - prep_compound_page is only called if __GFP_COMP is set.
   *    OptInv - prep_compound_page is only called if __GFP_NO_COMP is not set.
   *
   * B. Are bounce buffers ever used to satisfy pci_alloc_consistent?
   *    (x86_64 only)
   *
   *    2.6 kernels introduced bounce buffers on x86_64 machines to access
   *    memory above 4G when using the DMA mapping API.  At some point,
   *    code was added to allow pci_alloc_consistent/dma_alloc_coherent to
   *    allocate memory from the bounce buffers if the general purpose
   *    allocator produced memory which wasn't suitable.  Such memory can
   *    be recognised by the PG_reserved bit being set.  At a later point,
   *    the __GFP_DMA32 flag was added and used to restrict the allocator
   *    to below 4G.  The effect of this later change was that 4G capable
   *    cards would no longer get memory from the bounce buffers, although
   *    a card which can address less than 4G might get memory from the
   *    bounce buffers.
   *
   *    See dma_alloc_coherent or pci_alloc_consistent in
   *    arch/x86_64/kernel/pci-gart.c or arch/x86/kernel/pci-dma_64.c
   *      Is (gfp |= GFP_DMA32) before dma_alloc_pages?
   *      Is swiotlb_alloc_coherent called?
   *
   * Keys:
   *    NU     - bounce buffers are Never Used
   *    Used   - bounce buffers are sometimes used
   *
   * C. Does munmap decrement the reference count of a PG_reserved page?
   *
   *    Originally, the munmap code would not decrement the reference count
   *    of a page which had PG_reserved set.  At some point in the 2.6
   *    series, VM_PFNMAP was introduced and could be set on a vma to
   *    indicate that no pages in that vma should have the reference count
   *    decremented (unless they are copy-on-write copies).  At that point,
   *    the check for PG_reserved pages in the munmap code path was
   *    removed.  Some hackery in vm_normal_page means that a VM_PFNMAP vma
   *    must map contiguous physical pages.  As a result, such pages should
   *    be mapped during mmap using remap_pfn_range (for an example, see
   *    drivers/char/mem.c).
   *
   *    In 2.6 kernels: See release_pages in mm/swap.c
   *      Does PageReserved get tested?
   *    In 2.6 kernels: See mm/memory.c
   *      Is VM_PFNMAP used?
   *    In 2.4 kernels: See __free_pte in mm/memory.c
   *      Does PageReserved get tested?
   *    In 2.4 kernels: See __free_pages in mm/page_alloc.c
   *      Does PageReserved get tested?
   *
   * Keys:
   *    resv   - The reference count is not touched for PG_reserved pages.
   *    pfnmap - The VM_PFNMAP flag is checked instead of PG_reserved.
   *
   * D. Does munmap honour the PG_compound bit?
   *
   *    When PG_compound was originally introduced, the munmap code path
   *    didn't check it before decrementing the reference count on the
   *    page.  As a result, the wrong reference count would be updated if a
   *    PG_compound page was ever mapped into user space.
   *
   *    In 2.6 kernels: See release_pages in mm/swap.c
   *      Does PageCompound get tested?
   *    In 2.4 kernels: See __free_pages in mm/page_alloc.c
   *      Does PageCompound get tested?
   *
   * Keys:
   *    NotHon - The PG_compound bit isn't honoured by munmap.
   *    Hon    - The PG_compound bit is honoured by munmap.
   *
   *                 OS      A       B       C       D
   * 2.4.18                  NotDef  NU      resv    NotHon
   * 2.4.29                  NotDef  NU      resv    NotHon
   * 2.4.20-31.9     rhl9    NotDef  NU      resv    NotHon
   *
   * 2.4.21-4.EL     rhel3   Comp    NU      resv    Hon
   * 2.4.21-15.EL    rhel3   Comp    NU      resv    Hon
   * 2.4.21-32.EL    rhel3   Comp    NU      resv    Hon
   * 2.4.21-40.EL    rhel3   Comp    NU      resv    Hon
   *
   * 2.6.0                   Comp    NU      resv    NotHon
   *
   * 2.6.5-7.97      sles9   OptInv  NU      resv    NotHon
   * 2.6.9           rhel4   Opt     NU      resv    NotHon
   *
   * 2.6.11          fc4     ?       ?       ?       ?
   * 2.6.12          fc4     Opt     Used    resv    NotHon
   * 2.6.13                  Opt     Used    resv    NotHon
   *
   * 2.6.15                  Opt     NU      pfnmap  NotHon
   *
   * 2.6.16                  Opt     NU      pfnmap  Hon
   * 2.6.16.9                Opt     NU      pfnmap  Hon
   * 2.6.17.2                Opt     NU      pfnmap  Hon
   * 2.6.24-rc7      k.org   Opt     NU      pfnmap  Hon
   *
   * This LKML thread gives some low down on mapping pages into user
   * space and using DMA.
   *  http://www.forbiddenweb.org/viewtopic.php?id=83167&page=1
   *
   * There is no problem with single page allocations (until some
   * kernel hands us a PG_reserved page and expects us to use
   * VM_PFNMAP on the vma).
   *
   * Bug 5450: Most kernels set the reference count to one on the
   * first sub-page of a high-order page allocation and zero on
   * subsequent sub-pages.  Some kernels, however, set the page count
   * to one on all the sub-pages.  The SLES 9 range are affected, as
   * are kernels built without CONFIG_MMU defined.
   *
   * Possible strategies for multi-page allocations:
   *
   * EFRM_MMAP_USE_COMPOUND
   * 1. Allocate a compound page.  Reference counting should then work
   *    on the whole allocation.  This is a good theory, but is broken
   *    by bug/feature D (above).
   *
   * EFRM_MMAP_USE_SPLIT
   * 2. Convert the multi-page allocation to many single page
   *    allocations.  This involves incrementing the reference counts
   *    and clearing PG_compound on all the pages (including the
   *    first).  The references should be released _after_ calling
   *    pci_free_consistent so that that call doesn't release the
   *    memory.
   *
   * EFRM_MMAP_USE_INCREMENT
   * 3. Increment the reference count on all the pages after
   *    allocating and decrement them again before freeing.  This gets
   *    round the zero reference count problem.  It doesn't handle the
   *    case where someone else is holding a reference to one of our
   *    pages when we free the pages, but we think VM_IO stops this
   *    from happening.
   */

/* Should we use strategy 1?  This can be forced on us by the OS. */
#if defined(PG_compound)
#define EFRM_MMAP_USE_COMPOUND 1
#else
#define EFRM_MMAP_USE_COMPOUND 0
#endif

/* Should we use strategy 2?  This can be used even if strategy 1 is
 * used. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#define EFRM_MMAP_USE_SPLIT    1
#else
#define EFRM_MMAP_USE_SPLIT    0
#endif

/* Should we use strategy 3?  There's no point doing this if either
 * strategy 1 or strategy 2 is used. */
#if !EFRM_MMAP_USE_COMPOUND && !EFRM_MMAP_USE_SPLIT
#error "We shouldn't have to use this strategy."
#define EFRM_MMAP_USE_INCREMENT 1
#else
#define EFRM_MMAP_USE_INCREMENT 0
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17)
#define EFRM_MMAP_RESET_REFCNT  1
#else
#define EFRM_MMAP_RESET_REFCNT  0
#endif

/* NB. 2.6.17 has renamed SetPageCompound to __SetPageCompound and
 * ClearPageCompound to __ClearPageCompound. */
#if ((defined(PageCompound)        !=  defined(PG_compound)) ||	\
     (defined(SetPageCompound)     !=  defined(PG_compound) &&	\
      defined(__SetPageCompound)   !=  defined(PG_compound)) ||	\
     (defined(ClearPageCompound)   !=  defined(PG_compound) &&	\
      defined(__ClearPageCompound) !=  defined(PG_compound)) ||	\
     (defined(__GFP_COMP)          && !defined(PG_compound)))
#error Mismatch of defined page-flags.
#endif

extern int use_pci_alloc;	/* Use pci_alloc_consistent to alloc iopages */

/****************************************************************************
 *
 * allocate a buffer suitable for DMA to/from the NIC
 *
 ****************************************************************************/

static inline void pci_mmap_pages_hack_after_alloc(caddr_t kva, unsigned order)
{
	unsigned pfn = __pa(kva) >> PAGE_SHIFT;
	struct page *start_pg = pfn_to_page(pfn);
#if !defined(NDEBUG) || EFRM_MMAP_USE_SPLIT
	struct page *end_pg = start_pg + (1 << order);
	struct page *pg;
#endif

	/* Compound pages don't get created for order 0 pages and there's no
	 * fixing up needs to be done. */
	if (order == 0)
		return;

	/* If we've been given a reserved page then it must have come from
	 * the bounce buffer pool. */
	if (PageReserved(start_pg)) {
#if defined(VM_PFNMAP) || !defined(__x86_64__)
		/* Kernel allocated reserved pages when not expected */
		BUG();
#endif
		return;
	}

	/* Check the page count and PG_compound bit. */
#ifndef NDEBUG
#  if defined(PG_compound)
	EFRM_ASSERT(PageCompound(start_pg) == EFRM_MMAP_USE_COMPOUND);
#  endif
	EFRM_ASSERT(page_count(start_pg) == 1);

	{
		/* Some kernels have the page count field hold (ref_count-1)
		 * rather than (ref_count).  This is so that decrementing the
		 * reference count to "zero" causes the internal value to change
		 * from 0 to -1 which sets the carry flag.  Other kernels store
		 * the real reference count value in the obvious way.  We handle
		 * this here by reading the reference count field of the first
		 * page, which is always 1. */
		int pg_count_zero;
		pg_count_zero = atomic_read(&page_count_field(start_pg)) - 1;
		for (pg = start_pg + 1; pg < end_pg; pg++) {
			int pg_count;
#  if defined(PG_compound)
			EFRM_ASSERT(PageCompound(pg) == EFRM_MMAP_USE_COMPOUND);
#  endif

			/* Bug 5450: Some kernels initialise the page count
			 * to one for pages other than the first and some
			 * leave it at zero.  We allow either behaviour
			 * here, but disallow anything strange.  Newer
			 * kernels only define set_page_count in an
			 * internal header file, so we have to make do with
			 * incrementing and decrementing the reference
			 * count.  Fortunately, those kernels don't set the
			 * reference count to one on all the pages. */
			pg_count = atomic_read(&page_count_field(pg));
#  if EFRM_MMAP_RESET_REFCNT
			if (pg_count != pg_count_zero)
				EFRM_ASSERT(pg_count == pg_count_zero + 1);
#  else
			EFRM_ASSERT(pg_count == pg_count_zero);
#  endif
		}
	}
#endif

	/* Split the multi-page allocation if necessary. */
#if EFRM_MMAP_USE_SPLIT
	for (pg = start_pg; pg < end_pg; pg++) {

		/* This is no longer a compound page. */
#  if EFRM_MMAP_USE_COMPOUND
		ClearPageCompound(pg);
		EFRM_ASSERT(PageCompound(pg) == 0);
#  endif

#  ifndef NDEBUG
		{
			int pg_count = page_count(pg);
			/* Bug 5450: The page count can be zero or one here. */
			if (pg == start_pg) {
				EFRM_ASSERT(pg_count == 1);
			} else {
#    if EFRM_MMAP_RESET_REFCNT
				if (pg_count != 0)
					EFRM_ASSERT(pg_count == 1);
#    else
				EFRM_ASSERT(pg_count == 0);
#    endif
			}
		}
#  endif

		/* Get a reference which will be released after the pages have
		 * been passed back to pci_free_consistent. */
#  if EFRM_MMAP_RESET_REFCNT
		/* Bug 5450: Reset the reference count since the count might
		 * already be 1. */
		ci_set_page_count(pg, (pg == start_pg) ? 2 : 1);
#  else
		get_page(pg);
#  endif
	}
#endif

	/* Fudge the reference count if necessary. */
#if EFRM_MMAP_USE_INCREMENT
	for (pg = start_pg; pg < end_pg; pg++)
		inc_page_count(pg);
#endif
}

static inline void pci_mmap_pages_hack_before_free(caddr_t kva, unsigned order)
{
#if EFRM_MMAP_USE_INCREMENT || !defined(NDEBUG)
	/* Drop the references taken in pci_mmap_pages_hack_after_alloc */
	unsigned pfn = __pa(kva) >> PAGE_SHIFT;
	struct page *start_pg = pfn_to_page(pfn);
	struct page *end_pg = start_pg + (1 << order);
	struct page *pg;

	/* Compound pages don't get created for order 0 pages and there's no
	 * fixing up needs to be done. */
	if (order == 0)
		return;

	if (PageReserved(start_pg))
		return;

#  if EFRM_MMAP_USE_INCREMENT
	for (pg = start_pg; pg < end_pg; pg++)
		dec_page_count(pg);
#  endif

#if !defined(NDEBUG)
	EFRM_ASSERT(page_count(start_pg) == 1+EFRM_MMAP_USE_SPLIT);

#  if EFRM_MMAP_USE_COMPOUND && !EFRM_MMAP_USE_SPLIT
	for (pg = start_pg; pg < end_pg; pg++)
		EFRM_ASSERT(PageCompound(pg));
#  else
	for (pg = start_pg+1; pg < end_pg; pg++) {
		unsigned exp_pg_count = EFRM_MMAP_USE_SPLIT;
		/* NB.  If this assertion fires, either we've messed up the
		 * page counting or someone is holding on to a reference.
		 */
		EFRM_ASSERT(page_count(pg) == exp_pg_count);
	}
#  endif
#endif

#endif
}

static inline void pci_mmap_pages_hack_after_free(caddr_t kva, unsigned order)
{
#if EFRM_MMAP_USE_SPLIT
	/* Drop the references taken in pci_mmap_pages_hack_after_alloc */
	unsigned pfn = __pa(kva) >> PAGE_SHIFT;
	struct page *start_pg = pfn_to_page(pfn);
	struct page *end_pg = start_pg + (1 << order);
	struct page *pg;

	/* Compound pages don't get created for order 0 pages and there's no
	 * fixing up needs to be done. */
	if (order == 0)
		return;

	if (PageReserved(start_pg))
		return;

	for (pg = start_pg; pg < end_pg; pg++) {
		EFRM_ASSERT(page_count(pg) == 1);
		put_page(pg);
	}
#endif
}


#if EFRM_IOPAGE_COUNTS_ENABLED

static int iopage_counts[8];

void dump_iopage_counts(void)
{
	EFRM_NOTICE("iopage counts: %d %d %d %d %d %d %d %d", iopage_counts[0],
		    iopage_counts[1], iopage_counts[2], iopage_counts[3],
		    iopage_counts[4], iopage_counts[5], iopage_counts[6],
		    iopage_counts[7]);
}

#endif



/*********** pci_alloc_consistent / pci_free_consistent ***********/

void *efrm_dma_alloc_coherent(struct device *dev, size_t size,
			      dma_addr_t *dma_addr, int flag)
{
	struct pci_dev *pci_dev;
	void *ptr;
	unsigned order;
	EFRM_IOMMU_DECL;

	order = __ffs(size/PAGE_SIZE);
	EFRM_ASSERT(size == (PAGE_SIZE<<order));

	/* NB. The caller may well set __GFP_COMP.  However we can't
	 * rely on this working on older kernels.  2.6.9 only acts on
	 * __GFP_COMP if CONFIG_HUGETLB_PAGE is defined.  If the flag
	 * did have an effect then PG_compound will be set on the
	 * pages. */

	if (use_pci_alloc) {
		/* Can't take a spinlock here since the allocation can
		 * block. */
		ptr = dma_alloc_coherent(dev, size, dma_addr, flag);
		if (ptr == NULL)
			return ptr;
	} else {
#ifdef CONFIG_SWIOTLB		/* BUG1340 */
		if (swiotlb) {
			EFRM_ERR("%s: This kernel is using DMA bounce "
				 "buffers.  Please upgrade kernel to "
				 "linux2.6 or reduce the amount of RAM "
				 "with mem=XXX.", __FUNCTION__);
			return NULL;
		}
#endif
		ptr = (void *)__get_free_pages(flag, order);

		if (ptr == NULL)
			return NULL;

		EFRM_IOMMU_LOCK();
		pci_dev = container_of(dev, struct pci_dev, dev);
		*dma_addr = pci_map_single(pci_dev, ptr, size,
					   PCI_DMA_BIDIRECTIONAL);
		EFRM_IOMMU_UNLOCK();
		if (pci_dma_mapping_error(*dma_addr)) {
			free_pages((unsigned long)ptr, order);
			return NULL;
		}
	}

#ifndef CONFIG_IA64
	pci_mmap_pages_hack_after_alloc(ptr, order);
#endif

#if EFRM_IOPAGE_COUNTS_ENABLED
	if (order < 8)
		iopage_counts[order]++;
	else
		EFRM_ERR("Huge iopages alloc (order=%d) ??? (not counted)",
			 order);
#endif

	return ptr;
}

void efrm_dma_free_coherent(struct device *dev, size_t size,
			    void *ptr, dma_addr_t dma_addr)
{
	struct pci_dev *pci_dev;
	unsigned order;
	EFRM_IOMMU_DECL;

	order = __ffs(size/PAGE_SIZE);
	EFRM_ASSERT(size == (PAGE_SIZE<<order));

#if EFRM_IOPAGE_COUNTS_ENABLED
	if (order < 8)
		--iopage_counts[order];
	else
		EFRM_ERR("Huge iopages free (order=%d) ??? (not counted)",
			 order);
#endif
#ifndef CONFIG_IA64
	pci_mmap_pages_hack_before_free(ptr, order);
#endif
	if (use_pci_alloc) {
		EFRM_IOMMU_LOCK();
		dma_free_coherent(dev, size, ptr, dma_addr);
		EFRM_IOMMU_UNLOCK();
	} else {
		pci_dev = container_of(dev, struct pci_dev, dev);
		EFRM_IOMMU_LOCK();
		efrm_pci_unmap_single(pci_dev, dma_addr, size,
				      PCI_DMA_BIDIRECTIONAL);
		EFRM_IOMMU_UNLOCK();

		free_pages((unsigned long)ptr, order);
	}

#ifndef CONFIG_IA64
	pci_mmap_pages_hack_after_free(ptr, order);
#endif
}
