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

#include <linux/config.h>
#include <linux/vmalloc.h>
#include <linux/memory_hotplug.h>
#include <xen/gnttab.h>
#include <asm/hypervisor.h>
#include <xen/interface/grant_table.h>
#include <asm/pgtable.h>
#include <asm/sections.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/cacheflush.h>
#include "setup.h"
#include "../pseries/plpar_wrappers.h"

#undef DEBUG

#ifdef DEBUG
#define DBG(fmt...) printk(KERN_EMERG fmt)
#else
#define DBG(fmt...)
#endif

#define NR_GRANT_FRAMES 4

struct address_space xen_foreign_dummy_mapping;

static ulong foreign_map_pfn;
static ulong foreign_map_pgs;
static unsigned long *foreign_map_bitmap;


/* hijack _mapcount */
static inline int gnt_mapcount(struct page *page)
{
	return atomic_read(&(page)->_mapcount) + 1;
}

static inline int gnt_map(struct page *page)
{
	/* return true is transition from -1 to 0 */
	return atomic_inc_and_test(&page->_mapcount);
}

static inline int gnt_unmap(struct page *page)
{
	int val;

	val = atomic_dec_return(&page->_mapcount);
	if (val < -1) {
		atomic_inc(&page->_mapcount);
		printk(KERN_EMERG "%s: %d\n", __func__, val);
	}

	return (val == -1);
}


static long map_to_linear(ulong paddr)
{
	unsigned long vaddr;
	int psize;
	unsigned long mode;
	int slot;
	uint shift;
	unsigned long tmp_mode;

	psize = MMU_PAGE_4K;
	shift = mmu_psize_defs[psize].shift;
	mode = _PAGE_ACCESSED | _PAGE_DIRTY | _PAGE_COHERENT | PP_RWXX;
	vaddr = (ulong)__va(paddr);

	{
		unsigned long vpn, hash, hpteg;
		unsigned long vsid = get_kernel_vsid(vaddr);
		unsigned long va = (vsid << 28) | (vaddr & 0x0fffffff);

		vpn = va >> shift;
		tmp_mode = mode;
		
		/* Make non-kernel text non-executable */
		if (!in_kernel_text(vaddr))
			tmp_mode = mode | HPTE_R_N;

		hash = hpt_hash(va, shift);
		hpteg = ((hash & htab_hash_mask) * HPTES_PER_GROUP);

		BUG_ON(!ppc_md.hpte_insert);
		slot = ppc_md.hpte_insert(hpteg, va, paddr,
					  tmp_mode, HPTE_V_BOLTED, psize);
		if (slot < 0)
			printk(KERN_EMERG
			       "%s: no more bolted entries "
			       "HTAB[0x%lx]: 0x%lx\n",
			       __func__, hpteg, paddr);
	}
	return slot;
}

static unsigned long get_hpte_vsid(ulong slot)
{
 	unsigned long dword0;
	unsigned long lpar_rc;
	unsigned long dummy_word1;
	unsigned long flags;

	/* Read 1 pte at a time                        */
	/* Do not need RPN to logical page translation */
	/* No cross CEC PFT access                     */
	flags = 0;

	lpar_rc = plpar_pte_read(flags, slot, &dword0, &dummy_word1);

	BUG_ON(lpar_rc != H_SUCCESS);

	return dword0;
}

static long find_hpte_slot(unsigned long va, int psize)
{
	unsigned long hash;
	unsigned long i, j;
	long slot;
	unsigned long want_v, hpte_v;

	hash = hpt_hash(va, mmu_psize_defs[psize].shift);
	want_v = hpte_encode_v(va, psize);

	for (j = 0; j < 2; j++) {
		slot = (hash & htab_hash_mask) * HPTES_PER_GROUP;
		for (i = 0; i < HPTES_PER_GROUP; i++) {
			hpte_v = get_hpte_vsid(slot);

			if (HPTE_V_COMPARE(hpte_v, want_v)
			    && (hpte_v & HPTE_V_VALID)
			    && (!!(hpte_v & HPTE_V_SECONDARY) == j)) {
				/* HPTE matches */
				if (j)
					slot = -slot;
				return slot;
			}
			++slot;
		}
		hash = ~hash;
	}

	return -1;
} 

static long find_map_slot(ulong ea)
{
	int psize = MMU_PAGE_4K;
	ulong vsid;
	ulong va;

	vsid = get_kernel_vsid(ea);
	va = (vsid << 28) | (ea & 0x0fffffff);
	
	return find_hpte_slot(va, psize);
}


static void gnttab_pre_unmap_grant_ref(
	struct gnttab_unmap_grant_ref *unmap, int count)
{
	long slot;
	int i;
	ulong ea;
	unsigned long dummy1, dummy2;
	ulong flags;

	/* paranoia */
	local_irq_save(flags);

	for (i = 0 ; i < count; i++) {
		struct page *page;

		ea = (ulong)__va(unmap[i].host_addr);
		page = virt_to_page(ea);
		
		if (!gnt_unmap(page)) {
			DBG("%s[0x%x]: skip: 0x%lx, mapcount 0x%x\n",
			    __func__, i, ea, gnt_mapcount(page));
			continue;
		}
		slot = find_map_slot(ea);
		if (slot < 0) {
			printk(KERN_EMERG "%s: PTE not found: 0x%lx\n",
			       __func__, ea);
			continue;
		}

		DBG("%s[0x%x]: 0x%lx: mapcount: 0x%x\n",
		    __func__, i, ea, gnt_mapcount(page));
		plpar_pte_remove(0, slot, 0, &dummy1, &dummy2);
	}
	local_irq_restore(flags);
}

static void gnttab_post_map_grant_ref(
	struct gnttab_map_grant_ref *map, int count)
{
	int i;
	long slot;
	ulong flags;

	/* paranoia */
	local_irq_save(flags);

	for (i = 0 ; i < count; i++) {
		ulong pa = map[i].host_addr;
		struct page *page;

		if (map[i].status != GNTST_okay) {
			printk(KERN_EMERG "%s: status, skip\n", __func__);
			continue;
		}

		BUG_ON(pa < (foreign_map_pfn << PAGE_SHIFT));
		BUG_ON(pa >= (foreign_map_pfn << PAGE_SHIFT) + 
		       (foreign_map_pgs << PAGE_SHIFT));

		page = virt_to_page(__va(pa));

		if (gnt_map(page)) {
#ifdef DEBUG			
			/* we need to get smarted than this */
			slot = find_map_slot((ulong)__va(pa));
			if (slot >= 0) {
				DBG("%s: redundant 0x%lx\n", __func__, pa);
				continue;
			}
#endif
			slot = map_to_linear(pa);
			DBG("%s[0x%x]: 0x%lx, mapcount:0x%x\n",
			    __func__, i, pa, gnt_mapcount(page));

		} else {
			DBG("%s[0x%x] skip 0x%lx, mapcount:0x%x\n",
			    __func__, i, pa, gnt_mapcount(page));
		}
	}
	local_irq_restore(flags);
}

int HYPERVISOR_grant_table_op(unsigned int cmd, void *op, unsigned int count)
{
	void *desc;
	void *frame_list = NULL;
	int argsize;
	int ret = -ENOMEM;

	switch (cmd) {
	case GNTTABOP_map_grant_ref:
		argsize = sizeof(struct gnttab_map_grant_ref);
		break;
	case GNTTABOP_unmap_grant_ref:
		gnttab_pre_unmap_grant_ref(op, count);
		argsize = sizeof(struct gnttab_unmap_grant_ref);
		break;
	case GNTTABOP_setup_table: {
		struct gnttab_setup_table setup;

		memcpy(&setup, op, sizeof(setup));
		argsize = sizeof(setup);

		frame_list = xencomm_map(
			xen_guest_handle(setup.frame_list),
			(sizeof(*xen_guest_handle(setup.frame_list)) 
			* setup.nr_frames));

		if (frame_list == NULL)
			return -ENOMEM;

		set_xen_guest_handle(setup.frame_list, frame_list);
		memcpy(op, &setup, sizeof(setup));
		}
		break;
	case GNTTABOP_dump_table:
		argsize = sizeof(struct gnttab_dump_table);
		break;
	case GNTTABOP_transfer:
		BUG();
		argsize = sizeof(struct gnttab_transfer);
		break;
	case GNTTABOP_copy:
		argsize = sizeof(struct gnttab_transfer);
		break;
	case GNTTABOP_query_size:
		argsize = sizeof(struct gnttab_query_size);
		break;
	default:
		printk(KERN_EMERG "%s: unknown grant table op %d\n",
		       __func__, cmd);
		return -ENOSYS;
	}

	desc = xencomm_map_no_alloc(op, argsize);
	if (desc) {
		ret = plpar_hcall_norets(XEN_MARK(__HYPERVISOR_grant_table_op),
					 cmd, desc, count);
		if (!ret && cmd == GNTTABOP_map_grant_ref)
			gnttab_post_map_grant_ref(op, count);
		xencomm_free(desc);
	}
	xencomm_free(frame_list);

	return ret;
}
EXPORT_SYMBOL(HYPERVISOR_grant_table_op);

static ulong find_grant_maps(void)
{
	struct device_node *xen;
	u64 *gm;
	u64 _gm[2];
	u64 expect;

	/* This value is currently hardcoded into the SLB logic that
	 * it written in assempler, See
	 * slb_miss_kernel_load_xen_linear for more information.
	 * Anything else and we can not run. */
	expect = 34 - PAGE_SHIFT;

	xen = of_find_node_by_path("/xen");

	/* 
	 * The foreign is 2x2 Cells.
	 * The first entry is log2 of the base page frame.
	 * The second is the number of pages
	 */
	gm = (u64 *)get_property(xen, "foreign-map", NULL);
	if (gm == NULL) {
		if (!is_initial_xendomain()) {
			printk("OF: /xen/foreign-map not present\n");
			_gm[0] = expect;
			_gm[1] = 2048;
			gm = _gm;
		} else
			panic("OF: /xen/foreign-map must be present\n");
	}

	if (gm[0] != expect)
		panic("foreign-map is 0x%lx, expect 0x%lx\n",
		      gm[0], expect);

	foreign_map_pfn = 1UL << gm[0];
	return gm[1];
}

static void setup_foreign_segment(void)
{
	extern int *slb_miss_kernel_load_xen_nop;
	ulong iaddr = (ulong)slb_miss_kernel_load_xen_nop;

	/* By default Linux will branch around this logic we replace
	 * the branch with a NOP to turn the logic on */
	*slb_miss_kernel_load_xen_nop = 0x60000000;
	flush_icache_range(iaddr, iaddr + 4);
}

struct page *alloc_foreign_page(void)
{
	ulong bit;
	do {
		bit = find_first_zero_bit(foreign_map_bitmap,
					  foreign_map_pgs);
		if (bit >= foreign_map_pgs)
			return NULL;
	} while (test_and_set_bit(bit, foreign_map_bitmap) == 1);

	return pfn_to_page(foreign_map_pfn + bit);
}

void free_foreign_page(struct page *page)
{
	ulong bit = page_to_pfn(page) - foreign_map_pfn;

	BUG_ON(bit >= foreign_map_pgs);
	BUG_ON(!test_bit(bit, foreign_map_bitmap));

	clear_bit(bit, foreign_map_bitmap);
}

static void setup_grant_area(void)
{
	ulong pgs;
	int err;
	struct zone *zone;
	struct pglist_data *pgdata;
	int nid;

	pgs = find_grant_maps();
	setup_foreign_segment();

	printk("%s: Xen VIO will use a foreign address space of 0x%lx pages\n",
	       __func__, pgs);

	/* add pages to the zone */
	nid = 0;
	pgdata = NODE_DATA(nid);
	zone = pgdata->node_zones;

	err = __add_pages(zone, foreign_map_pfn, pgs);

	if (err < 0) {
		printk(KERN_EMERG "%s: add_pages(0x%lx, 0x%lx) = %d\n",
		       __func__, foreign_map_pfn, pgs, err);
		BUG();
	}

	/* create a bitmap to manage these pages */
	foreign_map_bitmap = kmalloc(BITS_TO_LONGS(pgs) * sizeof(long),
				     GFP_KERNEL);
	if (foreign_map_bitmap == NULL) {
		printk(KERN_EMERG 
		       "%s: could not allocate foreign_map_bitmap to "
		       "manage 0x%lx foreign pages\n", __func__, pgs);
		BUG();
	}
	/* I'm paranoid so make sure we assign the top bits so we
	 * don't give them away */
	bitmap_fill(&foreign_map_bitmap[BITS_TO_LONGS(pgs) - 1],
		    BITS_PER_LONG);
	/* now clear all the real bits */
	bitmap_zero(foreign_map_bitmap, pgs);

	foreign_map_pgs = pgs;
}

void *arch_gnttab_alloc_shared(unsigned long *frames)
{
	void *shared;
	ulong pa = frames[0] << PAGE_SHIFT;
	static int resume;

	shared = ioremap(pa, PAGE_SIZE * NR_GRANT_FRAMES);
	BUG_ON(shared == NULL);
	printk("%s: grant table at %p\n", __func__, shared);

	/* no need to do the rest of this if we are resuming */
	if (!resume)
		setup_grant_area();

	resume = 1;

	return shared;
}
