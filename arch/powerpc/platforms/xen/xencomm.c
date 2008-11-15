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
 * Authors: Hollis Blanchard <hollisb@us.ibm.com>
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/page.h>
#include <asm/current.h>
#include <xen/interface/arch-powerpc.h>
#include <xen/xencomm.h>

/* translate virtual address to physical address */
unsigned long xencomm_vtop(unsigned long vaddr)
{
	struct page *page;
	struct vm_area_struct *vma;

	/* NULL is NULL */
	if (vaddr == 0)
		return 0;

	if (is_kernel_addr(vaddr))
		return __pa(vaddr);

	/* XXX double-check (lack of) locking */
	vma = find_extend_vma(current->mm, vaddr);
	BUG_ON(!vma);
	if (!vma)
		return ~0UL;

	page = follow_page(vma, vaddr, 0);
	BUG_ON(!page);
	if (!page)
		return ~0UL;

	return (page_to_pfn(page) << PAGE_SHIFT) | (vaddr & ~PAGE_MASK);
}
