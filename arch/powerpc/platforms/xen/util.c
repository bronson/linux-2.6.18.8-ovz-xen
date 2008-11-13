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
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <xen/driver_util.h>
#include "setup.h"

struct vm_struct *alloc_vm_area(unsigned long size)
{
	struct vm_struct *area;
	struct page *page;

	page = alloc_foreign_page();
	if (page == NULL) {
		BUG();
		return NULL;
	}

	area = kmalloc(sizeof(*area), GFP_KERNEL);
	if (area != NULL) {
		area->flags = VM_MAP;//XXX
		area->addr = pfn_to_kaddr(page_to_pfn(page));
		area->size = size;
		area->pages = NULL; //XXX
		area->nr_pages = size >> PAGE_SHIFT;
		area->phys_addr = 0;
	}
	return area;
}
EXPORT_SYMBOL_GPL(alloc_vm_area);

void free_vm_area(struct vm_struct *area)
{
	free_foreign_page(virt_to_page(area->addr));
	kfree(area);
}
EXPORT_SYMBOL_GPL(free_vm_area);

void lock_vm_area(struct vm_struct *area)
{
	preempt_disable();
}

void unlock_vm_area(struct vm_struct *area)
{
	preempt_enable();
}
EXPORT_SYMBOL_GPL(unlock_vm_area);
