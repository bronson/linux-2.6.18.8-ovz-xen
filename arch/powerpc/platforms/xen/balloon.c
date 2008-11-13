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

#include <linux/module.h>
#include <linux/mm.h>
#include <asm/hypervisor.h>
#include "setup.h"

/*
 * FIXME: Port balloon driver, if ever
 */

struct page **alloc_empty_pages_and_pagevec(int nr_pages)
{
	struct page *page, **pagevec;
	int i;

	pagevec = kmalloc(sizeof(*pagevec) * nr_pages, GFP_KERNEL);
	if (pagevec == NULL)
		return  NULL;

	for (i = 0; i < nr_pages; i++) {
		page = alloc_foreign_page();
		BUG_ON(page == NULL);
		pagevec[i] = page;
		/* There is no real page backing us yet so it cannot
		 * be scrubbed */
	}

	return pagevec;
}

void free_empty_pages_and_pagevec(struct page **pagevec, int nr_pages)
{
	int i;

	if (pagevec == NULL)
		return;

	for (i = 0; i < nr_pages; i++) {
		free_foreign_page(pagevec[i]);
	}
	
	kfree(pagevec);
}

void balloon_dealloc_empty_page_range(
	struct page *page, unsigned long nr_pages)
{
	__free_pages(page, get_order(nr_pages * PAGE_SIZE));
}

void balloon_update_driver_allowance(long delta)
{
}

void balloon_release_driver_page(struct page *page)
{
	BUG();
}

EXPORT_SYMBOL_GPL(balloon_update_driver_allowance);
EXPORT_SYMBOL_GPL(alloc_empty_pages_and_pagevec);
EXPORT_SYMBOL_GPL(free_empty_pages_and_pagevec);
EXPORT_SYMBOL_GPL(balloon_release_driver_page);
