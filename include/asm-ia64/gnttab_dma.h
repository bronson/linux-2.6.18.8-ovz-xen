/*
 * Copyright (c) 2007 Herbert Xu <herbert@gondor.apana.org.au>
 * Copyright (c) 2007 Isaku Yamahata <yamahata at valinux co jp>
 *                    VA Linux Systems Japan K.K.
 *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _ASM_IA64_GNTTAB_DMA_H
#define _ASM_IA64_GNTTAB_DMA_H

static inline int gnttab_dma_local_pfn(struct page *page)
{
	return 0;
}

/* caller must get dma address after calling this function */
static inline void gnttab_dma_use_page(struct page *page)
{
	__gnttab_dma_map_page(page);
}

static inline dma_addr_t gnttab_dma_map_page(struct page *page)
{
	gnttab_dma_use_page(page);
	return page_to_bus(page);
}

static inline dma_addr_t gnttab_dma_map_virt(void *ptr)
{
	return gnttab_dma_map_page(virt_to_page(ptr)) + offset_in_page(ptr);
}

static inline void gnttab_dma_unmap_page(dma_addr_t dma_address)
{
	__gnttab_dma_unmap_page(virt_to_page(bus_to_virt(dma_address)));
}

#endif /* _ASM_IA64_GNTTAB_DMA_H */
