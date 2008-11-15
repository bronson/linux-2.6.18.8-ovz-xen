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

#include <asm/machdep.h>
#include <asm/time.h>

extern void evtchn_init_IRQ(void);
extern void xen_init_IRQ(void);
extern void xen_reboot_init(struct machdep_calls *);
extern void xen_maple_init_IRQ(void);
extern unsigned int xen_get_irq(struct pt_regs *regs);

static inline u64 tb_to_ns(u64 tb)
{
	if (likely(tb_ticks_per_sec)) {
		return tb * (1000000000UL / tb_ticks_per_sec);
	}
	return 0;
}

static inline u64 jiffies_to_ns(unsigned long j) 
{
	return j * (1000000000UL / HZ);
}

extern struct page *alloc_foreign_page(void);
extern void free_foreign_page(struct page *page);

extern void __init xen_setup_time(struct machdep_calls *host_md);
extern void xen_setup_smp(void);
