/*
 * Copyright (C) 2006 Hollis Blanchard <hollisb@us.ibm.com>, IBM Corporation
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef _ASM_IA64_XENCOMM_H_
#define _ASM_IA64_XENCOMM_H_

#define is_kernel_addr(x)					\
	((PAGE_OFFSET <= (x) &&					\
	  (x) < (PAGE_OFFSET + (1UL << IA64_MAX_PHYS_BITS))) ||	\
	 (KERNEL_START <= (x) &&				\
	  (x) < KERNEL_START + KERNEL_TR_PAGE_SIZE))

/* Must be called before any hypercall.  */
extern void xencomm_initialize (void);

#include <xen/xencomm.h>

#endif /* _ASM_IA64_XENCOMM_H_ */
