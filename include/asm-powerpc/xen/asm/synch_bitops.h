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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Copyright 2006 IBM Corp.
 *
 * Authors: Jimi Xenidis <jimix@watson.ibm.com>
 */

#ifndef  __SYNCH_BITOPS_H__
#define __SYNCH_BITOPS_H__

#include <linux/config.h>
#include <xen/interface/xen.h>

#ifdef CONFIG_SMP
#include <asm/bitops.h>

#define synch_change_bit(a,b) change_bit(a,b)
#define synch_clear_bit(a,b) clear_bit(a,b)
#define synch_const_test_bit(a,b) const_test_bit(a,b) 
#define synch_set_bit(a,b) set_bit(a,b)
#define synch_test_and_set_bit(a,b) test_and_set_bit(a,b)
#define synch_test_and_change_bit(a,b) test_and_change_bit(a,b)
#define synch_test_and_clear_bit(a,b) test_and_clear_bit(a,b)
#define synch_test_bit(a,b) test_bit(a,b)

static __inline__ unsigned long
__synch_cmpxchg_u16(volatile unsigned short *p, unsigned long old, unsigned long new)
{
	int idx;
	volatile unsigned int *xp = (unsigned int *)((ulong)p & ~(0x3UL));
	union {
		unsigned int word;
		struct {
			unsigned short s[2];
		}s;
	} xold, xnew;

	/* we could start the reservation here and copy the u32
	 * assembler, but I don't think it will gain us a whole
	 * lot. */
	xold.word = *xp;
	xnew.word = xold.word;
	idx = ((ulong)p >> 1) & 0x1;
	xold.s.s[idx] = old;
	xnew.s.s[idx] = new;

	return __cmpxchg_u32(xp, xold.word, xnew.word);
}

/*
 * This function doesn't exist, so you'll get a linker error
 * if something tries to do an invalid xchg().
 */
extern void __synch_cmpxchg_called_with_bad_pointer(void);
static __inline__ unsigned long
__synch_cmpxchg(volatile void *ptr, unsigned long old, unsigned long new,
	       unsigned int size)
{
	switch (size) {
	case 2:
		return __synch_cmpxchg_u16(ptr, old, new);
	case 4:
		return __cmpxchg_u32(ptr, old, new);
#ifdef CONFIG_PPC64
	case 8:
		return __cmpxchg_u64(ptr, old, new);
#endif
	}
	__synch_cmpxchg_called_with_bad_pointer();
	return old;
}

#define synch_cmpxchg(ptr,o,n)						 \
  ({									 \
     __typeof__(*(ptr)) _o_ = (o);					 \
     __typeof__(*(ptr)) _n_ = (n);					 \
     (__typeof__(*(ptr))) __synch_cmpxchg((ptr), (unsigned long)_o_,		 \
				    (unsigned long)_n_, sizeof(*(ptr))); \
  })

#define synch_cmpxchg_subword(ptr,o,n) __synch_cmpxchg_u16((ptr), (o), (n))

#else
#error "this only works for CONFIG_SMP"
#endif

#endif /* __SYNCH_BITOPS_H__ */
