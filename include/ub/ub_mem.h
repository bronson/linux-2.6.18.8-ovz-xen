/*
 *  include/ub/ub_mem.h
 *
 *  Copyright (C) 2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#ifndef __UB_SLAB_H_
#define __UB_SLAB_H_

#include <linux/config.h>
#include <linux/kmem_slab.h>
#include <ub/beancounter.h>
#include <ub/ub_decl.h>

/*
 * UB_KMEMSIZE accounting
 */

#ifdef CONFIG_UBC_DEBUG_ITEMS
#define CHARGE_ORDER(__o)		(1 << (__o))
#define CHARGE_SIZE(__s)		1
#else
#define CHARGE_ORDER(__o)		(PAGE_SIZE << (__o))
#define CHARGE_SIZE(__s)		(__s)
#endif

#define page_ub(__page)	((__page)->bc.page_ub)

struct mm_struct;
struct page;
struct kmem_cache;

UB_DECLARE_FUNC(struct user_beancounter *, slab_ub(void *obj))
UB_DECLARE_FUNC(struct user_beancounter *, vmalloc_ub(void *obj))
UB_DECLARE_FUNC(struct user_beancounter *, mem_ub(void *obj))

UB_DECLARE_FUNC(int, ub_kmemsize_charge(struct user_beancounter *ub,
		unsigned long size, enum ub_severity strict))
UB_DECLARE_VOID_FUNC(ub_kmemsize_uncharge(struct user_beancounter *ub,
		unsigned long size))

UB_DECLARE_FUNC(int, ub_page_charge(struct page *page, int order, gfp_t mask))
UB_DECLARE_VOID_FUNC(ub_page_uncharge(struct page *page, int order))
UB_DECLARE_FUNC(int, ub_slab_charge(struct kmem_cache *cachep,
			void *objp, gfp_t flags))
UB_DECLARE_VOID_FUNC(ub_slab_uncharge(struct kmem_cache *cachep, void *obj))

#define slab_ubcs(cachep, slabp) ((struct user_beancounter **)\
		(ALIGN((unsigned long)(slab_bufctl(slabp) + (cachep)->num),\
		       sizeof(void *))))

#ifdef CONFIG_USER_RESOURCE
extern struct user_beancounter *ub_select_worst(long *);

/* mm/slab.c needed stuff */
#define UB_ALIGN(flags)		(flags & SLAB_UBC ? sizeof(void *) : 1)
#define UB_EXTRA(flags)		(flags & SLAB_UBC ? sizeof(void *) : 0)
#define set_cache_objuse(cachep)	do {				\
		(cachep)->objuse = ((PAGE_SIZE << (cachep)->gfporder) +	\
				(cachep)->num - 1) / (cachep)->num;	\
		if (!OFF_SLAB(cachep))					\
			break;						\
		(cachep)->objuse += ((cachep)->slabp_cache->objuse +	\
				(cachep)->num - 1) / (cachep)->num;	\
	} while (0)
#define init_slab_ubps(cachep, slabp)	do {				\
		if (!((cachep)->flags & SLAB_UBC))			\
			break;						\
		memset(slab_ubcs(cachep, slabp), 0,			\
				(cachep)->num * sizeof(void *));	\
	} while (0)
#define kmem_obj_memusage(o)	(virt_to_cache(o)->objuse)
#else
#define UB_ALIGN(flags)		1
#define UB_EXTRA(flags)		0
#define set_cache_objuse(c)	do { } while (0)
#define init_slab_ubps(c, s)	do { } while (0)
#endif
#endif /* __UB_SLAB_H_ */
