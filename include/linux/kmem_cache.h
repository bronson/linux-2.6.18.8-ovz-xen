#ifndef __KMEM_CACHE_H__
#define __KMEM_CACHE_H__
#include <linux/threads.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <asm/atomic.h>

/*
 * DEBUG	- 1 for kmem_cache_create() to honour; SLAB_DEBUG_INITIAL,
 *		  SLAB_RED_ZONE & SLAB_POISON.
 *		  0 for faster, smaller code (especially in the critical paths).
 *
 * STATS	- 1 to collect stats for /proc/slabinfo.
 *		  0 for faster, smaller code (especially in the critical paths).
 *
 * FORCED_DEBUG	- 1 enables SLAB_RED_ZONE and SLAB_POISON (if possible)
 */

#ifdef CONFIG_DEBUG_SLAB
#define	SLAB_DEBUG		1
#define	SLAB_STATS		1
#define	SLAB_FORCED_DEBUG	1
#else
#define	SLAB_DEBUG		0
#define	SLAB_STATS		0
#define	SLAB_FORCED_DEBUG	0
#endif

/*
 * struct array_cache
 *
 * Purpose:
 * - LIFO ordering, to hand out cache-warm objects from _alloc
 * - reduce the number of linked list operations
 * - reduce spinlock operations
 *
 * The limit is stored in the per-cpu structure to reduce the data cache
 * footprint.
 *
 */
struct array_cache {
	unsigned int avail;
	unsigned int limit;
	unsigned int batchcount;
	unsigned int touched;
	spinlock_t lock;
	void *entry[0];		/*
				 * Must have this definition in here for the proper
				 * alignment of array_cache. Also simplifies accessing
				 * the entries.
				 * [0] is for gcc 2.95. It should really be [].
				 */
};

/* bootstrap: The caches do not work without cpuarrays anymore,
 * but the cpuarrays are allocated from the generic caches...
 */
#define BOOT_CPUCACHE_ENTRIES	1
struct arraycache_init {
	struct array_cache cache;
	void *entries[BOOT_CPUCACHE_ENTRIES];
};

/*
 * The slab lists for all objects.
 */
struct kmem_list3 {
	struct list_head slabs_partial;	/* partial list first, better asm code */
	struct list_head slabs_full;
	struct list_head slabs_free;
	unsigned long free_objects;
	unsigned int free_limit;
	unsigned int colour_next;	/* Per-node cache coloring */
	spinlock_t list_lock;
	struct array_cache *shared;	/* shared per node */
	struct array_cache **alien;	/* on other nodes */
	unsigned long next_reap;	/* updated without locking */
	int free_touched;		/* updated without locking */
};

/*
 * struct kmem_cache
 *
 * manages a cache.
 */

struct kmem_cache {
/* 1) per-cpu data, touched during every alloc/free */
	struct array_cache *array[NR_CPUS];
/* 2) Cache tunables. Protected by cache_chain_mutex */
	unsigned int batchcount;
	unsigned int limit;
	unsigned int shared;

	unsigned int buffer_size;
/* 3) touched by every alloc & free from the backend */
	struct kmem_list3 *nodelists[MAX_NUMNODES];

	unsigned int flags;		/* constant flags */
	unsigned int num;		/* # of objs per slab */

/* 4) cache_grow/shrink */
	/* order of pgs per slab (2^n) */
	unsigned int gfporder;

	/* force GFP flags, e.g. GFP_DMA */
	gfp_t gfpflags;

	size_t colour;			/* cache colouring range */
	unsigned int colour_off;	/* colour offset */
	struct kmem_cache *slabp_cache;
	unsigned int slab_size;
	unsigned int dflags;		/* dynamic flags */

	/* constructor func */
	void (*ctor) (void *, struct kmem_cache *, unsigned long);

	/* de-constructor func */
	void (*dtor) (void *, struct kmem_cache *, unsigned long);

/* 5) cache creation/removal */
	const char *name;
	struct list_head next;

/* 6) statistics */
	unsigned long grown;
	unsigned long reaped;
	unsigned long shrunk;
#if SLAB_STATS
	unsigned long num_active;
	unsigned long num_allocations;
	unsigned long high_mark;
	unsigned long errors;
	unsigned long max_freeable;
	unsigned long node_allocs;
	unsigned long node_frees;
	unsigned long node_overflow;
	atomic_t allochit;
	atomic_t allocmiss;
	atomic_t freehit;
	atomic_t freemiss;
#endif
#if SLAB_DEBUG
	/*
	 * If debugging is enabled, then the allocator can add additional
	 * fields and/or padding to every object. buffer_size contains the total
	 * object size including these internal fields, the following two
	 * variables contain the offset to the user object and its size.
	 */
	int obj_offset;
	int obj_size;
#endif
#ifdef CONFIG_USER_RESOURCE
	unsigned int		objuse;
#endif
};

#define CFLGS_OFF_SLAB		(0x80000000UL)
#define CFLGS_ENVIDS		(0x04000000UL)
#define OFF_SLAB(x)		((x)->flags & CFLGS_OFF_SLAB)
#define ENVIDS(x)		((x)->flags & CFLGS_ENVIDS)
#define kmem_mark_nocharge(c)	do { (c)->flags |= SLAB_NO_CHARGE; } while (0)

struct slab;
/*
 * Functions for storing/retrieving the cachep and or slab from the page
 * allocator.  These are used to find the slab an obj belongs to.  With kfree(),
 * these are used to find the cache which an obj belongs to.
 */
static inline void page_set_cache(struct page *page, struct kmem_cache *cache)
{
	page->lru.next = (struct list_head *)cache;
}

static inline struct kmem_cache *page_get_cache(struct page *page)
{
	if (unlikely(PageCompound(page)))
		page = (struct page *)page_private(page);
	BUG_ON(!PageSlab(page));
	return (struct kmem_cache *)page->lru.next;
}

static inline void page_set_slab(struct page *page, struct slab *slab)
{
	page->lru.prev = (struct list_head *)slab;
}

static inline struct slab *page_get_slab(struct page *page)
{
	if (unlikely(PageCompound(page)))
		page = (struct page *)page_private(page);
	BUG_ON(!PageSlab(page));
	return (struct slab *)page->lru.prev;
}

static inline struct kmem_cache *virt_to_cache(const void *obj)
{
	struct page *page = virt_to_page(obj);
	return page_get_cache(page);
}

static inline struct slab *virt_to_slab(const void *obj)
{
	struct page *page = virt_to_page(obj);
	return page_get_slab(page);
}

#include <linux/kmem_slab.h>

static inline void *index_to_obj(struct kmem_cache *cache, struct slab *slab,
				 unsigned int idx)
{
	return slab->s_mem + cache->buffer_size * idx;
}

static inline unsigned int obj_to_index(struct kmem_cache *cache,
					struct slab *slab, void *obj)
{
	return (unsigned)(obj - slab->s_mem) / cache->buffer_size;
}

#endif
