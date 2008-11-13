#ifndef __LINUX_PERCPU_H
#define __LINUX_PERCPU_H
#include <linux/spinlock.h> /* For preempt_disable() */
#include <linux/slab.h> /* For kmalloc() */
#include <linux/smp.h>
#include <linux/string.h> /* For memset() */
#include <asm/percpu.h>

/* Enough to cover all DEFINE_PER_CPUs in kernel, including modules. */
#ifndef PERCPU_ENOUGH_ROOM
#define PERCPU_ENOUGH_ROOM 32768
#endif

/* Must be an lvalue. */
#define get_cpu_var(var) (*({ preempt_disable(); &__get_cpu_var(var); }))
#define put_cpu_var(var) preempt_enable()

#ifdef CONFIG_SMP

struct percpu_data {
	void *ptrs[NR_CPUS];
};

/* 
 * Use this to get to a cpu's version of the per-cpu object allocated using
 * alloc_percpu.  Non-atomic access to the current CPU's version should
 * probably be combined with get_cpu()/put_cpu().
 */ 
#define per_cpu_ptr(ptr, cpu)                   \
({                                              \
        struct percpu_data *__p = (struct percpu_data *)~(unsigned long)(ptr); \
        (__typeof__(ptr))__p->ptrs[(cpu)];	\
})

#define static_percpu_ptr(sptr, sptrs) ({		\
		int i;					\
		for (i = 0; i < NR_CPUS; i++)		\
			(sptr)->ptrs[i] = &(sptrs)[i];	\
		((void *)(~(unsigned long)(sptr)));	\
	})

extern void *__alloc_percpu_mask(size_t size, gfp_t gfp);
extern void free_percpu(const void *);

#else /* CONFIG_SMP */

#define per_cpu_ptr(ptr, cpu) ({ (void)(cpu); (ptr); })

#define static_percpu_ptr(sptr, sptrs)	(&sptrs[0])

static inline void *__alloc_percpu_mask(size_t size, gfp_t gfp)
{
	void *ret = kmalloc(size, gfp);
	if (ret)
		memset(ret, 0, size);
	return ret;
}
static inline void free_percpu(const void *ptr)
{	
	kfree(ptr);
}

#endif /* CONFIG_SMP */

/* Simple wrapper for the common case: zeros memory. */
#define __alloc_percpu(size)		\
	__alloc_percpu_mask((size), GFP_KERNEL)
#define alloc_percpu(type)		\
	((type *)(__alloc_percpu_mask(sizeof(type), GFP_KERNEL)))
#define alloc_percpu_atomic(type)	\
	((type *)(__alloc_percpu_mask(sizeof(type), GFP_ATOMIC)))

#endif /* __LINUX_PERCPU_H */
