/*
 * Generic pidhash and scalable, time-bounded PID allocator
 *
 * (C) 2002-2003 William Irwin, IBM
 * (C) 2004 William Irwin, Oracle
 * (C) 2002-2004 Ingo Molnar, Red Hat
 *
 * pid-structures are backing objects for tasks sharing a given ID to chain
 * against. There is very little to them aside from hashing them and
 * parking tasks using given ID's on a list.
 *
 * The hash is always changed with the tasklist_lock write-acquired,
 * and the hash is only accessed with the tasklist_lock at least
 * read-acquired, so there's no additional SMP locking needed here.
 *
 * We have a list of bitmap pages, which bitmaps represent the PID space.
 * Allocating and freeing PIDs is completely lockless. The worst-case
 * allocation scenario when all but one out of 1 million PIDs possible are
 * allocated already: the scanning of 32 list entries and at most PAGE_SIZE
 * bytes. The typical fastpath is a single successful setbit. Freeing is O(1).
 */

#include <linux/mm.h>
#include <linux/module.h>
#include <linux/kmem_cache.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/hash.h>

#include <ub/ub_mem.h>

#ifdef CONFIG_VE
int glob_virt_pids = 1;
EXPORT_SYMBOL(glob_virt_pids);
#endif

#define pid_hashfn(nr) hash_long((unsigned long)nr, pidhash_shift)
static struct hlist_head *pid_hash;
static int pidhash_shift;
static kmem_cache_t *pid_cachep;

int pid_max = PID_MAX_DEFAULT;
int last_pid;

#define RESERVED_PIDS		300

int pid_max_min = RESERVED_PIDS + 1;
int pid_max_max = PID_MAX_LIMIT;

#define PIDMAP_ENTRIES		((PID_MAX_LIMIT + 8*PAGE_SIZE - 1)/PAGE_SIZE/8)
#define BITS_PER_PAGE		(PAGE_SIZE*8)
#define BITS_PER_PAGE_MASK	(BITS_PER_PAGE-1)
#define mk_pid(map, off)	(((map) - pidmap_array)*BITS_PER_PAGE + (off))
#define find_next_offset(map, off)					\
		find_next_zero_bit((map)->page, BITS_PER_PAGE, off)

/*
 * PID-map pages start out as NULL, they get allocated upon
 * first use and are never deallocated. This way a low pid_max
 * value does not cause lots of bitmaps to be allocated, but
 * the scheme scales to up to 4 million PIDs, runtime.
 */
typedef struct pidmap {
	atomic_t nr_free;
	void *page;
} pidmap_t;

#ifdef CONFIG_VE
#define PIDMAP_NRFREE (BITS_PER_PAGE/2)
#else
#define PIDMAP_NRFREE BITS_PER_PAGE
#endif

static pidmap_t pidmap_array[PIDMAP_ENTRIES] =
	 { [ 0 ... PIDMAP_ENTRIES-1 ] = { ATOMIC_INIT(PIDMAP_NRFREE), NULL } };

/*
 * Note: disable interrupts while the pidmap_lock is held as an
 * interrupt might come in and do read_lock(&tasklist_lock).
 *
 * If we don't disable interrupts there is a nasty deadlock between
 * detach_pid()->free_pid() and another cpu that does
 * spin_lock(&pidmap_lock) followed by an interrupt routine that does
 * read_lock(&tasklist_lock);
 *
 * After we clean up the tasklist_lock and know there are no
 * irq handlers that take it we can leave the interrupts enabled.
 * For now it is easier to be safe than to prove it can't happen.
 */
static  __cacheline_aligned_in_smp DEFINE_SPINLOCK(pidmap_lock);

fastcall void free_pidmap(int pid)
{
	pidmap_t *map = pidmap_array + pid / BITS_PER_PAGE;
	int offset = pid & BITS_PER_PAGE_MASK;

	BUG_ON(__is_virtual_pid(pid) || pid == 1);

	if (test_and_clear_bit(offset, map->page) == 0)
		BUG();
	atomic_inc(&map->nr_free);
}
EXPORT_SYMBOL_GPL(free_pidmap);

int alloc_pidmap(void)
{
	int i, offset, max_scan, pid, last = last_pid;
	pidmap_t *map;

	pid = last + 1;
	if (__is_virtual_pid(pid))
		pid += VPID_DIV;
	if (pid >= pid_max)
		pid = RESERVED_PIDS;
	offset = pid & BITS_PER_PAGE_MASK;
	map = &pidmap_array[pid/BITS_PER_PAGE];
	max_scan = (pid_max + BITS_PER_PAGE - 1)/BITS_PER_PAGE - !offset;
	for (i = 0; i <= max_scan; ++i) {
		if (unlikely(!map->page)) {
			unsigned long page = get_zeroed_page(GFP_KERNEL);
			/*
			 * Free the page if someone raced with us
			 * installing it:
			 */
			spin_lock_irq(&pidmap_lock);
			if (map->page)
				free_page(page);
			else
				map->page = (void *)page;
			spin_unlock_irq(&pidmap_lock);
			if (unlikely(!map->page))
				break;
		}
		if (likely(atomic_read(&map->nr_free))) {
			do {
				if (!test_and_set_bit(offset, map->page)) {
					atomic_dec(&map->nr_free);
					last_pid = pid;
					return pid;
				}
				offset = find_next_offset(map, offset);
				if (__is_virtual_pid(offset))
					offset += VPID_DIV;
				pid = mk_pid(map, offset);
			/*
			 * find_next_offset() found a bit, the pid from it
			 * is in-bounds, and if we fell back to the last
			 * bitmap block and the final block was the same
			 * as the starting point, pid is before last_pid.
			 */
			} while (offset < BITS_PER_PAGE && pid < pid_max &&
					(i != max_scan || pid < last ||
					    !((last+1) & BITS_PER_PAGE_MASK)));
		}
		if (map < &pidmap_array[(pid_max-1)/BITS_PER_PAGE]) {
			++map;
			offset = 0;
		} else {
			map = &pidmap_array[0];
			offset = RESERVED_PIDS;
			if (unlikely(last == offset))
				break;
		}
		pid = mk_pid(map, offset);
	}
	return -1;
}
EXPORT_SYMBOL_GPL(alloc_pidmap);

fastcall void put_pid(struct pid *pid)
{
	if (!pid)
		return;
	if ((atomic_read(&pid->count) == 1) ||
	     atomic_dec_and_test(&pid->count))
		kmem_cache_free(pid_cachep, pid);
}

static void delayed_put_pid(struct rcu_head *rhp)
{
	struct pid *pid = container_of(rhp, struct pid, rcu);
	put_pid(pid);
}

fastcall void free_pid(struct pid *pid)
{
	/* We can be called with write_lock_irq(&tasklist_lock) held */
	unsigned long flags;

	spin_lock_irqsave(&pidmap_lock, flags);
	hlist_del_rcu(&pid->pid_chain);
#ifdef CONFIG_VE
	if (pid->veid)
		hlist_del_rcu(&pid->vpid_chain);
#endif
	spin_unlock(&pidmap_lock);
	ub_kmemsize_uncharge(pid->ub, CHARGE_SIZE(pid_cachep->objuse));
	local_irq_restore(flags);

	free_pidmap(pid->nr);
	put_beancounter(pid->ub);
	call_rcu(&pid->rcu, delayed_put_pid);
}
EXPORT_SYMBOL_GPL(free_pid);

struct pid *alloc_pid(void)
{
	struct pid *pid;
	enum pid_type type;
	int nr = -1;
	struct user_beancounter *ub;

	pid = kmem_cache_alloc(pid_cachep, GFP_KERNEL);
	if (!pid)
		goto out;

	nr = alloc_pidmap();
	if (nr < 0)
		goto out_free;

	atomic_set(&pid->count, 1);
	pid->nr = pid->vnr = nr;
	for (type = 0; type < PIDTYPE_MAX; ++type)
		INIT_HLIST_HEAD(&pid->tasks[type]);
#ifdef CONFIG_VE
	pid->vnr = nr;
	pid->veid = 0;
	INIT_HLIST_NODE(&pid->vpid_chain);
#endif
	local_irq_disable();
#ifdef CONFIG_USER_RESOURCE
	ub = get_exec_ub();
	if (ub_kmemsize_charge(ub, CHARGE_SIZE(pid_cachep->objuse), UB_HARD))
		goto out_free_map;

	pid->ub = get_beancounter(ub);
#endif

	spin_lock(&pidmap_lock);
	hlist_add_head_rcu(&pid->pid_chain, &pid_hash[pid_hashfn(pid->nr)]);
	spin_unlock_irq(&pidmap_lock);

out:
	return pid;

#ifdef CONFIG_USER_RESOURCE
out_free_map:
	free_pidmap(nr);
#endif
out_free:
	kmem_cache_free(pid_cachep, pid);
	pid = NULL;
	goto out;
}
EXPORT_SYMBOL_GPL(alloc_pid);

struct pid * fastcall find_pid(int nr)
{
	struct hlist_node *elem;
	struct pid *pid;

	hlist_for_each_entry_rcu(pid, elem,
			&pid_hash[pid_hashfn(nr)], pid_chain) {
		if (pid->nr == nr)
			return pid;
	}
	return NULL;
}
EXPORT_SYMBOL(find_pid);

static struct pid *__lookup_vpid_mapping(int vnr, int veid);

struct pid * fastcall find_vpid(int nr)
{
	return (!is_virtual_pid(nr) ? find_pid(nr) :
			__lookup_vpid_mapping(nr, VEID(get_exec_env())));
}

EXPORT_SYMBOL(find_vpid);

int fastcall attach_pid(struct task_struct *task, enum pid_type type, int nr)
{
	struct pid_link *link;
	struct pid *pid;

	WARN_ON(!task->pid); /* to be removed soon */
	WARN_ON(!nr); /* to be removed soon */

	link = &task->pids[type];
	link->pid = pid = find_pid(nr);
	hlist_add_head_rcu(&link->node, &pid->tasks[type]);

	return 0;
}
EXPORT_SYMBOL_GPL(attach_pid);

void fastcall detach_pid(struct task_struct *task, enum pid_type type)
{
	struct pid_link *link;
	struct pid *pid;
	int tmp;

	link = &task->pids[type];
	pid = link->pid;

	hlist_del_rcu(&link->node);
	link->pid = NULL;

	for (tmp = PIDTYPE_MAX; --tmp >= 0; )
		if (!hlist_empty(&pid->tasks[tmp]))
			return;

	free_pid(pid);
}
EXPORT_SYMBOL_GPL(detach_pid);

struct task_struct * fastcall pid_task(struct pid *pid, enum pid_type type)
{
	struct task_struct *result = NULL;
	if (pid) {
		struct hlist_node *first;
		first = rcu_dereference(pid->tasks[type].first);
		if (first)
			result = hlist_entry(first, struct task_struct, pids[(type)].node);
	}
	return result;
}
EXPORT_SYMBOL(pid_task);

/*
 * Must be called under rcu_read_lock() or with tasklist_lock read-held.
 * This function shouldn't be used, but proprietary ATI video driver uses it.
 */
struct task_struct *find_task_by_pid_type(int type, int nr)
{
	static int warning_first = 1;

	if (warning_first) {
		warning_first = 0;
		printk(KERN_ERR "%s: deprecated find_task_by_pid function is "
			"used. If it is called by the proprietary ATI video driver it is ok, "
			"otherwise please report the case to users@openvz.org\n", current->comm);
		dump_stack();
	}
	return find_task_by_pid_type_ve(type, nr);
}

EXPORT_SYMBOL(find_task_by_pid_type);

struct task_struct *find_task_by_pid_type_all(int type, int nr)
{
	BUG_ON(nr != -1 && is_virtual_pid(nr));
	return pid_task(find_pid(nr), type);
}

EXPORT_SYMBOL(find_task_by_pid_type_all);

#ifdef CONFIG_VE

struct task_struct *find_task_by_pid_type_ve(int type, int nr)
{
	struct task_struct *tsk;
	struct pid *pid;

	pid = find_vpid(nr);
	if (!pid)
		return NULL;

	tsk = pid_task(pid, type);
	return (tsk != NULL && ve_accessible(VE_TASK_INFO(tsk)->owner_env,
			get_exec_env()) ? tsk : NULL);
}

EXPORT_SYMBOL(find_task_by_pid_type_ve);

#endif

struct task_struct *fastcall get_pid_task(struct pid *pid, enum pid_type type)
{
	struct task_struct *result;
	rcu_read_lock();
	result = pid_task(pid, type);
	if (result)
		get_task_struct(result);
	rcu_read_unlock();
	return result;
}

struct pid *find_get_pid(pid_t nr)
{
	struct pid *pid;

	rcu_read_lock();
	pid = get_pid(find_pid(nr));
	rcu_read_unlock();

	return pid;
}

#ifdef CONFIG_VE

/* Virtual PID bits.
 *
 * At the moment all internal structures in kernel store real global pid.
 * The only place, where virtual PID is used, is at user frontend. We
 * remap virtual pids obtained from user to global ones (vpid_to_pid) and
 * map globals to virtuals before showing them to user (virt_pid_type).
 *
 * We hold virtual PIDs inside struct pid, so map global -> virtual is easy.
 */

pid_t _pid_to_vpid(pid_t pid)
{
	struct pid * p;

	if (unlikely(is_virtual_pid(pid)))
		return -1;

	rcu_read_lock();
	p = find_pid(pid);
	pid = (p != NULL ? p->vnr : -1);
	rcu_read_unlock();
	return pid;
}
EXPORT_SYMBOL_GPL(_pid_to_vpid);

pid_t pid_to_vpid(pid_t pid)
{
	int vpid;

	if (unlikely(pid <= 0))
		return pid;

	BUG_ON(is_virtual_pid(pid));

	if (ve_is_super(get_exec_env()))
		return pid;

	vpid = _pid_to_vpid(pid);
	if (unlikely(vpid == -1))
		/* It is allowed: global pid can be used everywhere.
		 * This can happen, when kernel remembers stray pids:
		 * signal queues, locks etc.
		 */
		vpid = pid;

	return vpid;
}
EXPORT_SYMBOL_GPL(pid_to_vpid);

/* To map virtual pids to global we maintain special hash table.
 *
 * Mapping entries are allocated when a process with non-trivial
 * mapping is forked, which is possible only after VE migrated.
 * Mappings are destroyed, when a global pid is removed from global
 * pidmap, which means we do not need to refcount mappings.
 */

static struct hlist_head *vpid_hash;

static inline int vpid_hashfn(int vnr, int veid)
{
	return hash_long((unsigned long)(vnr+(veid<<16)), pidhash_shift);
}

static struct pid *__lookup_vpid_mapping(int vnr, int veid)
{
	struct hlist_node *elem;
	struct pid *map;

	hlist_for_each_entry_rcu(map, elem,
			&vpid_hash[vpid_hashfn(vnr, veid)], vpid_chain) {
		if (map->vnr == vnr && map->veid == veid)
			return map;
	}
	return NULL;
}

/* __vpid_to_pid() is raw version of vpid_to_pid(). It is to be used
 * only under tasklist_lock. In some places we must use only this version
 * (f.e. __kill_pg_info is called under write lock!)
 *
 * Caller should pass virtual pid. This function returns an error, when
 * seeing a global pid.
 */
pid_t __vpid_to_pid(pid_t pid)
{
	struct pid *map;

	if (unlikely(!is_virtual_pid(pid) || ve_is_super(get_exec_env())))
		return -1;

	if (!get_exec_env()->sparse_vpid) {
		int init_pid;

		init_pid = get_exec_env()->init_entry->pid;
		if (pid == 1)
			return init_pid;
		if (pid == init_pid + VPID_DIV)
			return -1; /* vpid of init is 1 */
		return pid - VPID_DIV;
	}

	rcu_read_lock();
	map = __lookup_vpid_mapping(pid, VEID(get_exec_env()));
	pid = (map != NULL ? map->nr : -1);
	rcu_read_unlock();
	return pid;
}
EXPORT_SYMBOL_GPL(__vpid_to_pid);

pid_t vpid_to_pid(pid_t pid)
{
	/* User gave bad pid. It is his problem. */
	if (unlikely(pid <= 0))
		return pid;

	if (!is_virtual_pid(pid))
		return pid;

	return __vpid_to_pid(pid);
}
EXPORT_SYMBOL_GPL(vpid_to_pid);

pid_t alloc_vpid(struct pid *pid, pid_t virt_pid)
{
	int result;
	struct ve_struct *env = get_exec_env();

	if (ve_is_super(env) || !env->virt_pids)
		return pid->vnr;

	BUG_ON(pid->veid != 0);

	spin_lock_irq(&pidmap_lock);
	if (!env->sparse_vpid) {
		result = pid->nr + VPID_DIV;
		if (virt_pid == -1)
			goto out_success;

		result = virt_pid;
		if (virt_pid == 1 || virt_pid == pid->nr + VPID_DIV)
			goto out_success;

		env->sparse_vpid = 1;
	}

	result = (virt_pid == -1) ? pid->nr + VPID_DIV : virt_pid;

	if (unlikely(__lookup_vpid_mapping(result, VEID(env)))) {
		if (virt_pid > 0) {
			result = -EEXIST;
			goto out;
		}

		/* No luck. Now we search for some not-existing vpid.
		 * It is weak place. We do linear search. */
		do {
			result++;
			if (!__is_virtual_pid(result))
				result += VPID_DIV;
			if (result >= pid_max)
				result = RESERVED_PIDS + VPID_DIV;
		} while (__lookup_vpid_mapping(result, VEID(env)) != NULL);

		/* And set last_pid in hope future alloc_pidmap to avoid
		 * collisions after future alloc_pidmap() */
		last_pid = result - VPID_DIV;
	}
	if (result > 0) {
out_success:
		pid->veid = VEID(env);
		pid->vnr = result;
		hlist_add_head_rcu(&pid->vpid_chain,
			       &vpid_hash[vpid_hashfn(result, pid->veid)]);
	}
out:
	spin_unlock_irq(&pidmap_lock);
	return result;
}
EXPORT_SYMBOL(alloc_vpid);

void free_vpid(struct pid * pid)
{
	if (pid->veid == 0)
		return;

	spin_lock_irq(&pidmap_lock);
	hlist_del_rcu(&pid->vpid_chain);
	spin_unlock_irq(&pidmap_lock);

	pid->veid = 0;
	pid->vnr = pid->nr;
}
EXPORT_SYMBOL(free_vpid);
#endif

/*
 * The pid hash table is scaled according to the amount of memory in the
 * machine.  From a minimum of 16 slots up to 4096 slots at one gigabyte or
 * more.
 */
void __init pidhash_init(void)
{
	int i, pidhash_size;
	unsigned long megabytes = nr_kernel_pages >> (20 - PAGE_SHIFT);

	pidhash_shift = max(4, fls(megabytes * 4));
	pidhash_shift = min(12, pidhash_shift);
	pidhash_size = 1 << pidhash_shift;

	printk("PID hash table entries: %d (order: %d, %Zd bytes)\n",
		pidhash_size, pidhash_shift,
		pidhash_size * sizeof(struct hlist_head));

	pid_hash = alloc_bootmem(pidhash_size *	sizeof(*(pid_hash)));
	if (!pid_hash)
		panic("Could not alloc pidhash!\n");
	for (i = 0; i < pidhash_size; i++)
		INIT_HLIST_HEAD(&pid_hash[i]);
 
#ifdef CONFIG_VE
	vpid_hash = alloc_bootmem(pidhash_size * sizeof(struct hlist_head));
	if (!vpid_hash)
		panic("Could not alloc vpid_hash!\n");
	for (i = 0; i < pidhash_size; i++)
		INIT_HLIST_HEAD(&vpid_hash[i]);
#endif
}

void __init pidmap_init(void)
{
	pidmap_array->page = (void *)get_zeroed_page(GFP_KERNEL);
	/* Reserve PID 0. We never call free_pidmap(0) */
	set_bit(0, pidmap_array->page);
	atomic_dec(&pidmap_array->nr_free);

	pid_cachep = kmem_cache_create("pid", sizeof(struct pid),
					__alignof__(struct pid),
					SLAB_PANIC, NULL, NULL);
}
