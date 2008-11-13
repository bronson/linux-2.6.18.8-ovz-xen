/*
 *  include/linux/ve_proto.h
 *
 *  Copyright (C) 2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#ifndef __VE_H__
#define __VE_H__

#ifdef CONFIG_VE

struct ve_struct;
struct seq_file;

typedef void (*ve_seq_print_t)(struct seq_file *, struct ve_struct *);

void vzmon_register_veaddr_print_cb(ve_seq_print_t);
void vzmon_unregister_veaddr_print_cb(ve_seq_print_t);

#ifdef CONFIG_INET
void ip_fragment_cleanup(struct ve_struct *envid);
void tcp_v4_kill_ve_sockets(struct ve_struct *envid);
struct fib_table * fib_hash_init(int id);
#ifdef CONFIG_VE_NETDEV
int venet_init(void);
#endif
#else
static inline void ip_fragment_cleanup(struct ve_struct *ve) { ; }
#endif

#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
extern void ip6_frag_cleanup(struct ve_struct *envid);
#else
static inline void ip6_frag_cleanup(struct ve_struct *envid) { ; }
#endif

extern struct list_head ve_list_head;
#define for_each_ve(ve)	list_for_each_entry((ve), &ve_list_head, ve_list)
extern rwlock_t ve_list_lock;
extern struct ve_struct *get_ve_by_id(envid_t);
extern struct ve_struct *__find_ve_by_id(envid_t);

struct env_create_param3;
extern int real_env_create(envid_t veid, unsigned flags, u32 class_id,
			   struct env_create_param3 *data, int datalen);
extern void ve_move_task(struct task_struct *, struct ve_struct *);

int set_device_perms_ve(envid_t veid, unsigned type, dev_t dev, unsigned mask);
int get_device_perms_ve(int dev_type, dev_t dev, int access_mode);
void clean_device_perms_ve(envid_t veid);
extern struct file_operations proc_devperms_ops;

enum {
	VE_SS_CHAIN,

	VE_MAX_CHAINS
};

typedef int ve_hook_init_fn(void *data);
typedef void ve_hook_fini_fn(void *data);

struct ve_hook
{
	ve_hook_init_fn *init;
	ve_hook_fini_fn *fini;
	struct module *owner;

	/* Functions are called in ascending priority */
	int priority;

	/* Private part */
	struct list_head list;
};

enum {
	HOOK_PRIO_DEFAULT = 0,

	HOOK_PRIO_FS = HOOK_PRIO_DEFAULT,

	HOOK_PRIO_NET_PRE,
	HOOK_PRIO_NET,
	HOOK_PRIO_NET_POST,

	HOOK_PRIO_AFTERALL = INT_MAX
};

extern int ve_hook_iterate_init(int chain, void *data);
extern void ve_hook_iterate_fini(int chain, void *data);

extern void ve_hook_register(int chain, struct ve_hook *vh);
extern void ve_hook_unregister(struct ve_hook *vh);
#else /* CONFIG_VE */
#define ve_hook_register(ch, vh)	do { } while (0)
#define ve_hook_unregister(ve)		do { } while (0)

#define get_device_perms_ve(t, d, a)	(0)
#endif /* CONFIG_VE */
#endif
