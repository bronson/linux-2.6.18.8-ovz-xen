/*
 *  include/linux/nfcalls.h
 *
 *  Copyright (C) 2005  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#ifndef _LINUX_NFCALLS_H
#define _LINUX_NFCALLS_H

#include <linux/rcupdate.h>

#ifdef CONFIG_MODULES
extern struct module no_module;

#define DECL_KSYM_MODULE(name)				\
	extern struct module *vz_mod_##name

#define INIT_KSYM_MODULE(name)				\
	struct module *vz_mod_##name = &no_module;	\
	EXPORT_SYMBOL(vz_mod_##name)

static inline void __vzksym_modresolve(struct module **modp, struct module *mod)
{
	/*
	 * we want to be sure, that pointer updates are visible first:
	 * 1. wmb() is here only for piece of sure
	 *    (note, no rmb() in KSYMSAFECALL)
	 * 2. synchronize_sched() guarantees that updates are visible
	 *    on all cpus and allows us to remove rmb() in KSYMSAFECALL
	 */
	wmb(); synchronize_sched();
	*modp = mod;
	/* just to be sure, our changes are visible as soon as possible */
	wmb(); synchronize_sched();
}

static inline void __vzksym_modunresolve(struct module **modp)
{
	/*
	 * try_module_get() in KSYMSAFECALL should fail at this moment since
	 * THIS_MODULE in in unloading state (we should be called from fini),
	 * no need to syncronize pointers/ve_module updates.
	 */
	*modp = &no_module;
	/*
	 * synchronize_sched() guarantees here that we see
	 * updated module pointer before the module really gets away
	 */
	synchronize_sched();
}

static inline int __vzksym_module_get(struct module *mod)
{
	/*
	 * we want to avoid rmb(), so use synchronize_sched() in KSYMUNRESOLVE
	 * and smp_read_barrier_depends() here...
	 */
	smp_read_barrier_depends(); /* for module loading */
	if (!try_module_get(mod))
		return -EBUSY;

	return 0;
}

static inline void __vzksym_module_put(struct module *mod)
{
	module_put(mod);
}
#else
#define DECL_KSYM_MODULE(name)
#define INIT_KSYM_MODULE(name)
#define __vzksym_modresolve(modp, mod)
#define __vzksym_modunresolve(modp)
#define __vzksym_module_get(mod)			(0)
#define __vzksym_module_put(mod)
#endif

#define __KSYMERRCALL(err, type, mod, name, args)	\
({							\
	type ret = (type)err;				\
	if (!__vzksym_module_get(vz_mod_##mod))	{	\
		if (vz_##name)				\
			ret = ((*vz_##name)args); 	\
		__vzksym_module_put(vz_mod_##mod);	\
	}						\
	ret;						\
})
#define __KSYMSAFECALL_VOID(mod, name, args)		\
do {							\
	if (!__vzksym_module_get(vz_mod_##mod)) {	\
		if (vz_##name)				\
			((*vz_##name)args); 		\
		__vzksym_module_put(vz_mod_##mod);	\
	}						\
} while (0)

#define DECL_KSYM_CALL(type, name, args)		\
	extern type (*vz_##name) args
#define INIT_KSYM_CALL(type, name, args)		\
	type (*vz_##name) args;				\
	EXPORT_SYMBOL(vz_##name)

#define KSYMERRCALL(err, mod, name, args)		\
	__KSYMERRCALL(err, int, mod, name, args)
#define KSYMSAFECALL(type, mod, name, args)		\
	__KSYMERRCALL(0, type, mod, name, args)
#define KSYMSAFECALL_VOID(mod, name, args)		\
	__KSYMSAFECALL_VOID(mod, name, args)
#define KSYMREF(name)					vz_##name

/* should be called _after_ KSYMRESOLVE's */
#define KSYMMODRESOLVE(name)				\
	__vzksym_modresolve(&vz_mod_##name, THIS_MODULE)
#define KSYMMODUNRESOLVE(name)				\
	__vzksym_modunresolve(&vz_mod_##name)

#define KSYMRESOLVE(name)				\
	vz_##name = &name
#define KSYMUNRESOLVE(name)				\
	vz_##name = NULL

#if defined(CONFIG_VE)
DECL_KSYM_MODULE(ip_tables);
DECL_KSYM_MODULE(ip6_tables);
DECL_KSYM_MODULE(iptable_filter);
DECL_KSYM_MODULE(ip6table_filter);
DECL_KSYM_MODULE(iptable_mangle);
DECL_KSYM_MODULE(ip6table_mangle);
DECL_KSYM_MODULE(ip_conntrack);
DECL_KSYM_MODULE(ip_conntrack_ftp);
DECL_KSYM_MODULE(ip_conntrack_irc);
DECL_KSYM_MODULE(xt_conntrack);
DECL_KSYM_MODULE(ip_nat);
DECL_KSYM_MODULE(iptable_nat);
DECL_KSYM_MODULE(ip_nat_ftp);
DECL_KSYM_MODULE(ip_nat_irc);

struct sk_buff;

DECL_KSYM_CALL(int, init_netfilter, (void));
DECL_KSYM_CALL(int, init_iptables, (void));
DECL_KSYM_CALL(int, init_ip6tables, (void));
DECL_KSYM_CALL(int, init_iptable_filter, (void));
DECL_KSYM_CALL(int, init_ip6table_filter, (void));
DECL_KSYM_CALL(int, init_iptable_mangle, (void));
DECL_KSYM_CALL(int, init_ip6table_mangle, (void));
DECL_KSYM_CALL(int, init_iptable_conntrack, (void));
DECL_KSYM_CALL(int, init_ip_ct_ftp, (void));
DECL_KSYM_CALL(int, init_ip_ct_irc, (void));
DECL_KSYM_CALL(int, ip_nat_init, (void));
DECL_KSYM_CALL(int, init_iptable_nat, (void));
DECL_KSYM_CALL(int, init_iptable_nat_ftp, (void));
DECL_KSYM_CALL(int, init_iptable_nat_irc, (void));
DECL_KSYM_CALL(void, fini_iptable_nat_irc, (void));
DECL_KSYM_CALL(void, fini_iptable_nat_ftp, (void));
DECL_KSYM_CALL(void, fini_iptable_nat, (void));
DECL_KSYM_CALL(void, ip_nat_cleanup, (void));
DECL_KSYM_CALL(void, fini_ip_ct_irc, (void));
DECL_KSYM_CALL(void, fini_ip_ct_ftp, (void));
DECL_KSYM_CALL(void, fini_iptable_conntrack, (void));
DECL_KSYM_CALL(void, fini_ip6table_filter, (void));
DECL_KSYM_CALL(void, fini_iptable_filter, (void));
DECL_KSYM_CALL(void, fini_ip6table_mangle, (void));
DECL_KSYM_CALL(void, fini_iptable_mangle, (void));
DECL_KSYM_CALL(void, fini_ip6tables, (void));
DECL_KSYM_CALL(void, fini_iptables, (void));
DECL_KSYM_CALL(void, fini_netfilter, (void));

#include <linux/netfilter/x_tables.h>
#endif /* CONFIG_VE */

#if defined(CONFIG_VE_ETHDEV) || defined(CONFIG_VE_ETHDEV_MODULE)
DECL_KSYM_MODULE(vzethdev);
DECL_KSYM_CALL(int, veth_open, (struct net_device *dev));
#endif

#if defined(CONFIG_VE_CALLS) || defined(CONFIG_VE_CALLS_MODULE)
DECL_KSYM_MODULE(vzmon);
DECL_KSYM_CALL(void, real_do_env_free, (struct ve_struct *env));
#endif

#endif /* _LINUX_NFCALLS_H */
