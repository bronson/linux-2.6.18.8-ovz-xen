#ifndef _IP_CONNTRACK_CORE_H
#define _IP_CONNTRACK_CORE_H
#include <linux/netfilter.h>

#define MAX_IP_CT_PROTO 256

/* This header is used to share core functionality between the
   standalone connection tracking module, and the compatibility layer's use
   of connection tracking. */
extern unsigned int ip_conntrack_in(unsigned int hooknum,
				    struct sk_buff **pskb,
				    const struct net_device *in,
				    const struct net_device *out,
				    int (*okfn)(struct sk_buff *));

extern int ip_conntrack_init(void);
extern void ip_conntrack_cleanup(void);

struct ip_conntrack_protocol;

extern int
ip_ct_get_tuple(const struct iphdr *iph,
		const struct sk_buff *skb,
		unsigned int dataoff,
		struct ip_conntrack_tuple *tuple,
		const struct ip_conntrack_protocol *protocol);

extern int
ip_ct_invert_tuple(struct ip_conntrack_tuple *inverse,
		   const struct ip_conntrack_tuple *orig,
		   const struct ip_conntrack_protocol *protocol);

/* Find a connection corresponding to a tuple. */
struct ip_conntrack_tuple_hash *
ip_conntrack_find_get(const struct ip_conntrack_tuple *tuple,
		      const struct ip_conntrack *ignored_conntrack);

extern int __ip_conntrack_confirm(struct sk_buff **pskb);

/* Confirm a connection: returns NF_DROP if packet must be dropped. */
static inline int ip_conntrack_confirm(struct sk_buff **pskb)
{
	struct ip_conntrack *ct = (struct ip_conntrack *)(*pskb)->nfct;
	int ret = NF_ACCEPT;

	if (ct) {
		if (!is_confirmed(ct))
			ret = __ip_conntrack_confirm(pskb);
		ip_ct_deliver_cached_events(ct);
	}
	return ret;
}

extern void ip_ct_unlink_expect(struct ip_conntrack_expect *exp);

#ifdef CONFIG_VE_IPTABLES
#include <linux/sched.h>
#define ve_ip_ct_initialized() \
	(get_exec_env()->_ip_conntrack != NULL)
#define ve_ip_ct_protos \
	(get_exec_env()->_ip_conntrack->_ip_ct_protos)
#define ve_ip_conntrack_hash	\
	(get_exec_env()->_ip_conntrack->_ip_conntrack_hash)
#define ve_ip_conntrack_expect_list \
	(get_exec_env()->_ip_conntrack->_ip_conntrack_expect_list)
#define ve_ip_conntrack_vmalloc \
	(get_exec_env()->_ip_conntrack->_ip_conntrack_vmalloc)
#else
extern struct ip_conntrack_protocol *ip_ct_protos[MAX_IP_CT_PROTO];
extern struct list_head *ip_conntrack_hash;
extern struct list_head ip_conntrack_expect_list;
#define ve_ip_ct_initialized()		1
#define ve_ip_ct_protos			ip_ct_protos
#define ve_ip_conntrack_hash		ip_conntrack_hash
#define ve_ip_conntrack_expect_list	ip_conntrack_expect_list
#define ve_ip_conntrack_vmalloc		ip_conntrack_vmalloc
#endif /* CONFIG_VE_IPTABLES */

extern rwlock_t ip_conntrack_lock;
#endif /* _IP_CONNTRACK_CORE_H */

