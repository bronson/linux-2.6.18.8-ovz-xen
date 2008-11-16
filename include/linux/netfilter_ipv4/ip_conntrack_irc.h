/* IRC extension for IP connection tracking.
 * (C) 2000 by Harald Welte <laforge@gnumonks.org>
 * based on RR's ip_conntrack_ftp.h
 *
 * ip_conntrack_irc.h,v 1.6 2000/11/07 18:26:42 laforge Exp
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 *
 */
#ifndef _IP_CONNTRACK_IRC_H
#define _IP_CONNTRACK_IRC_H

#include <linux/netfilter_ipv4/ip_conntrack_helper.h>

/* This structure exists only once per master */
struct ip_ct_irc_master {
};

#ifdef __KERNEL__
typedef unsigned int (*ip_nat_helper_irc_hook)(struct sk_buff **,
		enum ip_conntrack_info, unsigned int, unsigned int,
		struct ip_conntrack_expect *);

extern ip_nat_helper_irc_hook ip_nat_irc_hook;
#ifdef CONFIG_VE_IPTABLES
#include <linux/sched.h>
#define ve_ip_nat_irc_hook \
	((ip_nat_helper_irc_hook) \
		(get_exec_env()->_ip_conntrack->_ip_nat_irc_hook))
#else
#define ve_ip_nat_irc_hook	ip_nat_irc_hook
#endif

#define IRC_PORT	6667

#endif /* __KERNEL__ */

#endif /* _IP_CONNTRACK_IRC_H */
