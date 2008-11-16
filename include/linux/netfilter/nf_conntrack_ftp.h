#ifndef _NF_CONNTRACK_FTP_H
#define _NF_CONNTRACK_FTP_H
/* FTP tracking. */

/* This enum is exposed to userspace */
enum ip_ct_ftp_type
{
	/* PORT command from client */
	IP_CT_FTP_PORT,
	/* PASV response from server */
	IP_CT_FTP_PASV,
	/* EPRT command from client */
	IP_CT_FTP_EPRT,
	/* EPSV response from server */
	IP_CT_FTP_EPSV,
};

#ifdef __KERNEL__

#define FTP_PORT	21

#define NUM_SEQ_TO_REMEMBER 2
/* This structure exists only once per master */
struct ip_ct_ftp_master {
	/* Valid seq positions for cmd matching after newline */
	u_int32_t seq_aft_nl[IP_CT_DIR_MAX][NUM_SEQ_TO_REMEMBER];
	/* 0 means seq_match_aft_nl not set */
	int seq_aft_nl_num[IP_CT_DIR_MAX];
};

struct ip_conntrack_expect;

/* For NAT to hook in when we find a packet which describes what other
 * connection we should expect. */
typedef unsigned int (*ip_nat_helper_ftp_hook)(struct sk_buff **pskb,
				       enum ip_conntrack_info ctinfo,
				       enum ip_ct_ftp_type type,
				       unsigned int matchoff,
				       unsigned int matchlen,
				       struct ip_conntrack_expect *exp,
				       u32 *seq);
extern ip_nat_helper_ftp_hook ip_nat_ftp_hook;
#ifdef CONFIG_VE_IPTABLES
#include <linux/sched.h>
#define ve_ip_nat_ftp_hook \
	((ip_nat_helper_ftp_hook) \
		(get_exec_env()->_ip_conntrack->_ip_nat_ftp_hook))
#else
#define ve_ip_nat_ftp_hook	ip_nat_ftp_hook
#endif
#endif /* __KERNEL__ */

#endif /* _NF_CONNTRACK_FTP_H */
