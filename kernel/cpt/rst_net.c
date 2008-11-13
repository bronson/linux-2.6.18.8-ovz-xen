/*
 *
 *  kernel/cpt/rst_net.c
 *
 *  Copyright (C) 2000-2005  SWsoft
 *  All rights reserved.
 *
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#include <linux/config.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/socket.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/rtnetlink.h>
#include <linux/ve.h>
#include <linux/ve_proto.h>
#include <net/route.h>
#include <net/ip_fib.h>
#include <net/addrconf.h>
#include <linux/if_tun.h>
#include <linux/veth.h>
#include <linux/nfcalls.h>
#include <linux/venet.h>

#include "cpt_obj.h"
#include "cpt_context.h"
#include "cpt_kernel.h"
#include "cpt_net.h"
#include "cpt_files.h"

#include "cpt_syscalls.h"

extern struct in_ifaddr *inet_alloc_ifa(void);
extern int inet_insert_ifa(struct in_ifaddr *ifa);

int rst_restore_ifaddr(struct cpt_context *ctx)
{
	int err;
	loff_t sec = ctx->sections[CPT_SECT_NET_IFADDR];
	loff_t endsec;
	struct cpt_section_hdr h;
	struct cpt_ifaddr_image di;
	struct net_device *dev;

	if (sec == CPT_NULL)
		return 0;

	err = ctx->pread(&h, sizeof(h), ctx, sec);
	if (err)
		return err;
	if (h.cpt_section != CPT_SECT_NET_IFADDR || h.cpt_hdrlen < sizeof(h))
		return -EINVAL;

	endsec = sec + h.cpt_next;
	sec += h.cpt_hdrlen;
	while (sec < endsec) {
		int cindex = -1;
		int err;
		err = rst_get_object(CPT_OBJ_NET_IFADDR, sec, &di, ctx);
		if (err)
			return err;
		cindex = di.cpt_index;
		rtnl_lock();
		dev = __dev_get_by_index(cindex);
		if (dev && di.cpt_family == AF_INET) {
			struct in_device *in_dev;
			struct in_ifaddr *ifa;
			if ((in_dev = __in_dev_get_rtnl(dev)) == NULL)
				in_dev = inetdev_init(dev);
			ifa = inet_alloc_ifa();
			if (ifa) {
				ifa->ifa_local = di.cpt_address[0];
				ifa->ifa_address = di.cpt_peer[0];
				ifa->ifa_broadcast = di.cpt_broadcast[0];
				ifa->ifa_prefixlen = di.cpt_masklen;
				ifa->ifa_mask = inet_make_mask(ifa->ifa_prefixlen);
				ifa->ifa_flags = di.cpt_flags;
				ifa->ifa_scope = di.cpt_scope;
				memcpy(ifa->ifa_label, di.cpt_label, IFNAMSIZ);
				in_dev_hold(in_dev);
				ifa->ifa_dev   = in_dev;
				err = inet_insert_ifa(ifa);
				if (err && err != -EEXIST) {
					rtnl_unlock();
					eprintk_ctx("add ifaddr err %d for %d %s\n", err, di.cpt_index, di.cpt_label);
					return err;
				}
			}
#if defined(CONFIG_IPV6) || defined (CONFIG_IPV6_MODULE)
		} else if (dev && di.cpt_family == AF_INET6) {
			__u32 prefered_lft;
			__u32 valid_lft;
			prefered_lft = (di.cpt_flags & IFA_F_DEPRECATED) ?
				0 : di.cpt_prefered_lft;
			valid_lft = (di.cpt_flags & IFA_F_PERMANENT) ?
				0xFFFFFFFF : di.cpt_valid_lft;
			err = inet6_addr_add(dev->ifindex,
					     (struct in6_addr *)di.cpt_address,
					     di.cpt_masklen,
					     prefered_lft,
					     valid_lft);
			if (err && err != -EEXIST) {
				rtnl_unlock();
				eprintk_ctx("add ifaddr err %d for %d %s\n", err, di.cpt_index, di.cpt_label);
				return err;
			}
#endif
		} else {
			rtnl_unlock();
			eprintk_ctx("unknown ifaddr 2 for %d\n", di.cpt_index);
			return -EINVAL;
		}
		rtnl_unlock();
		sec += di.cpt_next;
	}
	return 0;
}

static int rewrite_rtmsg(struct nlmsghdr *nlh, struct cpt_context *ctx)
{
	int min_len = NLMSG_LENGTH(sizeof(struct rtmsg));
	struct rtmsg *rtm = NLMSG_DATA(nlh);
	__u32 prefix0 = 0;

	if (nlh->nlmsg_len > min_len) {
		int attrlen = nlh->nlmsg_len - NLMSG_ALIGN(min_len);
		struct rtattr *rta = (void*)nlh + NLMSG_ALIGN(min_len);

		while (RTA_OK(rta, attrlen)) {
			if (rta->rta_type == RTA_DST) {
				prefix0 = *(__u32*)RTA_DATA(rta);
			}
			rta = RTA_NEXT(rta, attrlen);
		}
	}
#if defined(CONFIG_IPV6) || defined (CONFIG_IPV6_MODULE)
	if (rtm->rtm_family == AF_INET6) {
		if (rtm->rtm_type == RTN_LOCAL)
			return 2;
		if (rtm->rtm_flags & RTM_F_CLONED)
			return 2;
		if (rtm->rtm_protocol == RTPROT_UNSPEC ||
		    rtm->rtm_protocol == RTPROT_RA ||
		    rtm->rtm_protocol == RTPROT_REDIRECT ||
		    rtm->rtm_protocol == RTPROT_KERNEL)
			return 2;
		if (rtm->rtm_protocol == RTPROT_BOOT &&
		    ((rtm->rtm_dst_len == 8 && prefix0 == htonl(0xFF000000)) ||
		     (rtm->rtm_dst_len == 64 && prefix0 == htonl(0xFE800000))))
			return 2;
	}
#endif
	return rtm->rtm_protocol == RTPROT_KERNEL;
}

int rst_restore_route(struct cpt_context *ctx)
{
	int err;
	struct socket *sock;
	struct msghdr msg;
	struct iovec iov;
	struct sockaddr_nl nladdr;
	mm_segment_t oldfs;
	loff_t sec = ctx->sections[CPT_SECT_NET_ROUTE];
	loff_t endsec;
	struct cpt_section_hdr h;
	struct cpt_object_hdr v;
	char *pg;

	if (sec == CPT_NULL)
		return 0;

	err = ctx->pread(&h, sizeof(h), ctx, sec);
	if (err)
		return err;
	if (h.cpt_section != CPT_SECT_NET_ROUTE || h.cpt_hdrlen < sizeof(h))
		return -EINVAL;

	if (h.cpt_hdrlen >= h.cpt_next)
		return 0;

	sec += h.cpt_hdrlen;
	err = rst_get_object(CPT_OBJ_NET_ROUTE, sec, &v, ctx);
	if (err < 0)
		return err;

	err = sock_create_kern(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE, &sock);
	if (err)
		return err;

	pg = (char*)__get_free_page(GFP_KERNEL);
	if (pg == NULL) {
		err = -ENOMEM;
		goto out_sock;
	}

	memset(&nladdr, 0, sizeof(nladdr));
	nladdr.nl_family = AF_NETLINK;

	endsec = sec + v.cpt_next;
	sec += v.cpt_hdrlen;

	while (sec < endsec) {
		struct nlmsghdr *n;
		struct nlmsghdr nh;
		int kernel_flag;

		if (endsec - sec < sizeof(nh))
			break;

		err = ctx->pread(&nh, sizeof(nh), ctx, sec);
		if (err)
			goto out_sock_pg;
		if (nh.nlmsg_len < sizeof(nh) || nh.nlmsg_len > PAGE_SIZE ||
		    endsec - sec < nh.nlmsg_len) {
			err = -EINVAL;
			goto out_sock_pg;
		}
		err = ctx->pread(pg, nh.nlmsg_len, ctx, sec);
		if (err)
			goto out_sock_pg;

		n = (struct nlmsghdr*)pg;
		n->nlmsg_flags = NLM_F_REQUEST|NLM_F_APPEND|NLM_F_CREATE;

		err = rewrite_rtmsg(n, ctx);
		if (err < 0)
			goto out_sock_pg;
		kernel_flag = err;

		if (kernel_flag == 2)
			goto do_next;

		iov.iov_base=n;
		iov.iov_len=nh.nlmsg_len;
		msg.msg_name=&nladdr;
		msg.msg_namelen=sizeof(nladdr);
		msg.msg_iov=&iov;
		msg.msg_iovlen=1;
		msg.msg_control=NULL;
		msg.msg_controllen=0;
		msg.msg_flags=MSG_DONTWAIT;

		oldfs = get_fs(); set_fs(KERNEL_DS);
		err = sock_sendmsg(sock, &msg, nh.nlmsg_len);
		set_fs(oldfs);

		if (err < 0)
			goto out_sock_pg;
		err = 0;

		iov.iov_base=pg;
		iov.iov_len=PAGE_SIZE;

		oldfs = get_fs(); set_fs(KERNEL_DS);
		err = sock_recvmsg(sock, &msg, PAGE_SIZE, MSG_DONTWAIT);
		set_fs(oldfs);
		if (err != -EAGAIN) {
			if (err == NLMSG_LENGTH(sizeof(struct nlmsgerr)) &&
			    n->nlmsg_type == NLMSG_ERROR) {
				struct nlmsgerr *e = NLMSG_DATA(n);
				if (e->error != -EEXIST || !kernel_flag)
					eprintk_ctx("NLMERR: %d\n", e->error);
			} else {
				eprintk_ctx("Res: %d %d\n", err, n->nlmsg_type);
			}
		}
do_next:
		err = 0;
		sec += NLMSG_ALIGN(nh.nlmsg_len);
	}

out_sock_pg:
	free_page((unsigned long)pg);
out_sock:
	sock_release(sock);
	return err;
}

int rst_resume_network(struct cpt_context *ctx)
{
	struct ve_struct *env;

	env = get_ve_by_id(ctx->ve_id);
	if (!env)
		return -ESRCH;
	env->disable_net = 0;
	put_ve(env);
	return 0;
}

/* We do not restore skb queue, just reinit it */
static int rst_restore_tuntap(loff_t start, struct cpt_netdev_image *di,
			struct cpt_context *ctx)
{
	int err = -ENODEV;
#if defined(CONFIG_TUN) || defined(CONFIG_TUN_MODULE)
	struct cpt_tuntap_image ti;
	struct net_device *dev;
	struct file *bind_file = NULL;
	struct tun_struct *tun;
	loff_t pos;

	pos = start + di->cpt_hdrlen;
	err = rst_get_object(CPT_OBJ_NET_TUNTAP, pos, &ti, ctx);
	if (err)
		return err;

	pos += ti.cpt_next;
	if (ti.cpt_bindfile) {
		bind_file = rst_file(ti.cpt_bindfile, -1, ctx);
		if (IS_ERR(bind_file)) {
			eprintk_ctx("rst_restore_tuntap:"
				"rst_file: %Ld\n",
				(unsigned long long)ti.cpt_bindfile);
			return PTR_ERR(bind_file);
		}
	}

	rtnl_lock();
	err = -ENOMEM;
	dev = alloc_netdev(sizeof(struct tun_struct), di->cpt_name, tun_setup);
	if (!dev)
		goto out;

	tun = netdev_priv(dev);

	tun->dev = dev;
	tun->owner = ti.cpt_owner;
	tun->flags = ti.cpt_flags;
	tun->attached = ti.cpt_attached;
	tun->if_flags = ti.cpt_if_flags;
	tun_net_init(dev);
	BUG_ON(sizeof(ti.cpt_dev_addr) != sizeof(tun->dev_addr));
	memcpy(tun->dev_addr, ti.cpt_dev_addr, sizeof(ti.cpt_dev_addr));
	BUG_ON(sizeof(ti.cpt_chr_filter) != sizeof(tun->chr_filter));
	memcpy(tun->chr_filter, ti.cpt_chr_filter, sizeof(ti.cpt_chr_filter));
	BUG_ON(sizeof(ti.cpt_net_filter) != sizeof(tun->net_filter));
	memcpy(tun->net_filter, ti.cpt_net_filter, sizeof(ti.cpt_net_filter));

	err = register_netdevice(dev);
	if (err < 0) {
		free_netdev(dev);
		eprintk_ctx("failed to register tun/tap net device\n");
		goto out;
	}
	if (pos < start + di->cpt_next) {
		struct cpt_hwaddr_image hw;
		/* Restore hardware address */
		err = rst_get_object(CPT_OBJ_NET_HWADDR, pos,
				&hw, ctx);
		if (err)
			goto out;
		BUG_ON(sizeof(hw.cpt_dev_addr) != sizeof(dev->dev_addr));
		memcpy(dev->dev_addr, hw.cpt_dev_addr,
				sizeof(hw.cpt_dev_addr));
	}
	list_add(&tun->list, &tun_dev_list);

	bind_file->private_data = tun;
	tun->bind_file = bind_file;

out:
	fput(bind_file);
	rtnl_unlock();
#endif
	return err;
}

static int rst_restore_veth(loff_t pos, struct net_device *dev,
			struct cpt_context *ctx)
{
	int err = -ENODEV;
#if defined(CONFIG_VE_ETHDEV) || defined(CONFIG_VE_ETHDEV_MODULE)
	struct cpt_veth_image vi;
	struct veth_struct *veth;

	if (!KSYMREF(veth_open) || dev->open != KSYMREF(veth_open)) {
		eprintk_ctx("Module vzethdev is not loaded, "
			    "or device %s is not a veth device\n", dev->name);
		return -EINVAL;
	}
	err = rst_get_object(CPT_OBJ_NET_VETH, pos, &vi, ctx);
	if (err)
		return err;
	veth = veth_from_netdev(dev);
	veth->allow_mac_change = vi.cpt_allow_mac_change;
#endif
	return err;
}

static int rst_restore_netstats(loff_t pos, struct net_device *dev,
			struct cpt_context * ctx)
{
	struct cpt_netstats_image *n;
	struct net_device_stats *stats = NULL;
	int err;

	if (!dev->get_stats)
		return 0;

	n = cpt_get_buf(ctx);
	err = rst_get_object(CPT_OBJ_NET_STATS, pos, n, ctx);
	if (err)
		goto out;
	BUG_ON(sizeof(struct cpt_netstats_image) != n->cpt_hdrlen);
	preempt_disable();
	if (dev == get_exec_env()->_loopback_dev)
		stats = per_cpu_ptr(get_exec_env()->_loopback_stats,
				smp_processor_id());
#if defined(CONFIG_VE_ETHDEV) || defined(CONFIG_VE_ETHDEV_MODULE)
	else if (KSYMREF(veth_open) && dev->open == KSYMREF(veth_open))
		stats = veth_stats(dev, smp_processor_id());
#endif
#if defined(CONFIG_VE_NETDEV) || defined(CONFIG_VE_NETDEV_MODULE)
	else if (dev == get_exec_env()->_venet_dev)
		stats = venet_stats(dev, smp_processor_id());
#endif
#if defined(CONFIG_TUN) || defined(CONFIG_TUN_MODULE)
	if (dev->open == tun_net_open)
		stats = &((struct tun_struct *)netdev_priv(dev))->stats;
#endif
	if (!stats) {
		err = -ENODEV;
		eprintk_ctx("Network device %s is not supported\n", dev->name);
		goto out;
	}

	stats->rx_packets = n->cpt_rx_packets;
	stats->tx_packets = n->cpt_tx_packets;
	stats->rx_bytes = n->cpt_rx_bytes;
	stats->tx_bytes = n->cpt_tx_bytes;
	stats->rx_errors = n->cpt_rx_errors;
	stats->tx_errors = n->cpt_tx_errors;
	stats->rx_dropped = n->cpt_rx_dropped;
	stats->tx_dropped = n->cpt_tx_dropped;
	stats->multicast = n->cpt_multicast;
	stats->collisions = n->cpt_collisions;
	stats->rx_length_errors = n->cpt_rx_length_errors;
	stats->rx_over_errors = n->cpt_rx_over_errors;
	stats->rx_crc_errors = n->cpt_rx_crc_errors;
	stats->rx_frame_errors = n->cpt_rx_frame_errors;
	stats->rx_fifo_errors = n->cpt_rx_fifo_errors;
	stats->rx_missed_errors = n->cpt_rx_missed_errors;
	stats->tx_aborted_errors = n->cpt_tx_aborted_errors;
	stats->tx_carrier_errors = n->cpt_tx_carrier_errors;
	stats->tx_fifo_errors = n->cpt_tx_fifo_errors;
	stats->tx_heartbeat_errors = n->cpt_tx_heartbeat_errors;
	stats->tx_window_errors = n->cpt_tx_window_errors;
	stats->rx_compressed = n->cpt_rx_compressed;
	stats->tx_compressed = n->cpt_tx_compressed;

out:
	preempt_enable();
	cpt_release_buf(ctx);
	return err;
}

int rst_restore_netdev(struct cpt_context *ctx)
{
	int err;
	loff_t sec = ctx->sections[CPT_SECT_NET_DEVICE];
	loff_t endsec;
	struct cpt_section_hdr h;
	struct cpt_netdev_image di;
	struct net_device *dev;

	get_exec_env()->disable_net = 1;

	if (sec == CPT_NULL)
		return 0;

	err = ctx->pread(&h, sizeof(h), ctx, sec);
	if (err)
		return err;
	if (h.cpt_section != CPT_SECT_NET_DEVICE || h.cpt_hdrlen < sizeof(h))
		return -EINVAL;

	endsec = sec + h.cpt_next;
	sec += h.cpt_hdrlen;
	while (sec < endsec) {
		loff_t pos;
		struct net_device *dev_new;
		err = rst_get_object(CPT_OBJ_NET_DEVICE, sec, &di, ctx);
		if (err)
			return err;

		pos = sec + di.cpt_hdrlen;
		if (di.cpt_next > sizeof(di)) {
			struct cpt_object_hdr hdr;
			err = ctx->pread(&hdr, sizeof(struct cpt_object_hdr),
					ctx, sec + di.cpt_hdrlen);
			if (err)
				return err;
			if (hdr.cpt_object == CPT_OBJ_NET_TUNTAP) {
				err = rst_restore_tuntap(sec, &di, ctx);
				if (err) {
					eprintk_ctx("restore tuntap %s: %d\n",
							di.cpt_name, err);
					return err;
				}
				pos += hdr.cpt_next;
			}
		}

		rtnl_lock();
		dev = __dev_get_by_name(di.cpt_name);
		if (dev) {
			if (dev->ifindex != di.cpt_index) {
				dev_new = __dev_get_by_index(di.cpt_index);
				if (!dev_new) {
					write_lock_bh(&dev_base_lock);
					hlist_del(&dev->index_hlist);
					if (dev->iflink == dev->ifindex)
						dev->iflink = di.cpt_index;
					dev->ifindex = di.cpt_index;
					hlist_add_head(&dev->index_hlist,
							dev_index_hash(dev->ifindex,
								get_exec_env()));
					write_unlock_bh(&dev_base_lock);
				} else {
					write_lock_bh(&dev_base_lock);
					hlist_del(&dev->index_hlist);
					hlist_del(&dev_new->index_hlist);
					if (dev_new->iflink == dev_new->ifindex)
						dev_new->iflink = dev->ifindex;
					dev_new->ifindex = dev->ifindex;
					if (dev->iflink == dev->ifindex)
						dev->iflink = di.cpt_index;
					dev->ifindex = di.cpt_index;
					hlist_add_head(&dev->index_hlist,
							dev_index_hash(dev->ifindex,
								get_exec_env()));
					hlist_add_head(&dev_new->index_hlist,
							dev_index_hash(dev_new->ifindex,
								get_exec_env()));
					write_unlock_bh(&dev_base_lock);
				}
			}
			if (di.cpt_flags^dev->flags) {
				err = dev_change_flags(dev, di.cpt_flags);
				if (err)
					eprintk_ctx("dev_change_flags err: %d\n", err);
			}
			while (pos < sec + di.cpt_next) {
				struct cpt_object_hdr hdr;
				err = ctx->pread(&hdr, sizeof(struct cpt_object_hdr),
						ctx, pos);
				if (err)
					goto out;
				if (hdr.cpt_object == CPT_OBJ_NET_VETH) {
					err = rst_restore_veth(pos, dev, ctx);
					if (err) {
						eprintk_ctx("restore veth %s: %d\n",
								di.cpt_name, err);
						goto out;
					}
				} else if (hdr.cpt_object == CPT_OBJ_NET_HWADDR) {
					/* Restore hardware address */
					struct cpt_hwaddr_image hw;
					err = rst_get_object(CPT_OBJ_NET_HWADDR,
							pos, &hw, ctx);
					if (err)
						goto out;
				BUG_ON(sizeof(hw.cpt_dev_addr) !=
						sizeof(dev->dev_addr));
					memcpy(dev->dev_addr, hw.cpt_dev_addr,
							sizeof(hw.cpt_dev_addr));
				} else if (hdr.cpt_object == CPT_OBJ_NET_STATS) {
					err = rst_restore_netstats(pos, dev, ctx);
					if (err) {
						eprintk_ctx("rst stats %s: %d\n",
								di.cpt_name, err);
						goto out;
					}
				}
				pos += hdr.cpt_next;
			}
		} else {
			eprintk_ctx("unknown interface 2 %s\n", di.cpt_name);
		}
		rtnl_unlock();
		sec += di.cpt_next;
	}
	return 0;
out:
	rtnl_unlock();
	return err;
}

static int dumpfn(void *arg)
{
	int i;
	int *pfd = arg;
	char *argv[] = { "iptables-restore", "-c", NULL };

	if (pfd[0] != 0)
		sc_dup2(pfd[0], 0);

	for (i=1; i<current->files->fdt->max_fds; i++)
		sc_close(i);

	module_put(THIS_MODULE);

	set_fs(KERNEL_DS);
	i = sc_execve("/sbin/iptables-restore", argv, NULL);
	if (i == -ENOENT)
		i = sc_execve("/usr/sbin/iptables-restore", argv, NULL);
	eprintk("failed to exec iptables-restore: %d\n", i);
	return 255 << 8;
}

static int rst_restore_iptables(struct cpt_context * ctx)
{
	int err;
	int pfd[2];
	struct file *f;
	struct cpt_object_hdr v;
	int n;
	struct cpt_section_hdr h;
	loff_t sec = ctx->sections[CPT_SECT_NET_IPTABLES];
	loff_t end;
	int pid;
	int status;
	mm_segment_t oldfs;
	sigset_t ignore, blocked;

	if (sec == CPT_NULL)
		return 0;

	err = ctx->pread(&h, sizeof(h), ctx, sec);
	if (err)
		return err;
	if (h.cpt_section != CPT_SECT_NET_IPTABLES || h.cpt_hdrlen < sizeof(h))
		return -EINVAL;

	if (h.cpt_hdrlen == h.cpt_next)
		return 0;
	if (h.cpt_hdrlen > h.cpt_next)
		return -EINVAL;
	sec += h.cpt_hdrlen;
	err = rst_get_object(CPT_OBJ_NAME, sec, &v, ctx);
	if (err < 0)
		return err;

	err = sc_pipe(pfd);
	if (err < 0)
		return err;
	ignore.sig[0] = CPT_SIG_IGNORE_MASK;
	sigprocmask(SIG_BLOCK, &ignore, &blocked);
	pid = err = local_kernel_thread(dumpfn, (void*)pfd, SIGCHLD, 0);
	if (err < 0) {
		eprintk_ctx("iptables local_kernel_thread: %d\n", err);
		goto out;
	}
	f = fget(pfd[1]);
	sc_close(pfd[1]);
	sc_close(pfd[0]);

	ctx->file->f_pos = sec + v.cpt_hdrlen;
	end = sec + v.cpt_next;
	do {
		char *p;
		char buf[16];

		n = end - ctx->file->f_pos;
		if (n > sizeof(buf))
			n = sizeof(buf);

		if (ctx->read(buf, n, ctx))
			break;
		if ((p = memchr(buf, 0, n)) != NULL)
			n = p - buf;
		oldfs = get_fs(); set_fs(KERNEL_DS);
		f->f_op->write(f, buf, n, &f->f_pos);
		set_fs(oldfs);
	} while (ctx->file->f_pos < end);

	fput(f);

	oldfs = get_fs(); set_fs(KERNEL_DS);
	if ((err = sc_waitx(pid, 0, &status)) < 0)
		eprintk_ctx("wait4: %d\n", err);
	else if ((status & 0x7f) == 0) {
		err = (status & 0xff00) >> 8;
		if (err != 0) {
			eprintk_ctx("iptables-restore exited with %d\n", err);
			eprintk_ctx("Most probably some iptables modules are not loaded\n");
			err = -EINVAL;
		}
	} else {
		eprintk_ctx("iptables-restore terminated\n");
		err = -EINVAL;
	}
	set_fs(oldfs);
	sigprocmask(SIG_SETMASK, &blocked, NULL);

	return err;

out:
	if (pfd[1] >= 0)
		sc_close(pfd[1]);
	if (pfd[0] >= 0)
		sc_close(pfd[0]);
	sigprocmask(SIG_SETMASK, &blocked, NULL);
	return err;
}

int rst_restore_net(struct cpt_context *ctx)
{
	int err;

	err = rst_restore_netdev(ctx);
	if (!err)
		err = rst_restore_ifaddr(ctx);
	if (!err)
		err = rst_restore_route(ctx);
	if (!err)
		err = rst_restore_iptables(ctx);
	if (!err)
		err = rst_restore_ip_conntrack(ctx);
	return err;
}
