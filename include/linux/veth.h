/*
 *  include/linux/veth.h
 *
 *  Copyright (C) 2007  SWsoft
 *  All rights reserved.
 *  
 *  Licensing governed by "linux/COPYING.SWsoft" file.
 *
 */

#ifndef _VETH_H
#define _VETH_H

struct veth_struct
{
	struct net_device_stats stats;
	struct net_device	*pair;
	struct list_head	hwaddr_list;
	struct net_device_stats	*real_stats;
	int			allow_mac_change;
};

#define veth_from_netdev(dev) \
	((struct veth_struct *)(netdev_priv(dev)))
#define veth_to_netdev(veth) \
	((struct net_device*)((char*)veth - \
	(unsigned long)netdev_priv(NULL)))

static inline struct net_device_stats *
veth_stats(struct net_device *dev, int cpuid)
{
	return per_cpu_ptr(veth_from_netdev(dev)->real_stats, cpuid);
}

#endif
