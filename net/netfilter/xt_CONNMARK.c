/* This kernel module is used to modify the connection mark values, or
 * to optionally restore the skb nfmark from the connection mark
 *
 * Copyright (C) 2002,2004 MARA Systems AB <http://www.marasystems.com>
 * by Henrik Nordstrom <hno@marasystems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <net/checksum.h>

MODULE_AUTHOR("Henrik Nordstrom <hno@marasytems.com>");
MODULE_DESCRIPTION("IP tables CONNMARK matching module");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ipt_CONNMARK");

#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_CONNMARK.h>
#include <net/netfilter/nf_conntrack_compat.h>

static unsigned int
target(struct sk_buff **pskb,
       const struct net_device *in,
       const struct net_device *out,
       unsigned int hooknum,
       const struct xt_target *target,
       const void *targinfo,
       void *userinfo)
{
	const struct xt_connmark_target_info *markinfo = targinfo;
	u_int32_t diff;
	u_int32_t nfmark;
	u_int32_t newmark;
	u_int32_t ctinfo;
	u_int32_t *ctmark = nf_ct_get_mark(*pskb, &ctinfo);

	if (ctmark) {
	    switch(markinfo->mode) {
	    case XT_CONNMARK_SET:
		newmark = (*ctmark & ~markinfo->mask) | markinfo->mark;
		if (newmark != *ctmark)
		    *ctmark = newmark;
		break;
	    case XT_CONNMARK_SAVE:
		newmark = (*ctmark & ~markinfo->mask) | ((*pskb)->nfmark & markinfo->mask);
		if (*ctmark != newmark)
		    *ctmark = newmark;
		break;
	    case XT_CONNMARK_RESTORE:
		nfmark = (*pskb)->nfmark;
		diff = (*ctmark ^ nfmark) & markinfo->mask;
		if (diff != 0)
		    (*pskb)->nfmark = nfmark ^ diff;
		break;
	    }
	}

	return XT_CONTINUE;
}

static int
checkentry(const char *tablename,
	   const void *entry,
	   const struct xt_target *target,
	   void *targinfo,
	   unsigned int targinfosize,
	   unsigned int hook_mask)
{
	struct xt_connmark_target_info *matchinfo = targinfo;

	if (matchinfo->mode == XT_CONNMARK_RESTORE) {
	    if (strcmp(tablename, "mangle") != 0) {
		    printk(KERN_WARNING "CONNMARK: restore can only be called from \"mangle\" table, not \"%s\"\n", tablename);
		    return 0;
	    }
	}

	if (matchinfo->mark > 0xffffffff || matchinfo->mask > 0xffffffff) {
		printk(KERN_WARNING "CONNMARK: Only supports 32bit mark\n");
		return 0;
	}

	return 1;
}

#ifdef CONFIG_COMPAT
static int connmark_reg_compat_to_user(void *target, void **dstptr,
		int *size, int off)
{
	struct xt_entry_target *pt;
	struct xt_connmark_target_info *pinfo;
	struct compat_xt_connmark_target_info rinfo;
	u_int16_t tsize;

	pt = (struct xt_entry_target *)target;
	tsize = pt->u.user.target_size;
	if (__copy_to_user(*dstptr, pt, sizeof(struct compat_xt_entry_target)))
		return -EFAULT;
	pinfo = (struct xt_connmark_target_info *)pt->data;
	memset(&rinfo, 0, sizeof(struct compat_xt_connmark_target_info));
	/* mark & mask fit in 32bit due to check in checkentry() */
	rinfo.mark = (compat_ulong_t)pinfo->mark;
	rinfo.mask = (compat_ulong_t)pinfo->mask;
	rinfo.mode = pinfo->mode;
	if (__copy_to_user(*dstptr + sizeof(struct compat_xt_entry_target),
			&rinfo, sizeof(struct compat_xt_connmark_target_info)))
		return -EFAULT;
	tsize -= off;
	if (put_user(tsize, (u_int16_t *)*dstptr))
		return -EFAULT;
	*size -= off;
	*dstptr += tsize;
	return 0;
}

static int connmark_reg_compat_from_user(void *target, void **dstptr,
		int *size, int off)
{
	struct compat_xt_entry_target *pt;
	struct xt_entry_target *dstpt;
	struct compat_xt_connmark_target_info *pinfo;
	struct xt_connmark_target_info rinfo;
	u_int16_t tsize;

	pt = (struct compat_xt_entry_target *)target;
	dstpt = (struct xt_entry_target *)*dstptr;
	tsize = pt->u.user.target_size;
	memset(*dstptr, 0, sizeof(struct xt_entry_target));
	memcpy(*dstptr, pt, sizeof(struct compat_xt_entry_target));

	pinfo = (struct compat_xt_connmark_target_info *)pt->data;
	memset(&rinfo, 0, sizeof(struct xt_connmark_target_info));
	rinfo.mark = pinfo->mark;
	rinfo.mask = pinfo->mask;
	rinfo.mode = pinfo->mode;

	memcpy(*dstptr + sizeof(struct xt_entry_target),
				&rinfo, sizeof(struct xt_connmark_target_info));
	tsize += off;
	dstpt->u.user.target_size = tsize;
	*size += off;
	*dstptr += tsize;
	return 0;
}

static int connmark_reg_compat(void *target, void **dstptr,
		int *size, int convert)
{
	int ret, off;

	off = XT_ALIGN(sizeof(struct xt_connmark_target_info)) -
		COMPAT_XT_ALIGN(sizeof(struct compat_xt_connmark_target_info));
	switch (convert) {
		case COMPAT_TO_USER:
			ret = connmark_reg_compat_to_user(target,
					dstptr, size, off);
			break;
		case COMPAT_FROM_USER:
			ret = connmark_reg_compat_from_user(target,
					dstptr, size, off);
			break;
		case COMPAT_CALC_SIZE:
			*size += off;
			ret = 0;
			break;
		default:
			ret = -ENOPROTOOPT;
			break;
	}
	return ret;
}
#endif /*CONFIG_COMPAT*/

static struct xt_target connmark_reg = {
	.name		= "CONNMARK",
	.target		= target,
	.targetsize	= sizeof(struct xt_connmark_target_info),
	.checkentry	= checkentry,
#ifdef CONFIG_COMPAT
	.compat		= connmark_reg_compat,
#endif
	.family		= AF_INET,
	.me		= THIS_MODULE
};

static struct xt_target connmark6_reg = {
	.name		= "CONNMARK",
	.target		= target,
	.targetsize	= sizeof(struct xt_connmark_target_info),
	.checkentry	= checkentry,
#ifdef CONFIG_COMPAT
	.compat		= connmark_reg_compat,
#endif
	.family		= AF_INET6,
	.me		= THIS_MODULE
};

static int __init xt_connmark_init(void)
{
	int ret;

	need_conntrack();

	ret = xt_register_target(&connmark_reg);
	if (ret)
		return ret;

	ret = xt_register_target(&connmark6_reg);
	if (ret)
		xt_unregister_target(&connmark_reg);

	return ret;
}

static void __exit xt_connmark_fini(void)
{
	xt_unregister_target(&connmark_reg);
	xt_unregister_target(&connmark6_reg);
}

module_init(xt_connmark_init);
module_exit(xt_connmark_fini);
