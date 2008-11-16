/* Kernel module to match NFMARK values. */

/* (C) 1999-2001 Marc Boucher <marc@mbsi.ca>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/skbuff.h>

#include <linux/netfilter/xt_mark.h>
#include <linux/netfilter/x_tables.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marc Boucher <marc@mbsi.ca>");
MODULE_DESCRIPTION("iptables mark matching module");
MODULE_ALIAS("ipt_mark");
MODULE_ALIAS("ip6t_mark");

static int
match(const struct sk_buff *skb,
      const struct net_device *in,
      const struct net_device *out,
      const struct xt_match *match,
      const void *matchinfo,
      int offset,
      unsigned int protoff,
      int *hotdrop)
{
	const struct xt_mark_info *info = matchinfo;

	return ((skb->nfmark & info->mask) == info->mark) ^ info->invert;
}

static int
checkentry(const char *tablename,
           const void *entry,
	   const struct xt_match *match,
           void *matchinfo,
           unsigned int matchsize,
           unsigned int hook_mask)
{
	const struct xt_mark_info *minfo = matchinfo;

	if (minfo->mark > 0xffffffff || minfo->mask > 0xffffffff) {
		printk(KERN_WARNING "mark: only supports 32bit mark\n");
		return 0;
	}
	return 1;
}

#ifdef CONFIG_COMPAT
static int mark_match_compat_to_user(void *match, void **dstptr,
		int *size, int off)
{
	struct xt_entry_match *pm;
	struct xt_mark_info *pinfo;
	struct compat_xt_mark_info rinfo;
	u_int16_t msize;

	pm = (struct xt_entry_match *)match;
	msize = pm->u.user.match_size;
	if (__copy_to_user(*dstptr, pm, sizeof(struct compat_xt_entry_match)))
		return -EFAULT;
	pinfo = (struct xt_mark_info *)pm->data;
	memset(&rinfo, 0, sizeof(struct compat_xt_mark_info));
	/* mark & mask fit in 32bit due to check in checkentry() */
	rinfo.mark = (compat_ulong_t)pinfo->mark;
	rinfo.mask = (compat_ulong_t)pinfo->mask;
	rinfo.invert = pinfo->invert;
	if (__copy_to_user(*dstptr + sizeof(struct compat_xt_entry_match),
			&rinfo, sizeof(struct compat_xt_mark_info)))
		return -EFAULT;
	msize -= off;
	if (put_user(msize, (u_int16_t *)*dstptr))
		return -EFAULT;
	*size -= off;
	*dstptr += msize;
	return 0;
}

static int mark_match_compat_from_user(void *match, void **dstptr,
		int *size, int off)
{
	struct compat_xt_entry_match *pm;
	struct xt_entry_match *dstpm;
	struct compat_xt_mark_info *pinfo;
	struct xt_mark_info rinfo;
	u_int16_t msize;

	pm = (struct compat_xt_entry_match *)match;
	dstpm = (struct xt_entry_match *)*dstptr;
	msize = pm->u.user.match_size;
	memset(*dstptr, 0, sizeof(struct xt_entry_match));
	memcpy(*dstptr, pm, sizeof(struct compat_xt_entry_match));

	pinfo = (struct compat_xt_mark_info *)pm->data;
	memset(&rinfo, 0, sizeof(struct xt_mark_info));
	rinfo.mark = pinfo->mark;
	rinfo.mask = pinfo->mask;
	rinfo.invert = pinfo->invert;

	memcpy(*dstptr + sizeof(struct xt_entry_match),
				&rinfo, sizeof(struct xt_mark_info));
	msize += off;
	dstpm->u.user.match_size = msize;
	*size += off;
	*dstptr += msize;
	return 0;
}

static int mark_match_compat(void *match, void **dstptr,
		int *size, int convert)
{
	int ret, off;

	off = XT_ALIGN(sizeof(struct xt_mark_info)) -
		COMPAT_XT_ALIGN(sizeof(struct compat_xt_mark_info));
	switch (convert) {
		case COMPAT_TO_USER:
			ret = mark_match_compat_to_user(match,
					dstptr, size, off);
			break;
		case COMPAT_FROM_USER:
			ret = mark_match_compat_from_user(match,
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

static struct xt_match mark_match = {
	.name		= "mark",
	.match		= match,
	.matchsize	= sizeof(struct xt_mark_info),
	.checkentry	= checkentry,
#ifdef CONFIG_COMPAT
	.compat		= mark_match_compat,
#endif
	.family		= AF_INET,
	.me		= THIS_MODULE,
};

static struct xt_match mark6_match = {
	.name		= "mark",
	.match		= match,
	.matchsize	= sizeof(struct xt_mark_info),
	.checkentry	= checkentry,
#ifdef CONFIG_COMPAT
	.compat		= mark_match_compat,
#endif
	.family		= AF_INET6,
	.me		= THIS_MODULE,
};

static int __init xt_mark_init(void)
{
	int ret;
	ret = xt_register_match(&mark_match);
	if (ret)
		return ret;

	ret = xt_register_match(&mark6_match);
	if (ret)
		xt_unregister_match(&mark_match);

	return ret;
}

static void __exit xt_mark_fini(void)
{
	xt_unregister_match(&mark_match);
	xt_unregister_match(&mark6_match);
}

module_init(xt_mark_init);
module_exit(xt_mark_fini);
