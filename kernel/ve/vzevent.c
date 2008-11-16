#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/errno.h>
#include <linux/ve_proto.h>
#include <linux/vzevent.h>

#define NETLINK_UEVENT	31
#define VZ_EVGRP_ALL	0x01

/*
 * NOTE: the original idea was to send events via kobject_uevent(),
 * however, it turns out that it has negative consequences like
 * start of /sbin/hotplug which tries to react on our events in inadequate manner.
 */

static struct sock *vzev_sock;

static char *action_to_string(int action)
{
	switch (action) {
	case KOBJ_MOUNT:
		return "ve-mount";
	case KOBJ_UMOUNT:
		return "ve-umount";
	case KOBJ_START:
		return "ve-start";
	case KOBJ_STOP:
		return "ve-stop";
	default:
		return NULL;
	}
}

static int do_vzevent_send(int event, char *msg, int len)
{
	struct sk_buff *skb;
	char *buf, *action;
	int alen;

	action = action_to_string(event);
	alen = strlen(action);

	skb = alloc_skb(len + 1 + alen, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	buf = skb_put(skb, len + 1 + alen);
	memcpy(buf, action, alen);
	buf[alen] = '@';
	memcpy(buf + alen + 1, msg, len);
	(void)netlink_broadcast(vzev_sock, skb, 0, VZ_EVGRP_ALL, GFP_KERNEL);
	return 0;
}

int vzevent_send(int event, const char *attrs_fmt, ...)
{
	va_list args;
	int len, err;
	struct ve_struct *ve;
	char *page;

	err = -ENOMEM;
	page = (char *)__get_free_page(GFP_KERNEL);
	if (!page)
		goto out;

	va_start(args, attrs_fmt);
	len = vscnprintf(page, PAGE_SIZE, attrs_fmt, args);
	va_end(args);

	ve = set_exec_env(get_ve0());
	err = do_vzevent_send(event, page, len);
	(void)set_exec_env(ve);
	free_page((unsigned long)page);
out:
	return err;
}
EXPORT_SYMBOL(vzevent_send);

static int ve_start(void *data)
{
	struct ve_struct *ve;

	ve = (struct ve_struct *)data;
	vzevent_send(KOBJ_START, "%d", ve->veid);
	return 0;
}

static void ve_stop(void *data)
{
	struct ve_struct *ve;

	ve = (struct ve_struct *)data;
	vzevent_send(KOBJ_STOP, "%d", ve->veid);
}

static struct ve_hook ve_start_stop_hook = {
	.init		= ve_start,
	.fini		= ve_stop,
	.owner		= THIS_MODULE,
	.priority	= HOOK_PRIO_AFTERALL,
};

static int __init init_vzevent(void)
{
	vzev_sock = netlink_kernel_create(NETLINK_UEVENT, 0, NULL, THIS_MODULE);
	if (vzev_sock == NULL)
		return -ENOMEM;
	ve_hook_register(VE_SS_CHAIN, &ve_start_stop_hook);
	return 0;
}

static void __exit exit_vzevent(void)
{
	ve_hook_unregister(&ve_start_stop_hook);
	sock_release(vzev_sock->sk_socket);
}

MODULE_LICENSE("GPL");

module_init(init_vzevent);
module_exit(exit_vzevent);
