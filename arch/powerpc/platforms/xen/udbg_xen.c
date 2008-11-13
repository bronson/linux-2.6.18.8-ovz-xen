/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * Copyright (C) IBM Corp. 2006
 *
 * Authors: Jimi Xenidis <jimix@watson.ibm.com>
 */

#include <linux/module.h>
#include <xen/interface/xen.h>
#include <xen/interface/io/console.h>
#include <xen/evtchn.h>
#include <asm/udbg.h>
#include <asm/hypervisor.h>
#include "setup.h"

static void udbg_xen_wait(void)
{
	evtchn_port_t port = 0;

	if (xen_start_info) {
		port = xen_start_info->console.domU.evtchn;
		clear_evtchn(port);
	}
	HYPERVISOR_poll(&port, 1, 10);
}

static int udbg_getc_xen(void)
{
	int ch;
	for (;;) {
		ch = udbg_getc_poll();
		if (ch == -1) {
			udbg_xen_wait();
		} else {
			return ch;
		}
	}
}

static void udbg_putc_dom0_xen(char c)
{
	unsigned long rc;

	if (c == '\n')
		udbg_putc_dom0_xen('\r');

	do {
		rc = HYPERVISOR_console_io(CONSOLEIO_write, 1, &c);
	} while (rc < 0);
}

/* Buffered chars getc */
static long inbuflen;
static char inbuf[128];	/* Xen serial ring buffer */

static int udbg_getc_poll_dom0_xen(void)
{
	/* The interface is tricky because it may return many chars.
	 * We save them statically for future calls to udbg_getc().
	 */
	char ch, *buf = (char *)inbuf;
	int i;

	if (inbuflen == 0) {
		/* get some more chars. */
		inbuflen = HYPERVISOR_console_io(CONSOLEIO_read,
						 sizeof(inbuf), buf);
	}

	if (inbuflen == 0)
		return -1;

	ch = buf[0];
	for (i = 1; i < inbuflen; i++)	/* shuffle them down. */
		buf[i-1] = buf[i];
	inbuflen--;

	return ch;
}

static struct xencons_interface *intf;

static void udbg_putc_domu_xen(char c)
{
	XENCONS_RING_IDX cons, prod;

	if (c == '\n')
		udbg_putc_domu_xen('\r');

	cons = intf->out_cons;
	prod = intf->out_prod;
	mb();

	if ((prod - cons) < sizeof(intf->out))
		intf->out[MASK_XENCONS_IDX(prod++, intf->out)] = c;

	wmb();
	intf->out_prod = prod;

	if (xen_start_info)
		notify_remote_via_evtchn(xen_start_info->console.domU.evtchn);
}

static int udbg_getc_poll_domu_xen(void)
{
	XENCONS_RING_IDX cons, prod;
	int c;

	mb();
	cons = intf->in_cons;
	prod = intf->in_prod;
	BUG_ON((prod - cons) > sizeof(intf->in));

	if (cons == prod)
		return -1;

	c = intf->in[MASK_XENCONS_IDX(cons++, intf->in)];
	wmb();
	intf->in_cons = cons;

	if (xen_start_info)
		notify_remote_via_evtchn(xen_start_info->console.domU.evtchn);

	return c;
}

void udbg_init_xen(void)
{
	ulong __console_mfn = 0;

	if (xen_start_info) {
		/* we can find out where everything is */
		if (!(xen_start_info->flags & SIF_INITDOMAIN))
			__console_mfn = xen_start_info->console.domU.mfn;
	} else {
		/* VERY early printf */
#ifdef CONFIG_PPC_EARLY_DEBUG_XEN_DOMU
		__console_mfn = 0x3ffdUL;
#endif
	}

	udbg_getc = udbg_getc_xen;
	if (__console_mfn == 0) {
		udbg_putc = udbg_putc_dom0_xen;
		udbg_getc_poll = udbg_getc_poll_dom0_xen;
	} else {
		udbg_putc = udbg_putc_domu_xen;
		udbg_getc_poll = udbg_getc_poll_domu_xen;
		intf = (struct xencons_interface *)mfn_to_virt(__console_mfn);
	}
}
