/******************************************************************************
 * hypercall.h
 * 
 * Linux-specific hypervisor handling.
 * 
 * Copyright (c) 2002-2004, K A Fraser
 * 
 * This file may be distributed separately from the Linux kernel, or
 * incorporated into other software packages, subject to the following license:
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Copyright 2007 IBM Corp.
 *
 * Authors: Hollis Blanchard <hollisb@us.ibm.com>
 *          Jimi Xenidis <jimix@watson.ibm.com>
 */

#ifndef __HYPERCALL_H__
#define __HYPERCALL_H__

#include <asm/hvcall.h>
#include <asm/page.h>
#include <xen/xencomm.h>
#include <xen/interface/xen.h>
#include <xen/interface/sched.h>

#define XEN_MARK(a)((a) | (~0UL << 16))

extern int HYPERVISOR_console_io(int cmd, int count, char *str);
extern int HYPERVISOR_event_channel_op(int cmd, void *op);
extern int HYPERVISOR_xen_version(int cmd, void *arg);
extern int HYPERVISOR_physdev_op(int cmd, void *op);
extern int HYPERVISOR_grant_table_op(unsigned int cmd, void *uop,
		unsigned int count);
extern int HYPERVISOR_vcpu_op(int cmd, int vcpuid, void *extra_args);
extern int HYPERVISOR_memory_op(unsigned int cmd, void *arg);
extern int HYPERVISOR_multicall(void *call_list, int nr_calls);

extern int HYPERVISOR_sched_op(int cmd, void *arg);
extern int HYPERVISOR_poll(
	evtchn_port_t *ports, unsigned int nr_ports, u64 timeout);

static inline int HYPERVISOR_shutdown(unsigned int reason)
{
	struct sched_shutdown sched_shutdown = {
		.reason = reason
	};

	return HYPERVISOR_sched_op(SCHEDOP_shutdown, &sched_shutdown);
}

static inline int HYPERVISOR_set_timer_op(unsigned long arg)
{
	return plpar_hcall_norets(XEN_MARK(__HYPERVISOR_set_timer_op), arg);
}

extern int HYPERVISOR_suspend(unsigned long srec);
extern int HYPERVISOR_kexec_op(unsigned long op, void *args);
static inline unsigned long HYPERVISOR_hvm_op(int op, void *arg) {
	return -ENOSYS;
}

static inline int
HYPERVISOR_mmu_update(
	mmu_update_t *req, int count, int *success_count, domid_t domid)
{
	return -ENOSYS;
}

struct privcmd_hypercall;
extern int privcmd_hypercall(struct privcmd_hypercall *hypercall);

#endif	/*  __HYPERCALL_H__ */
