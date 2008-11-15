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

#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/bootmem.h>
#include <linux/irq.h>
#include <linux/smp.h>
#include <xen/interface/xen.h>
#include <xen/interface/vcpu.h>
#include <xen/evtchn.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <asm/hypervisor.h>
#include "setup.h"

#undef DEBUG

#ifdef DEBUG
#define DBG(fmt...) printk(KERN_EMERG fmt)
#else
#define DBG(fmt...)
#endif

static inline void *xen_of_alloc(ulong size)
{
	if (mem_init_done)
		return kmalloc(size, GFP_KERNEL);
	return alloc_bootmem(size);
}
static inline void xen_of_free(void *ptr)
{
	/* if this happens with the boot allocator then we are screwed */
	BUG_ON(!mem_init_done);
	kfree(ptr);
}

static struct property *dup_prop(struct property *op)
{
	struct property *np;
	void *p;
	ulong sz;


	/* allocate everything in one go in case it fails */
	sz = sizeof (*np); /* prop node */
	sz += strlen(op->name) + 1; /* prop name */
	sz += op->length; /* prop value */
		
	p = xen_of_alloc(sz);
	if (!p)
		return NULL;
	memset(p, 0, sz);

	/* prop node first */
	np = p;
	p += sizeof (*np);

	/* value next becuase we want it aligned */
	np->value = p;
	p += op->length;

	/* name */
	np->name = p;

	/* copy it all */
	strcpy(np->name, op->name);
	np->length = op->length;
	memcpy(np->value, op->value, np->length);

	return np;
}

static int dup_properties(struct device_node *dst, struct device_node *src)
{
	struct property *op;
	struct property *np;
	struct property *lp;
	int rc = 0;

	DBG("%s: duping to new cpu node: %s\n", __func__, dst->full_name);

	np = lp = NULL;
	for (op = src->properties; op != 0; op = op->next) {
		lp = np;
		np = dup_prop(op);
		if (!np)
			break;

		prom_add_property(dst, np);
	}

	if (!np) {
		DBG("%s: FAILED duping: %s\n", __func__, dst->full_name);
		/* we could not allocate enuff so free what we have
		 * allocated */
		rc = -ENOMEM;
		for (op = dst->properties; lp && op != lp; op = op->next)
			xen_of_free(op);
	}

	return rc;
}

/* returns added device node so it can be added to procfs in the case
 * of hotpluging */
static struct device_node *xen_add_vcpu_node(struct device_node *boot_cpu,
					     uint cpu)
{
	struct device_node *new_cpu;
	struct property *pp;
	void *p;
	int sz;
	int type_sz;
	int name_sz;

	DBG("%s: boot cpu: %s\n", __func__, boot_cpu->full_name);

	/* allocate in one shot in case we fail */
	name_sz = strlen(boot_cpu->name) + 1;
	type_sz = strlen(boot_cpu->type) + 1;

	sz = sizeof (*new_cpu);	/* the node */
	sz += strlen(boot_cpu->full_name) + 3; /* full_name */
	sz += name_sz; /* name */
	sz += type_sz; /* type */

	p = xen_of_alloc(sz);
	if (!p)
		return NULL;
	memset(p, 0, sz);

	/* the node */
	new_cpu = p;
	p += sizeof (*new_cpu);
	
	/* name */
	new_cpu->name = p;
	strcpy(new_cpu->name, boot_cpu->name);
	p += name_sz;
	
	/* type */
	new_cpu->type = p;
	strcpy(new_cpu->type, boot_cpu->type);
	p += type_sz;

	/* full_name */
	new_cpu->full_name = p;

	/* assemble new full_name */
	pp = of_find_property(boot_cpu, "name", NULL);
	if (!pp)
		panic("%s: no name prop\n", __func__);

	DBG("%s: name is: %s = %s\n", __func__, pp->name, pp->value);
	sprintf(new_cpu->full_name, "/cpus/%s@%u", pp->value, cpu);

	if (dup_properties(new_cpu, boot_cpu)) {
		xen_of_free(new_cpu);
		return NULL;
	}

	/* fixup reg property */
	DBG("%s: updating reg: %d\n", __func__, cpu);
	pp = of_find_property(new_cpu, "reg", NULL);
	if (!pp)
		panic("%s: no reg prop\n", __func__);
	*(int *)pp->value = cpu;

	if (mem_init_done)
		OF_MARK_DYNAMIC(new_cpu);

	kref_init(&new_cpu->kref);

	/* insert the node */
	new_cpu->parent = of_get_parent(boot_cpu);
	of_attach_node(new_cpu);
	of_node_put(new_cpu->parent);

	return new_cpu;
}

static void cpu_initialize_context(unsigned int vcpu, ulong entry)
{
	vcpu_guest_context_t ctxt;

	memset(&ctxt.user_regs, 0x55, sizeof(ctxt.user_regs));

	ctxt.user_regs.pc = entry;
	ctxt.user_regs.msr = 0;
	ctxt.user_regs.gprs[1] = 0; /* Linux uses its own stack */
	ctxt.user_regs.gprs[3] = vcpu;

	/* XXX verify this *** */
	/* There is a buggy kernel that does not zero the "local_paca", so
	 * we must make sure this register is 0 */
	ctxt.user_regs.gprs[13] = 0;

	DBG("%s: initializing vcpu: %d\n", __func__, vcpu);

	if (HYPERVISOR_vcpu_op(VCPUOP_initialise, vcpu, &ctxt))
		panic("%s: VCPUOP_initialise failed, vcpu: %d\n",
		       __func__, vcpu);

}

static int xen_start_vcpu(uint vcpu, ulong entry)
{
	DBG("%s: starting vcpu: %d\n", __func__, vcpu);

	cpu_initialize_context(vcpu, entry);

	DBG("%s: Spinning up vcpu: %d\n", __func__, vcpu);
	return HYPERVISOR_vcpu_op(VCPUOP_up, vcpu, NULL);
}

extern void __secondary_hold(void);
extern unsigned long __secondary_hold_spinloop;
extern unsigned long __secondary_hold_acknowledge;

static void xen_boot_secondary_vcpus(void)
{
	int vcpu;
	int rc;
	const unsigned long mark = (unsigned long)-1;
	unsigned long *spinloop = &__secondary_hold_spinloop;
	unsigned long *acknowledge = &__secondary_hold_acknowledge;
#ifdef CONFIG_PPC64
	/* __secondary_hold is actually a descriptor, not the text address */
	unsigned long secondary_hold = __pa(*(unsigned long *)__secondary_hold);
#else
	unsigned long secondary_hold = __pa(__secondary_hold);
#endif
	struct device_node *boot_cpu;

	DBG("%s: finding CPU node\n", __func__);
	boot_cpu = of_find_node_by_type(NULL, "cpu");
	if (!boot_cpu)
		panic("%s: Cannot find Booting CPU node\n", __func__);

	/* Set the common spinloop variable, so all of the secondary cpus
	 * will block when they are awakened from their OF spinloop.
	 * This must occur for both SMP and non SMP kernels, since OF will
	 * be trashed when we move the kernel.
	 */
	*spinloop = 0;

	DBG("%s: Searching for all vcpu numbers > 0\n", __func__);
	/* try and start as many as we can */
	for (vcpu = 1; vcpu < NR_CPUS; vcpu++) {
		int i;

		rc = HYPERVISOR_vcpu_op(VCPUOP_is_up, vcpu, NULL);
		if (rc < 0)
			continue;

		DBG("%s: Found vcpu: %d\n", __func__, vcpu);
		/* Init the acknowledge var which will be reset by
		 * the secondary cpu when it awakens from its OF
		 * spinloop.
		 */
		*acknowledge = mark;

		DBG("%s: Starting vcpu: %d at pc: 0x%lx\n", __func__,
		    vcpu, secondary_hold);
		rc = xen_start_vcpu(vcpu, secondary_hold);
		if (rc)
			panic("%s: xen_start_vpcu() failed\n", __func__);


		DBG("%s: Waiting for ACK on vcpu: %d\n", __func__, vcpu);
		for (i = 0; (i < 100000000) && (*acknowledge == mark); i++)
			mb();

		if (*acknowledge == vcpu)
			DBG("%s: Recieved for ACK on vcpu: %d\n",
			    __func__, vcpu);

		xen_add_vcpu_node(boot_cpu, vcpu);

		cpu_set(vcpu, cpu_present_map);
		set_hard_smp_processor_id(vcpu, vcpu);
	}
	of_node_put(boot_cpu);
	DBG("%s: end...\n", __func__);
}

static int __init smp_xen_probe(void)
{
	return cpus_weight(cpu_present_map);
}

static irqreturn_t xen_ppc_msg_reschedule(int irq, void *dev_id,
					  struct pt_regs *regs)
{
	smp_message_recv(PPC_MSG_RESCHEDULE, regs);
	return IRQ_HANDLED;
}

static irqreturn_t xen_ppc_msg_call_function(int irq, void *dev_id,
					     struct pt_regs *regs)
{
	smp_message_recv(PPC_MSG_CALL_FUNCTION, regs);
	return IRQ_HANDLED;
}

static irqreturn_t xen_ppc_msg_debugger_break(int irq, void *dev_id,
					  struct pt_regs *regs)
{
	smp_message_recv(PPC_MSG_DEBUGGER_BREAK, regs);
	return IRQ_HANDLED;
}

struct message {
	irqreturn_t (*f)(int, void *, struct pt_regs *);
	int num;
	char *name;
};
static struct message ipi_msgs[] = {
	{
		.num = PPC_MSG_RESCHEDULE,
		.f = xen_ppc_msg_reschedule,
		.name = "IPI-resched"
	},
	{
		.num = PPC_MSG_CALL_FUNCTION,
		.f = xen_ppc_msg_call_function,
		.name = "IPI-function"
		},
	{
		.num = PPC_MSG_DEBUGGER_BREAK,
		.f = xen_ppc_msg_debugger_break,
		.name = "IPI-debug"
	}
};

DECLARE_PER_CPU(int, ipi_to_irq[NR_IPIS]);

static void __devinit smp_xen_setup_cpu(int cpu)
{
	int irq;
	int i;
	const int nr_ipis = ARRAY_SIZE(__get_cpu_var(ipi_to_irq));

	/* big scary include web could mess with our values, so we
	 * make sure they are sane */
	BUG_ON(ARRAY_SIZE(ipi_msgs) > nr_ipis);

	for (i = 0; i < ARRAY_SIZE(ipi_msgs); i++) {
		BUG_ON(ipi_msgs[i].num >= nr_ipis);

		irq = bind_ipi_to_irqhandler(ipi_msgs[i].num,
					     cpu,
					     ipi_msgs[i].f,
					     SA_INTERRUPT,
					     ipi_msgs[i].name,
					     NULL);
		BUG_ON(irq < 0);
		per_cpu(ipi_to_irq, cpu)[ipi_msgs[i].num] = irq;
		DBG("%s: cpu: %d vector :%d irq: %d\n",
		       __func__, cpu, ipi_msgs[i].num, irq);
	}
}

static inline void send_IPI_one(unsigned int cpu, int vector)
{
	int irq;

	irq = per_cpu(ipi_to_irq, cpu)[vector];
	BUG_ON(irq < 0);

	DBG("%s: cpu: %d vector :%d irq: %d!\n",
	       __func__, cpu, vector, irq);
	DBG("%s: per_cpu[%p]: %d %d %d %d\n",
	       __func__, per_cpu(ipi_to_irq, cpu),
	       per_cpu(ipi_to_irq, cpu)[0],
	       per_cpu(ipi_to_irq, cpu)[1],
	       per_cpu(ipi_to_irq, cpu)[2],
	       per_cpu(ipi_to_irq, cpu)[3]);

	notify_remote_via_irq(irq);
}

static void smp_xen_message_pass(int target, int msg)
{
	int cpu;

	switch (msg) {
	case PPC_MSG_RESCHEDULE:
	case PPC_MSG_CALL_FUNCTION:
	case PPC_MSG_DEBUGGER_BREAK:
		break;
	default:
		panic("SMP %d: smp_message_pass: unknown msg %d\n",
		       smp_processor_id(), msg);
		return;
	}
	switch (target) {
	case MSG_ALL:
	case MSG_ALL_BUT_SELF:
		for_each_online_cpu(cpu) {
			if (target == MSG_ALL_BUT_SELF &&
			    cpu == smp_processor_id())
				continue;
			send_IPI_one(cpu, msg);
		}
		break;
	default:
		send_IPI_one(target, msg);
		break;
	}
}

static struct smp_ops_t xen_smp_ops = {
	.probe		= smp_xen_probe,
	.message_pass	= smp_xen_message_pass,
	.kick_cpu	= smp_generic_kick_cpu,
	.setup_cpu	= smp_xen_setup_cpu,
};

void xen_setup_smp(void)
{
	smp_ops = &xen_smp_ops;

	xen_boot_secondary_vcpus();
	smp_release_cpus();
}
