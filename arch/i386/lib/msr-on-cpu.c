#include <linux/module.h>
#include <linux/preempt.h>
#include <linux/smp.h>
#include <linux/types.h>

struct msr_info {
	unsigned int cpu;
	u32 msr_no;
	u32 l, h;
};

static void __rdmsr_on_cpu(void *info)
{
	struct msr_info *rv = info;

	if (smp_processor_id() == rv->cpu)
		rdmsr(rv->msr_no, rv->l, rv->h);
}

void rdmsr_on_cpu(unsigned int cpu, u32 msr_no, u32 *l, u32 *h)
{
	preempt_disable();
	if (smp_processor_id() == cpu)
		rdmsr(msr_no, *l, *h);
	else {
		struct msr_info rv;

		rv.cpu = cpu;
		rv.msr_no = msr_no;
		smp_call_function(__rdmsr_on_cpu, &rv, 0, 1);
		*l = rv.l;
		*h = rv.h;
	}
	preempt_enable();
}

static void __wrmsr_on_cpu(void *info)
{
	struct msr_info *rv = info;

	if (smp_processor_id() == rv->cpu)
		wrmsr(rv->msr_no, rv->l, rv->h);
}

void wrmsr_on_cpu(unsigned int cpu, u32 msr_no, u32 l, u32 h)
{
	preempt_disable();
	if (smp_processor_id() == cpu)
		wrmsr(msr_no, l, h);
	else {
		struct msr_info rv;

		rv.cpu = cpu;
		rv.msr_no = msr_no;
		rv.l = l;
		rv.h = h;
		smp_call_function(__wrmsr_on_cpu, &rv, 0, 1);
	}
	preempt_enable();
}

EXPORT_SYMBOL(rdmsr_on_cpu);
EXPORT_SYMBOL(wrmsr_on_cpu);
