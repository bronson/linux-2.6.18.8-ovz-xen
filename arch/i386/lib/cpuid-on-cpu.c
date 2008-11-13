#include <linux/module.h>
#include <linux/preempt.h>
#include <linux/smp.h>
#include <linux/types.h>

struct cpuid_info {
	unsigned int cpu;
	u32 op;
	u32 eax, ebx, ecx, edx;
};

static void __cpuid_on_cpu(void *info)
{
	struct cpuid_info *rv = info;

	if (smp_processor_id() == rv->cpu)
		cpuid(rv->op, &rv->eax, &rv->ebx, &rv->ecx, &rv->edx);
}

void cpuid_on_cpu(unsigned int cpu, u32 op, u32 *eax, u32 *ebx, u32 *ecx, u32 *edx)
{
	preempt_disable();
	if (smp_processor_id() == cpu)
		cpuid(op, eax, ebx, ecx, edx);
	else {
		struct cpuid_info rv;

		rv.cpu = cpu;
		rv.op = op;
		smp_call_function(__cpuid_on_cpu, &rv, 0, 1);
		*eax = rv.eax;
		*ebx = rv.ebx;
		*ecx = rv.ecx;
		*edx = rv.edx;
	}
	preempt_enable();
}

struct cpuid_eax_info {
	unsigned int cpu;
	u32 op;
	u32 eax;
};

static void __cpuid_eax_on_cpu(void *info)
{
	struct cpuid_info *rv = info;

	if (smp_processor_id() == rv->cpu)
		rv->eax = cpuid_eax(rv->op);
}

u32 cpuid_eax_on_cpu(unsigned int cpu, u32 op)
{
	u32 ret;

	preempt_disable();
	if (smp_processor_id() == cpu)
		ret = cpuid_eax(op);
	else {
		struct cpuid_eax_info rv;

		rv.cpu = cpu;
		rv.op = op;
		smp_call_function(__cpuid_eax_on_cpu, &rv, 0, 1);
		ret = rv.eax;
	}
	preempt_enable();
	return ret;
}

EXPORT_SYMBOL(cpuid_on_cpu);
EXPORT_SYMBOL(cpuid_eax_on_cpu);
