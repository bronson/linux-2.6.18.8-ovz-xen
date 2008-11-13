#ifndef _ASM_IA64_KEXEC_H
#define _ASM_IA64_KEXEC_H


/* Maximum physical address we can use pages from */
#define KEXEC_SOURCE_MEMORY_LIMIT (-1UL)
/* Maximum address we can reach in physical address mode */
#define KEXEC_DESTINATION_MEMORY_LIMIT (-1UL)
/* Maximum address we can use for the control code buffer */
#define KEXEC_CONTROL_MEMORY_LIMIT TASK_SIZE

#define KEXEC_CONTROL_CODE_SIZE (8192 + 8192 + 4096)

/* The native architecture */
#define KEXEC_ARCH KEXEC_ARCH_IA_64

#define MAX_NOTE_BYTES 1024

#define kexec_flush_icache_page(page) do { \
                unsigned long page_addr = (unsigned long)page_address(page); \
                flush_icache_range(page_addr, page_addr + PAGE_SIZE); \
        } while(0)

extern struct kimage *ia64_kimage;
DECLARE_PER_CPU(u64, ia64_mca_pal_base);
const extern unsigned int relocate_new_kernel_size;
extern void relocate_new_kernel(unsigned long, unsigned long,
		struct ia64_boot_param *, unsigned long);
static inline void
crash_setup_regs(struct pt_regs *newregs, struct pt_regs *oldregs)
{
}
extern struct resource efi_memmap_res;
extern struct resource boot_param_res;
extern void kdump_smp_send_stop(void);
extern void kdump_smp_send_init(void);
extern void kexec_disable_iosapic(void);
extern void crash_save_this_cpu(void);
struct rsvd_region;
extern unsigned long kdump_find_rsvd_region(unsigned long size,
		struct rsvd_region *rsvd_regions, int n);
extern void kdump_cpu_freeze(struct unw_frame_info *info, void *arg);
extern int kdump_status[];
extern atomic_t kdump_cpu_freezed;
extern atomic_t kdump_in_progress;

/* Kexec needs to know about the actual physical addresss.
 * But in xen, on some architectures, a physical address is a
 * pseudo-physical addresss. */
#ifdef CONFIG_XEN
#define KEXEC_ARCH_HAS_PAGE_MACROS
#define kexec_page_to_pfn(page)  pfn_to_mfn_for_dma(page_to_pfn(page))
#define kexec_pfn_to_page(pfn)   pfn_to_page(mfn_to_pfn_for_dma(pfn))
#define kexec_virt_to_phys(addr) phys_to_machine_for_dma(__pa(addr))
#define kexec_phys_to_virt(addr) phys_to_virt(machine_to_phys_for_dma(addr))
#endif

#endif /* _ASM_IA64_KEXEC_H */
