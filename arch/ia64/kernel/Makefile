#
# Makefile for the linux kernel.
#

extra-y	:= head.o init_task.o vmlinux.lds

obj-y := acpi.o entry.o efi.o efi_stub.o gate-data.o fsys.o ia64_ksyms.o irq.o irq_ia64.o	\
	 irq_lsapic.o ivt.o machvec.o pal.o patch.o process.o perfmon.o ptrace.o sal.o		\
	 salinfo.o semaphore.o setup.o signal.o sys_ia64.o time.o traps.o unaligned.o \
	 unwind.o mca.o mca_asm.o topology.o

obj-$(CONFIG_IA64_BRL_EMU)	+= brl_emu.o
obj-$(CONFIG_IA64_GENERIC)	+= acpi-ext.o
obj-$(CONFIG_IA64_HP_ZX1)	+= acpi-ext.o
obj-$(CONFIG_IA64_HP_ZX1_SWIOTLB) += acpi-ext.o

ifneq ($(CONFIG_ACPI_PROCESSOR),)
obj-y				+= acpi-processor.o
endif

obj-$(CONFIG_IA64_PALINFO)	+= palinfo.o
obj-$(CONFIG_IOSAPIC)		+= iosapic.o
obj-$(CONFIG_MODULES)		+= module.o
obj-$(CONFIG_SMP)		+= smp.o smpboot.o
obj-$(CONFIG_NUMA)		+= numa.o
obj-$(CONFIG_PERFMON)		+= perfmon_default_smpl.o
obj-$(CONFIG_IA64_CYCLONE)	+= cyclone.o
obj-$(CONFIG_CPU_FREQ)		+= cpufreq/
obj-$(CONFIG_IA64_MCA_RECOVERY)	+= mca_recovery.o
obj-$(CONFIG_KPROBES)		+= kprobes.o jprobes.o
obj-$(CONFIG_KEXEC)		+= machine_kexec.o relocate_kernel.o crash.o
obj-$(CONFIG_CRASH_DUMP)	+= crash_dump.o
obj-$(CONFIG_IA64_UNCACHED_ALLOCATOR)	+= uncached.o
obj-$(CONFIG_AUDIT)		+= audit.o
mca_recovery-y			+= mca_drv.o mca_drv_asm.o

# The gate DSO image is built using a special linker script.
targets += gate.so gate-syms.o

extra-y += gate.so gate-syms.o gate.lds gate.o

# fp_emulate() expects f2-f5,f16-f31 to contain the user-level state.
CFLAGS_traps.o  += -mfixed-range=f2-f5,f16-f31

CPPFLAGS_gate.lds := -P -C -U$(ARCH)

quiet_cmd_gate = GATE $@
      cmd_gate = $(CC) -nostdlib $(GATECFLAGS_$(@F)) -Wl,-T,$(filter-out FORCE,$^) -o $@

GATECFLAGS_gate.so = -shared -s -Wl,-soname=linux-gate.so.1 \
		     $(call ld-option, -Wl$(comma)--hash-style=sysv)
$(obj)/gate.so: $(obj)/gate.lds $(obj)/gate.o FORCE
	$(call if_changed,gate)

$(obj)/built-in.o: $(obj)/gate-syms.o
$(obj)/built-in.o: ld_flags += -R $(obj)/gate-syms.o

GATECFLAGS_gate-syms.o = -r
$(obj)/gate-syms.o: $(obj)/gate.lds $(obj)/gate.o FORCE
	$(call if_changed,gate)

# gate-data.o contains the gate DSO image as data in section .data.gate.
# We must build gate.so before we can assemble it.
# Note: kbuild does not track this dependency due to usage of .incbin
$(obj)/gate-data.o: $(obj)/gate.so

#
# gate page paravirtualization for xen
#
obj-$(CONFIG_XEN) += xengate-data.o

ifeq ($(CONFIG_XEN), y)
# The gate DSO image is built using a special linker script.
targets += xengate.so xengate-syms.o
endif

extra-$(CONFIG_XEN) += xengate.so xengate.lds xengate.o

AFLAGS_xengate.o += -D__XEN_IA64_VDSO_PARAVIRT
$(obj)/xengate.o: $(src)/gate.S FORCE
	$(call if_changed_dep,as_o_S)

CPPFLAGS_xengate.lds := -P -C -U$(ARCH) -D__XEN_IA64_VDSO_PARAVIRT
$(obj)/xengate.lds: $(src)/gate.lds.S
	$(call if_changed_dep,cpp_lds_S)

GATECFLAGS_xengate.so = -shared -s -Wl,-soname=linux-gate.so.1 \
		     $(call ld-option, -Wl$(comma)--hash-style=sysv)
$(obj)/xengate.so: $(obj)/xengate.lds $(obj)/xengate.o FORCE
	$(call if_changed,gate)

ifeq ($(CONFIG_XEN), y)
$(obj)/built-in.o: $(obj)/xengate-syms.o
$(obj)/built-in.o: ld_flags += -R $(obj)/xengate-syms.o
$(obj)/mca_recovery.o: $(obj)/gate-syms.o $(obj)/xengate-syms.o
endif

GATECFLAGS_xengate-syms.o = -r
$(obj)/xengate-syms.o: $(obj)/xengate.lds $(obj)/xengate.o FORCE
	$(call if_changed,gate)
$(obj)/xengate-data.o: $(obj)/xengate.so

#
# .tmp_gate.o to calculate padding size for __kernel_syscall_via_epc
#
extra-$(CONFIG_XEN) += gate-skip.s .tmp_gate.o

ifeq ($(CONFIG_XEN), y)
AFLAGS_gate.o += -D__KERNEL_SYSCALL_VIA_EPC_PADDING
$(obj)/gate.o: $(obj)/gate-skip.s FORCE
endif

$(obj)/.tmp_gate.o: $(src)/gate.S FORCE
	$(call if_changed_dep,as_o_S)

quiet_cmd_gate_size = GATE_SIZE	$@
      cmd_gate_size = $(NM) --extern-only --print-size $(obj)/xengate.o | \
	$(AWK) '/__kernel_syscall_via_epc/{printf "\t.skip 0x"$$2" - "}' > $@; \
	$(NM) --extern-only --print-size $(obj)/.tmp_gate.o | \
	$(AWK) '/__kernel_syscall_via_epc/{printf "0x"$$2"\n"}' >> $@

$(obj)/gate-skip.s: $(obj)/xengate.o $(obj)/.tmp_gate.o FORCE
	$(call if_changed,gate_size)
