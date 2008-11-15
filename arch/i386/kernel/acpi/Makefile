obj-$(CONFIG_ACPI)		+= boot.o
obj-$(CONFIG_X86_IO_APIC)	+= earlyquirk.o
obj-$(CONFIG_ACPI_SLEEP)	+= sleep.o wakeup.o

ifneq ($(CONFIG_ACPI_PROCESSOR),)
obj-y				+= cstate.o processor.o
obj-$(CONFIG_XEN)		+= processor_extcntl_xen.o
endif

disabled-obj-$(CONFIG_XEN)	:= cstate.o wakeup.o
