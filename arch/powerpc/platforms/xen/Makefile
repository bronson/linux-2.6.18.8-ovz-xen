obj-y	+= gnttab.o
obj-y	+= hcall.o
obj-y	+= reboot.o
obj-y	+= setup.o
obj-y	+= smp.o
obj-y	+= time.o
obj-y	+= udbg_xen.o
obj-y	+= xen_guest.o
obj-y	+= xencomm.o

# we need the latest __XEN_INTERFACE_VERSION__ (see xen-compat.h)
CFLAGS_hcall.o += -D__XEN_TOOLS__

ifndef CONFIG_XEN_BALLOON
obj-y += balloon.o
endif

ifndef CONFIG_XEN_UTIL
obj-y	+= util.o
endif
