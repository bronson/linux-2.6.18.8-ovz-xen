# grsecurity's ACL system was originally written in 2001 by Michael Dalton
# during 2001-2005 it has been completely redesigned by Brad Spengler
# into an RBAC system
#
# All code in this directory and various hooks inserted throughout the kernel
# are copyright Brad Spengler, and released under the GPL v2 or higher

obj-y = grsec_tpe.o grsec_sysctl.o

obj-$(CONFIG_GRKERNSEC) += grsec_init.o gracl.o grsec_log.o

ifndef CONFIG_GRKERNSEC
obj-y += grsec_disabled.o
endif

