#
# User resources part (UBC)
#
# Copyright (C) 2005  SWsoft
# All rights reserved.
#
# Licensing governed by "linux/COPYING.SWsoft" file.

obj-y := ub_sys.o beancounter.o ub_dcache.o ub_mem.o ub_misc.o \
	 ub_pages.o ub_stat.o ub_oom.o

obj-$(CONFIG_NET) += ub_net.o
obj-$(CONFIG_USER_RSS_ACCOUNTING) += ub_page_bc.o
obj-$(CONFIG_USER_RESOURCE_PROC)  += ub_proc.o
obj-$(CONFIG_UBC_IO_ACCT) += io_acct.o
obj-$(CONFIG_UBC_IO_PRIO) += io_prio.o
