/*
 * xencomm suspend support
 * Support routines for Xen
 *
 * Copyright (C) 2005 Dan Magenheimer <dan.magenheimer@hp.com>
 */
#include <asm/asmmacro.h>
#include <xen/interface/xen.h>

/*
 * Stub for suspend.
 * Just force the stacked registers to be written in memory.
 */
GLOBAL_ENTRY(xencomm_arch_hypercall_suspend)
	;; 
	alloc r20=ar.pfs,0,0,6,0
	mov r2=__HYPERVISOR_sched_op
	;; 
	/* We don't want to deal with RSE.  */
	flushrs
	mov r33=r32
	mov r32=2 // SCHEDOP_shutdown
	;;
	break 0x1000
	;; 
	br.ret.sptk.many b0
END(xencomm_arch_hypercall_suspend)
