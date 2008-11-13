/*
 * Support routines for Xen
 *
 * Copyright (C) 2005 Dan Magenheimer <dan.magenheimer@hp.com>
 */

#include <asm/processor.h>
#include <asm/asmmacro.h>

	.section .data.read_mostly
	.align 8
	.global running_on_xen
running_on_xen:
	data4 0
	.previous

#define isBP	p3	// are we the Bootstrap Processor?

	.text
GLOBAL_ENTRY(early_xen_setup)
	mov r8=ar.rsc		// Initialized in head.S
(isBP)	movl r9=running_on_xen;;
	extr.u r8=r8,2,2;;	// Extract pl fields
	cmp.eq p7,p0=r8,r0	// p7: !running on xen
	mov r8=1		// booleanize.
(p7)	br.ret.sptk.many rp;;
(isBP)	st4 [r9]=r8
	movl r10=xen_ivt;;
	
	mov cr.iva=r10

	/* Set xsi base.  */
#define FW_HYPERCALL_SET_SHARED_INFO_VA			0x600
(isBP)	mov r2=FW_HYPERCALL_SET_SHARED_INFO_VA
(isBP)	movl r28=XSI_BASE;;
(isBP)	break 0x1000;;

	br.ret.sptk.many rp
	;;
END(early_xen_setup)
