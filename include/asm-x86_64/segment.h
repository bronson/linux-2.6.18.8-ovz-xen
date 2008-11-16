#ifndef _ASM_SEGMENT_H
#define _ASM_SEGMENT_H

#include <asm/cache.h>

#define GDT_ENTRY_BOOT_CS		2
#define __BOOT_CS	(GDT_ENTRY_BOOT_CS * 8)
#define GDT_ENTRY_BOOT_DS		3
#define __BOOT_DS	(GDT_ENTRY_BOOT_DS * 8)
#define GDT_ENTRY_TSS 4	/* needs two entries */
/* 
 * we cannot use the same code segment descriptor for user and kernel
 * -- not even in the long flat mode, because of different DPL /kkeil 
 * The segment offset needs to contain a RPL. Grr. -AK
 * GDT layout to get 64bit syscall right (sysret hardcodes gdt offsets) 
 */
#define GDT_ENTRY_TLS_MIN 6
#define GDT_ENTRY_TLS_MAX 8

#define GDT_ENTRY_LDT 9 /* needs two entries */
#define __KERNEL32_CS   0x58	/* 11*8 */
#define __KERNEL_CS	0x60	/* 12*8 */
#define __KERNEL_DS	0x68	/* 13*8 */
#define __USER32_CS   0x73   /* 14*8+3 */ 
#define __USER_DS     0x7b   /* 15*8+3 */ 
#define __USER32_DS	__USER_DS 
#define __USER_CS     0x83   /* 16*8+3 */ 

#define GDT_ENTRY_TLS_ENTRIES 3

/* TLS indexes for 64bit - hardcoded in arch_prctl */
#define FS_TLS 0	
#define GS_TLS 1	

#define GS_TLS_SEL ((GDT_ENTRY_TLS_MIN+GS_TLS)*8 + 3)
#define FS_TLS_SEL ((GDT_ENTRY_TLS_MIN+FS_TLS)*8 + 3)

#define IDT_ENTRIES 256
#define GDT_ENTRIES 32
#define GDT_SIZE (GDT_ENTRIES * 8)
#define TLS_SIZE (GDT_ENTRY_TLS_ENTRIES * 8) 

#endif
