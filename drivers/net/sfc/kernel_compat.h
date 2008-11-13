/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005-2006: Fen Systems Ltd.
 * Copyright 2005-2008: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Initially developed by Michael Brown <mbrown@fensystems.co.uk>
 * Maintained by Solarflare Communications <linux-net-drivers@solarflare.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation, incorporated herein by reference.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 ****************************************************************************
 */

#ifndef EFX_KERNEL_COMPAT_H
#define EFX_KERNEL_COMPAT_H

#include <linux/version.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include "extraversion.h"

/*
 * Kernel backwards compatibility
 *
 * This file provides macros that enable the driver to be compiled on
 * any kernel from 2.6.9 onward (plus SLES 9 2.6.5), without requiring
 * explicit version tests scattered throughout the code.
 */

/**************************************************************************
 *
 * Version/config/architecture tests to set feature flags
 *
 **************************************************************************
 *
 * NOTE: For simplicity, these initial version tests cover kernel.org
 * releases only.  Backported features in "enterprise" kernels are
 * handled further down.
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9) &&	 \
	!(defined(EFX_DIST_SUSE) &&			 \
	  LINUX_VERSION_CODE == KERNEL_VERSION(2,6,5) && \
	  EFX_DIST_KVER_LEVEL_1 == 7)
	#error "This kernel version is now unsupported"
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,6)
	#define EFX_NEED_RANDOM_ETHER_ADDR yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
	#define EFX_NEED_I2C_CLASS_HWMON yes
	#define EFX_NEED_IF_MII yes
	#define EFX_NEED_MSLEEP yes
	#define EFX_NEED_MSECS_TO_JIFFIES yes
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,8)
	#define EFX_USE_MTD_ERASE_FAIL_ADDR yes
#else
	#define EFX_NEED_MTD_ERASE_CALLBACK yes
	#define EFX_NEED_DUMMY_PCI_DISABLE_MSI yes
	#define EFX_NEED_DUMMY_MSIX yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
	#define EFX_NEED_BYTEORDER_TYPES yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
	#define EFX_NEED_MMIOWB yes
	#define EFX_NEED_PCI_SAVE_RESTORE_WRAPPERS yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
	#define EFX_NEED_DUMMY_SUPPORTS_GMII yes
	#define EFX_NEED_MII_CONSTANTS yes
	#define EFX_NEED_MII_ETHTOOL_FIX yes
	#define EFX_HAVE_MSIX_TABLE_RESERVED yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
	#define EFX_NEED_SCHEDULE_TIMEOUT_INTERRUPTIBLE yes
	#define EFX_NEED_SCHEDULE_TIMEOUT_UNINTERRUPTIBLE yes
	#define EFX_NEED_GFP_T yes
	#define EFX_NEED_KZALLOC yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
	#define EFX_NEED_SETUP_TIMER yes
	#ifdef CONFIG_HUGETLB_PAGE
		#define EFX_USE_COMPOUND_PAGES yes
	#endif
#else
	#define EFX_USE_COMPOUND_PAGES yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	#define EFX_NEED_MUTEX yes
	#define EFX_NEED_SAFE_LISTS yes
	#ifdef EFX_USE_COMPOUND_PAGES
		#define EFX_NEED_COMPOUND_PAGE_FIX yes
	#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17)
	#define EFX_NEED_UNREGISTER_NETDEVICE_NOTIFIER_FIX yes
	#define EFX_NEED_DEV_NOTICE yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
	#define EFX_NEED_IRQF_FLAGS yes
	#define EFX_NEED_NETDEV_ALLOC_SKB yes
	/* Fedora backported 2.6.18 netdevice.h changes */
	#ifndef NETIF_F_GSO
		#define EFX_NEED_NETIF_TX_LOCK yes
	#endif
#else
	#define EFX_USE_MTD_WRITESIZE yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	#define EFX_NEED_IRQ_HANDLER_T yes
	#define EFX_HAVE_IRQ_HANDLER_REGS yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	#define EFX_NEED_WORK_API_WRAPPERS yes
	#define EFX_USE_FASTCALL yes
	#define EFX_NEED_CSUM_UNFOLDED yes
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21)
	/*
	 * debugfs was introduced earlier, but only supports sym-links
	 * from 2.6.21
	 */
	#ifdef CONFIG_DEBUG_FS
		#define EFX_USE_DEBUGFS yes
	#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	#define EFX_NEED_SKB_HEADER_MACROS yes
	#define EFX_NEED_HEX_DUMP yes
#else
	#define EFX_USE_CANCEL_WORK_SYNC yes
#endif

#if LINUX_VERSION_CODE == KERNEL_VERSION(2,6,22)
	#define EFX_NEED_HEX_DUMP_CONST_FIX yes
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,14)) && \
    (LINUX_VERSION_CODE <  KERNEL_VERSION(2,6,23))
	#define EFX_USE_ETHTOOL_GET_PERM_ADDR yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
	#ifdef __ia64__
		/* csum_tcpudp_nofold() is extern but not exported */
		#define EFX_NEED_CSUM_TCPUDP_NOFOLD yes
	#endif
#else
	#define EFX_USE_PCI_DEV_REVISION yes
	#define EFX_USE_CANCEL_DELAYED_WORK_SYNC yes
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	#define EFX_HAVE_OLD_NAPI yes
	#define EFX_NEED_GENERIC_LRO yes
	#define EFX_NEED_PRINT_MAC yes
#else
	#define EFX_USE_ETHTOOL_FLAGS yes
#endif

/*
 * SFC Bug 4560: Some kernels leak IOMMU entries under heavy load.  Use a
 * spinlock to serialise access where possible to alleviate the
 * problem.
 *
 * NB. The following definition is duplicated in
 * the char driver.  Please keep in sync.
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17) && \
     defined(__x86_64__) && defined(CONFIG_SMP))
	#define EFX_NEED_IOMMU_LOCK yes
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
		#if defined(CONFIG_GART_IOMMU)
			#define EFX_NO_IOMMU no_iommu
		#else
			#define EFX_NO_IOMMU 1
		#endif
	#else
		#define EFX_NO_IOMMU 0
	#endif
#endif

#ifdef CONFIG_PPC64
	/* __raw_writel and friends are broken on ppc64 */
	#define EFX_NEED_RAW_READ_AND_WRITE_FIX yes
#endif

/**************************************************************************
 *
 * Exceptions for backported features
 *
 **************************************************************************
 */

/* RHEL4 */
#if defined(EFX_DIST_RHEL) && LINUX_VERSION_CODE == KERNEL_VERSION(2,6,9)
	#if EFX_DIST_KVER_LEVEL_1 >= 22
		/* linux-2.6.9-mmiowb.patch */
		#undef EFX_NEED_MMIOWB
	#endif
	#if EFX_DIST_KVER_LEVEL_1 >= 34
		/* linux-2.6.9-net-mii-update.patch */
		#undef EFX_NEED_DUMMY_SUPPORTS_GMII
		#undef EFX_NEED_MII_CONSTANTS
		#undef EFX_NEED_MII_ETHTOOL_FIX
		/* linux-2.6.9-gfp_t-typedef.patch */
		#undef EFX_NEED_GFP_T
		/* linux-2.6.9-slab-update.patch */
		#undef EFX_NEED_KZALLOC
	#endif
	#if EFX_DIST_KVER_LEVEL_1 >= 55
		/* linux-2.6.18-sata-update.patch (ported from 2.6.18->2.6.9) */
		#undef EFX_NEED_SCHEDULE_TIMEOUT_UNINTERRUPTIBLE
		#undef EFX_NEED_IRQ_HANDLER_T
	#endif
#endif

/* RHEL5 */
#if defined(EFX_DIST_RHEL) && LINUX_VERSION_CODE == KERNEL_VERSION(2,6,18)
	#if EFX_DIST_KVER_LEVEL_1 >= 53
		/* linux-2.6.18-sata-update.patch */
		#undef EFX_NEED_SCHEDULE_TIMEOUT_UNINTERRUPTIBLE
		#undef EFX_NEED_IRQ_HANDLER_T
	#endif
#endif

#if defined(EFX_DIST_RHEL)
	#if (LINUX_VERSION_CODE != KERNEL_VERSION(2,6,9)) && \
	     (LINUX_VERSION_CODE != KERNEL_VERSION(2,6,18))
		#error "Unknown Red Hat Enterprise kernel version"
	#endif
#endif

/* SLES9 */
#if defined(EFX_DIST_SUSE) && LINUX_VERSION_CODE == KERNEL_VERSION(2,6,5) && \
	EFX_DIST_KVER_LEVEL_1 == 7
	#if EFX_DIST_KVER_LEVEL_2 >= 139
		#undef EFX_NEED_MMIOWB
	#endif
	#if EFX_DIST_KVER_LEVEL_2 >= 191
		#undef EFX_NEED_MSLEEP
		#undef EFX_NEED_MSECS_TO_JIFFIES
	#endif
	#if EFX_DIST_KVER_LEVEL_2 >= 244
		#undef EFX_NEED_BYTEORDER_TYPES
	#endif
	#if EFX_DIST_KVER_LEVEL_2 >= 252
		#undef EFX_NEED_KZALLOC
	#endif
#endif

/**************************************************************************
 *
 * Definitions of missing constants, types, functions and macros
 *
 **************************************************************************
 *
 */

#ifndef DMA_40BIT_MASK
	#define DMA_40BIT_MASK	0x000000ffffffffffULL
#endif

#ifndef spin_trylock_irqsave
	#define spin_trylock_irqsave(lock, flags)	\
	({						\
		local_irq_save(flags);			\
		spin_trylock(lock) ?			\
		1 : ({local_irq_restore(flags); 0;});	\
	})
#endif

#ifndef raw_smp_processor_id
	#define raw_smp_processor_id() (current_thread_info()->cpu)
#endif

#ifndef NETIF_F_LRO
	#define NETIF_F_LRO 0
#endif

/* Cope with small changes in PCI constants between minor kernel revisions */
#if PCI_X_STATUS != 4
	#undef PCI_X_STATUS
	#define PCI_X_STATUS 4
	#undef PCI_X_STATUS_MAX_SPLIT
	#define PCI_X_STATUS_MAX_SPLIT 0x03800000
#endif

#ifndef PCI_EXP_LNKSTA
	#define PCI_EXP_LNKSTA		18	    /* Link Status */
#endif

/* Used for struct pt_regs */
#ifndef regs_return_value
	#if defined(__x86_64__)
		#define regs_return_value(regs) ((regs)->rax)
	#elif defined(__i386__)
		#define regs_return_value(regs) ((regs)->eax)
	#elif defined(__ia64__)
		#define regs_return_value(regs) ((regs)->r8)
	#else
		#error "Need definition for regs_return_value()"
	#endif
#endif

#ifndef __GFP_COMP
	#define __GFP_COMP 0
#endif

#ifndef __iomem
	#define __iomem
#endif

#ifndef NET_IP_ALIGN
	#define NET_IP_ALIGN 2
#endif

#ifndef PCI_CAP_ID_EXP
#define PCI_CAP_ID_EXP		0x10    /* PCI Express */
#endif

#ifndef PCI_EXP_FLAGS
#define PCI_EXP_FLAGS           2           /* Capabilities register */
#define PCI_EXP_FLAGS_TYPE      0x00f0      /* Capability version */
#define PCI_EXP_TYPE_ENDPOINT   0x0         /* Express Endpoint */
#define PCI_EXP_TYPE_LEG_END    0x1         /* Legacy Endpoint */
#define PCI_EXP_TYPE_ROOT_PORT  0x4         /* Root Port */
#endif

#ifndef PCI_EXP_DEVCAP
#define PCI_EXP_DEVCAP          4           /* Device capabilities */
#define PCI_EXP_DEVCAP_PAYLOAD  0x07        /* Max_Payload_Size */
#define PCI_EXP_DEVCAP_PWR_VAL  0x3fc0000   /* Slot Power Limit Value */
#define PCI_EXP_DEVCAP_PWR_SCL  0xc000000   /* Slot Power Limit Scale */
#endif

#ifndef PCI_EXP_DEVCTL
#define PCI_EXP_DEVCTL          8           /* Device Control */
#define PCI_EXP_DEVCTL_PAYLOAD  0x00e0      /* Max_Payload_Size */
#define PCI_EXP_DEVCTL_READRQ   0x7000      /* Max_Read_Request_Size */
#endif

#ifndef PCI_EXP_LNKSTA
#define PCI_EXP_LNKSTA		18	    /* Link Status */
#endif

#ifndef NETDEV_TX_OK
	#define NETDEV_TX_OK 0
#endif

#ifndef NETDEV_TX_BUSY
	#define NETDEV_TX_BUSY 1
#endif

#ifndef __force
	#define __force
#endif

#if ! defined(for_each_cpu_mask) && ! defined(CONFIG_SMP)
	#define for_each_cpu_mask(cpu, mask)            \
		for ((cpu) = 0; (cpu) < 1; (cpu)++, (void)mask)
#endif

/**************************************************************************/

#ifdef EFX_NEED_IRQ_HANDLER_T
	typedef irqreturn_t (*irq_handler_t)(int, void *, struct pt_regs *);
#endif

#ifdef EFX_NEED_I2C_CLASS_HWMON
	#define I2C_CLASS_HWMON (1<<0)
#endif

#ifdef EFX_NEED_MII_CONSTANTS
	#define MII_CTRL1000		0x09
	#define MII_STAT1000		0x0a
	#define BMCR_SPEED1000		0x0040
	#define ADVERTISE_PAUSE_ASYM	0x0800
	#define ADVERTISE_PAUSE_CAP	0x0400
	#define LPA_PAUSE_ASYM		0x0800
	#define LPA_PAUSE_CAP		0x0400
	#define ADVERTISE_1000FULL	0x0200
	#define ADVERTISE_1000HALF	0x0100
	#define LPA_1000FULL		0x0800
	#define LPA_1000HALF		0x0400
#endif

#ifdef EFX_NEED_DUMMY_SUPPORTS_GMII
	#include <linux/mii.h>
	/* Ugly; redirect nonexistent new field to an old unused field */
	#undef supports_gmii
	#define supports_gmii full_duplex
#endif

#ifdef EFX_NEED_SKB_HEADER_MACROS
	#define skb_mac_header(skb)	(skb)->mac.raw
	#define skb_network_header(skb) (skb)->nh.raw
	#define tcp_hdr(skb)		(skb)->h.th
	#define ip_hdr(skb)		(skb)->nh.iph
	#define skb_tail_pointer(skb)   (skb)->tail
#endif

#ifdef EFX_NEED_RAW_READ_AND_WRITE_FIX
	#include <asm/io.h>
	static inline void
	efx_raw_writeb(u8 value, volatile void __iomem *addr)
	{
		writeb(value, addr);
	}
	static inline void
	efx_raw_writew(u16 value, volatile void __iomem *addr)
	{
		writew(le16_to_cpu(value), addr);
	}
	static inline void
	efx_raw_writel(u32 value, volatile void __iomem *addr)
	{
		writel(le32_to_cpu(value), addr);
	}
	static inline void
	efx_raw_writeq(u64 value, volatile void __iomem *addr)
	{
		writeq(le64_to_cpu(value), addr);
	}
	static inline u8
	efx_raw_readb(const volatile void __iomem *addr)
	{
		return readb(addr);
	}
	static inline u16
	efx_raw_readw(const volatile void __iomem *addr)
	{
		return cpu_to_le16(readw(addr));
	}
	static inline u32
	efx_raw_readl(const volatile void __iomem *addr)
	{
		return cpu_to_le32(readl(addr));
	}
	static inline u64
	efx_raw_readq(const volatile void __iomem *addr)
	{
		return cpu_to_le64(readq(addr));
	}

	#undef __raw_writeb
	#undef __raw_writew
	#undef __raw_writel
	#undef __raw_writeq
	#undef __raw_readb
	#undef __raw_readw
	#undef __raw_readl
	#undef __raw_readq
	#define __raw_writeb efx_raw_writeb
	#define __raw_writew efx_raw_writew
	#define __raw_writel efx_raw_writel
	#define __raw_writeq efx_raw_writeq
	#define __raw_readb efx_raw_readb
	#define __raw_readw efx_raw_readw
	#define __raw_readl efx_raw_readl
	#define __raw_readq efx_raw_readq
#endif

#ifdef EFX_NEED_SCHEDULE_TIMEOUT_INTERRUPTIBLE
	static inline signed long
	schedule_timeout_interruptible(signed long timeout)
	{
		set_current_state(TASK_INTERRUPTIBLE);
		return schedule_timeout(timeout);
	}
#endif

#ifdef EFX_NEED_SCHEDULE_TIMEOUT_UNINTERRUPTIBLE
	static inline signed long
	schedule_timeout_uninterruptible(signed long timeout)
	{
		set_current_state(TASK_UNINTERRUPTIBLE);
		return schedule_timeout(timeout);
	}
#endif

#ifdef EFX_NEED_MMIOWB
	#if defined(__i386__) || defined(__x86_64__)
		#define mmiowb()
	#elif defined(__ia64__)
		#ifndef ia64_mfa
			#define ia64_mfa() asm volatile ("mf.a" ::: "memory")
		#endif
		#define mmiowb ia64_mfa
	#else
		#error "Need definition for mmiowb()"
	#endif
#endif

#ifdef EFX_NEED_KZALLOC
	static inline void *kzalloc(size_t size, int flags)
	{
		void *buf = kmalloc(size, flags);
		if (buf)
			memset(buf, 0,size);
		return buf;
	}
#endif

#ifdef EFX_NEED_SETUP_TIMER
	static inline void setup_timer(struct timer_list * timer,
				       void (*function)(unsigned long),
				       unsigned long data)
	{
		timer->function = function;
		timer->data = data;
		init_timer(timer);
	}
#endif

#ifdef EFX_NEED_MUTEX
	#define EFX_DEFINE_MUTEX(x) DECLARE_MUTEX(x)
	#undef DEFINE_MUTEX
	#define DEFINE_MUTEX EFX_DEFINE_MUTEX

	#define efx_mutex semaphore
	#undef mutex
	#define mutex efx_mutex

	#define efx_mutex_init(x) init_MUTEX(x)
	#undef mutex_init
	#define mutex_init efx_mutex_init

	#define efx_mutex_destroy(x) do { } while(0)
	#undef mutex_destroy
	#define mutex_destroy efx_mutex_destroy

	#define efx_mutex_lock(x) down(x)
	#undef mutex_lock
	#define mutex_lock efx_mutex_lock

	#define efx_mutex_lock_interruptible(x) down_interruptible(x)
	#undef mutex_lock_interruptible
	#define mutex_lock_interruptible efx_mutex_lock_interruptible

	#define efx_mutex_unlock(x) up(x)
	#undef mutex_unlock
	#define mutex_unlock efx_mutex_unlock

	#define efx_mutex_trylock(x) (!down_trylock(x))
	#undef mutex_trylock
	#define mutex_trylock efx_mutex_trylock

	static inline int efx_mutex_is_locked(struct efx_mutex *m)
	{
		/* NB. This is quite inefficient, but it's the best we
		 * can do with the semaphore API. */
		if ( down_trylock(m) )
			return 1;
		/* Undo the effect of down_trylock. */
		up(m);
		return 0;
	}
	#undef mutex_is_locked
	#define mutex_is_locked efx_mutex_is_locked
#endif

#ifndef NETIF_F_GSO
	#define efx_gso_size tso_size
	#undef gso_size
	#define gso_size efx_gso_size
	#define efx_gso_segs tso_segs
	#undef gso_segs
	#define gso_segs efx_gso_segs
#endif

#ifdef EFX_NEED_IRQF_FLAGS
	#ifdef SA_PROBEIRQ
		#define IRQF_PROBE_SHARED  SA_PROBEIRQ
	#else
		#define IRQF_PROBE_SHARED  0
	#endif
	#define IRQF_SHARED	   SA_SHIRQ
#endif

#ifdef EFX_NEED_NETDEV_ALLOC_SKB
	#ifndef NET_SKB_PAD
		#define NET_SKB_PAD 16
	#endif

	static inline
	struct sk_buff *netdev_alloc_skb(struct net_device *dev,
					 unsigned int length)
	{
		struct sk_buff *skb = alloc_skb(length + NET_SKB_PAD,
						GFP_ATOMIC | __GFP_COLD);
		if (likely(skb)) {
			skb_reserve(skb, NET_SKB_PAD);
			skb->dev = dev;
		}
		return skb;
	}
#endif

#ifdef EFX_NEED_NETIF_TX_LOCK
	static inline void netif_tx_lock(struct net_device *dev)
	{
		spin_lock(&dev->xmit_lock);
		dev->xmit_lock_owner = smp_processor_id();
	}
	static inline void netif_tx_lock_bh(struct net_device *dev)
	{
		spin_lock_bh(&dev->xmit_lock);
		dev->xmit_lock_owner = smp_processor_id();
	}
	static inline void netif_tx_unlock_bh(struct net_device *dev)
	{
		dev->xmit_lock_owner = -1;
		spin_unlock_bh(&dev->xmit_lock);
	}
	static inline void netif_tx_unlock(struct net_device *dev)
	{
		dev->xmit_lock_owner = -1;
		spin_unlock(&dev->xmit_lock);
	}
#endif

#ifdef EFX_NEED_CSUM_UNFOLDED
	typedef u32 __wsum;
	#define csum_unfold(x) ((__force __wsum) x)
#endif

#ifdef EFX_NEED_HEX_DUMP
	enum {
		DUMP_PREFIX_NONE,
		DUMP_PREFIX_ADDRESS,
		DUMP_PREFIX_OFFSET
	};
#endif

#ifdef EFX_NEED_PRINT_MAC
	#define DECLARE_MAC_BUF(var) char var[18] __attribute__((unused))
#endif

#ifdef EFX_NEED_GFP_T
	typedef unsigned int gfp_t;
#endif

#ifdef EFX_NEED_SAFE_LISTS
	#define list_for_each_entry_safe_reverse(pos, n, head, member)	     \
		for (pos = list_entry((head)->prev, typeof(*pos), member),   \
		     n = list_entry(pos->member.prev, typeof(*pos), member); \
		     &pos->member != (head);				     \
		     pos = n,						     \
		     n = list_entry(n->member.prev, typeof(*n), member))
#endif

#ifdef EFX_NEED_DEV_NOTICE
	#define dev_notice dev_warn
#endif

#ifdef EFX_NEED_IF_MII
	#include <linux/mii.h>
	static inline struct mii_ioctl_data *efx_if_mii ( struct ifreq *rq ) {
		return ( struct mii_ioctl_data * ) &rq->ifr_ifru;
	}
	#undef if_mii
	#define if_mii efx_if_mii
#endif

#ifdef EFX_NEED_MTD_ERASE_CALLBACK
	#include <linux/mtd/mtd.h>
	static inline void efx_mtd_erase_callback(struct erase_info *instr) {
		if ( instr->callback )
			instr->callback ( instr );
	}
	#undef mtd_erase_callback
	#define mtd_erase_callback efx_mtd_erase_callback
#endif

#ifdef EFX_NEED_DUMMY_PCI_DISABLE_MSI
	#include <linux/pci.h>
	static inline void dummy_pci_disable_msi ( struct pci_dev *dev ) {
		/* Do nothing */
	}
	#undef pci_disable_msi
	#define pci_disable_msi dummy_pci_disable_msi
#endif

#ifdef EFX_NEED_DUMMY_MSIX
	struct msix_entry {
		u16 	vector;	/* kernel uses to write allocated vector */
		u16	entry;	/* driver uses to specify entry, OS writes */
	};
	static inline int pci_enable_msix(struct pci_dev* dev,
					  struct msix_entry *entries, int nvec)
		{return -1;}
	static inline void pci_disable_msix(struct pci_dev *dev) { /* Do nothing */}
#endif

#ifdef EFX_NEED_BYTEORDER_TYPES
	typedef __u16 __be16;
	typedef __u32 __be32;
	typedef __u64 __be64;
	typedef __u16 __le16;
	typedef __u32 __le32;
	typedef __u64 __le64;
#endif

/**************************************************************************
 *
 * Missing functions provided by kernel_compat.c
 *
 **************************************************************************
 *
 */
#ifdef EFX_NEED_RANDOM_ETHER_ADDR
	extern void efx_random_ether_addr(uint8_t *addr);
	#ifndef EFX_IN_KCOMPAT_C
		#undef random_ether_addr
		#define random_ether_addr efx_random_ether_addr
	#endif
#endif

#ifdef EFX_NEED_MII_ETHTOOL_FIX
	extern int efx_mii_ethtool_gset(struct mii_if_info *mii,
					struct ethtool_cmd *ecmd);
	extern int efx_mii_ethtool_sset(struct mii_if_info *mii,
					struct ethtool_cmd *ecmd);
	#ifndef EFX_IN_KCOMPAT_C
		#undef mii_ethtool_gset
		#define mii_ethtool_gset efx_mii_ethtool_gset
		#undef mii_ethtool_sset
		#define mii_ethtool_sset efx_mii_ethtool_sset
	#endif
#endif

#ifdef EFX_NEED_UNREGISTER_NETDEVICE_NOTIFIER_FIX
	extern int efx_unregister_netdevice_notifier(struct notifier_block *nb);
	#ifndef EFX_IN_KCOMPAT_C
		#undef unregister_netdevice_notifier
		#define unregister_netdevice_notifier \
				efx_unregister_netdevice_notifier
	#endif
#endif

#ifdef EFX_NEED_IOMMU_LOCK
	extern dma_addr_t efx_pci_map_single(struct pci_dev *pci, void *ptr,
					     size_t size, int direction);
	extern void efx_pci_unmap_single(struct pci_dev *pci,
					 dma_addr_t dma_addr, size_t size,
					 int direction);
	extern void * efx_pci_alloc_consistent(struct pci_dev *pci,
					       size_t size,
					       dma_addr_t *dma_addr);
	extern void efx_pci_free_consistent(struct pci_dev *pci,
					    size_t size, void *ptr,
					    dma_addr_t dma_addr);
	#ifndef EFX_IN_KCOMPAT_C
		#undef pci_map_single
		#undef pci_unmap_single
		#undef pci_alloc_consistent
		#undef pci_free_consistent
		#define pci_map_single efx_pci_map_single
		#define pci_unmap_single efx_pci_unmap_single
		#define pci_alloc_consistent efx_pci_alloc_consistent
		#define pci_free_consistent efx_pci_free_consistent
	#endif
#endif

#ifdef EFX_NEED_PRINT_MAC
	extern char *print_mac(char *buf, const u8 *addr);
#endif

#ifdef EFX_NEED_COMPOUND_PAGE_FIX
	extern void efx_compound_page_destructor(struct page *page);
#endif

#ifdef EFX_NEED_HEX_DUMP
	extern void
	print_hex_dump(const char *level, const char *prefix_str,
		       int prefix_type, int rowsize, int groupsize,
		       const void *buf, size_t len, int ascii);
#endif

#ifdef EFX_NEED_MSECS_TO_JIFFIES
	extern unsigned long msecs_to_jiffies(const unsigned int m);
#endif

#ifdef EFX_NEED_MSLEEP
	extern void msleep(unsigned int msecs);
#endif

/**************************************************************************
 *
 * Wrappers to fix bugs and parameter changes
 *
 **************************************************************************
 *
 */

#ifdef EFX_NEED_PCI_SAVE_RESTORE_WRAPPERS
	#define pci_save_state(_dev)					\
		pci_save_state(_dev, (_dev)->saved_config_space)

	#define pci_restore_state(_dev)					\
		pci_restore_state(_dev, (_dev)->saved_config_space)
#endif

#ifdef EFX_NEED_WORK_API_WRAPPERS
	/**
	 * queue_delayed_work in pre 2.6.20 can't rearm from inside
	 * the work member. So instead do a rather hacky sleep
	 */
	#define delayed_work work_struct
	#define INIT_DELAYED_WORK INIT_WORK

	static int inline efx_queue_delayed_work(struct workqueue_struct *wq,
						 struct work_struct *work,
						 unsigned long delay)
	{
		if (unlikely(delay > 0))
			schedule_timeout_uninterruptible(delay);
		return queue_work(wq, work);
	}
	#define queue_delayed_work efx_queue_delayed_work

	/**
	 * The old and new work-function prototypes just differ
	 * in the type of the pointer returned, so it's safe
	 * to cast between the prototypes.
	 */
	typedef void (*efx_old_work_func_t)(void *p);

	#undef INIT_WORK
	#define INIT_WORK(_work, _func)					\
		do {							\
			INIT_LIST_HEAD(&(_work)->entry);		\
			(_work)->pending = 0;				\
			PREPARE_WORK((_work),				\
				     (efx_old_work_func_t) (_func),	\
				     (_work));				\
		} while (0)
#endif

#ifdef EFX_HAVE_OLD_NAPI
	#define napi_str napi_dev[0]

	static inline void netif_napi_add(struct net_device *dev,
					  struct net_device *dummy,
					  int (*poll) (struct net_device *,
						       int *),
					  int weight)
	{
		dev->weight = weight;
		dev->poll = poll;
	}

	#define napi_enable netif_poll_enable
	#define napi_disable netif_poll_disable

	#define netif_rx_complete(dev, dummy) netif_rx_complete(dev)
#endif

#ifdef EFX_NEED_COMPOUND_PAGE_FIX
	static inline
	struct page *efx_alloc_pages(gfp_t flags, unsigned int order)
	{
		struct page *p = alloc_pages(flags, order);
		if ((flags & __GFP_COMP) && (p != NULL) && (order > 0))
			p[1].mapping = (void *)efx_compound_page_destructor;
		return p;
	}
	#undef alloc_pages
	#define alloc_pages efx_alloc_pages

	static inline
	void efx_free_pages(struct page *p, unsigned int order)
	{
		if ((order > 0) && (page_count(p) == 1))
			p[1].mapping = NULL;
		__free_pages(p, order);
	}
	#define __free_pages efx_free_pages
#endif

#ifdef EFX_NEED_HEX_DUMP_CONST_FIX
	#define print_hex_dump(v,s,t,r,g,b,l,a) \
		print_hex_dump((v),(s),(t),(r),(g),(void*)(b),(l),(a))
#endif

#endif /* EFX_KERNEL_COMPAT_H */
