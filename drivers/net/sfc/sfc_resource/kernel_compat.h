/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file provides compatibility layer for various Linux kernel versions
 * (starting from 2.6.9 RHEL kernel).
 *
 * Copyright 2005-2007: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed and maintained by Solarflare Communications:
 *                      <linux-xen-drivers@solarflare.com>
 *                      <onload-dev@solarflare.com>
 *
 * Certain parts of the driver were implemented by
 *          Alexandra Kossovsky <Alexandra.Kossovsky@oktetlabs.ru>
 *          OKTET Labs Ltd, Russia,
 *          http://oktetlabs.ru, <info@oktetlabs.ru>
 *          by request of Solarflare Communications
 *
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

#ifndef DRIVER_LINUX_RESOURCE_KERNEL_COMPAT_H
#define DRIVER_LINUX_RESOURCE_KERNEL_COMPAT_H

#include <linux/version.h>

/********* wait_for_completion_timeout() ********************/
#include <linux/sched.h>

/* RHEL_RELEASE_CODE from linux/version.h is only defined for 2.6.9-55EL
 * UTS_RELEASE is unfortunately unusable
 * Really only need this fix for <2.6.9-34EL
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)) && \
	!defined(RHEL_RELEASE_CODE)

static inline unsigned long fastcall __sched
efrm_wait_for_completion_timeout(struct completion *x, unsigned long timeout)
{
	might_sleep();

	spin_lock_irq(&x->wait.lock);
	if (!x->done) {
		DECLARE_WAITQUEUE(wait, current);

		wait.flags |= WQ_FLAG_EXCLUSIVE;
		__add_wait_queue_tail(&x->wait, &wait);
		do {
			__set_current_state(TASK_UNINTERRUPTIBLE);
			spin_unlock_irq(&x->wait.lock);
			timeout = schedule_timeout(timeout);
			spin_lock_irq(&x->wait.lock);
			if (!timeout) {
				__remove_wait_queue(&x->wait, &wait);
				goto out;
			}
		} while (!x->done);
		__remove_wait_queue(&x->wait, &wait);
	}
	x->done--;
out:
	spin_unlock_irq(&x->wait.lock);
	return timeout;
}

#  ifdef wait_for_completion_timeout
#    undef wait_for_completion_timeout
#  endif
#  define wait_for_completion_timeout efrm_wait_for_completion_timeout

#endif

/********* pci_map_*() ********************/

#include <linux/pci.h>

/* Bug 4560: Some kernels leak IOMMU entries under heavy load.  Use a
 * spinlock to serialise access where possible to alleviate the
 * problem.
 *
 * NB. This is duplicated in the net driver.  Please keep in sync. */
#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)) && \
     (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17)) && \
      defined(__x86_64__) && defined(CONFIG_SMP))

#define EFRM_HAVE_IOMMU_LOCK 1

#if ((LINUX_VERSION_CODE == KERNEL_VERSION(2,6,5)) &&	\
      defined(CONFIG_SUSE_KERNEL))
#define EFRM_NEED_ALTERNATE_MAX_PFN 1
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#if defined(CONFIG_GART_IOMMU)
#define EFRM_NO_IOMMU no_iommu
#else
#define EFRM_NO_IOMMU 1
#endif
#else
#define EFRM_NO_IOMMU 0
#endif

/* Set to 0 if we should never use the lock.  Set to 1 if we should
 * automatically determine if we should use the lock.  Set to 2 if we
 * should always use the lock. */
extern unsigned int efx_use_iommu_lock;
/* Defined in the net driver. */
extern spinlock_t efx_iommu_lock;
/* Non-zero if there is a card which needs the lock. */
extern int efrm_need_iommu_lock;

/* The IRQ state is needed if the lock is being used.  The flag is
 * cached to ensure that every lock is followed by an unlock, even
 * if the global flag changes in the middle of the operation. */

#define EFRM_IOMMU_DECL				\
	unsigned long efx_iommu_irq_state = 0;	\
	int efx_iommu_using_lock;
#define EFRM_IOMMU_LOCK()						\
	do {								\
		efx_iommu_using_lock = (efx_use_iommu_lock &&		\
					(efrm_need_iommu_lock ||	\
					 efx_use_iommu_lock >= 2));	\
		if (efx_iommu_using_lock)				\
		spin_lock_irqsave(&efx_iommu_lock, efx_iommu_irq_state);\
	} while (0)
#define EFRM_IOMMU_UNLOCK()						\
	do {								\
		if (efx_iommu_using_lock)				\
		spin_unlock_irqrestore(&efx_iommu_lock,			\
				       efx_iommu_irq_state);		\
	} while (0)

#else /* defined(__x86_64__) && defined(CONFIG_SMP) */

#define EFRM_HAVE_IOMMU_LOCK 0
#define EFRM_IOMMU_DECL
#define EFRM_IOMMU_LOCK()    do {} while (0)
#define EFRM_IOMMU_UNLOCK()  do {} while (0)

#endif

static inline dma_addr_t efrm_pci_map_single(struct pci_dev *hwdev, void *ptr,
					     size_t size, int direction)
{
	dma_addr_t dma_addr;
	EFRM_IOMMU_DECL;

	EFRM_IOMMU_LOCK();
	dma_addr = pci_map_single(hwdev, ptr, size, direction);
	EFRM_IOMMU_UNLOCK();

	return dma_addr;
}

static inline void efrm_pci_unmap_single(struct pci_dev *hwdev,
					 dma_addr_t dma_addr, size_t size,
					 int direction)
{
	EFRM_IOMMU_DECL;

	EFRM_IOMMU_LOCK();
	pci_unmap_single(hwdev, dma_addr, size, direction);
	EFRM_IOMMU_UNLOCK();
}

static inline dma_addr_t efrm_pci_map_page(struct pci_dev *hwdev,
					   struct page *page,
					   unsigned long offset, size_t size,
					   int direction)
{
	dma_addr_t dma_addr;
	EFRM_IOMMU_DECL;

	EFRM_IOMMU_LOCK();
	dma_addr = pci_map_page(hwdev, page, offset, size, direction);
	EFRM_IOMMU_UNLOCK();

	return dma_addr;
}

static inline void efrm_pci_unmap_page(struct pci_dev *hwdev,
				       dma_addr_t dma_addr, size_t size,
				       int direction)
{
	EFRM_IOMMU_DECL;

	EFRM_IOMMU_LOCK();
	pci_unmap_page(hwdev, dma_addr, size, direction);
	EFRM_IOMMU_UNLOCK();
}

#ifndef IN_KERNEL_COMPAT_C
#  ifndef __GFP_COMP
#    define __GFP_COMP 0
#  endif
#  ifndef __GFP_ZERO
#    define __GFP_ZERO 0
#  endif
#endif

extern void *efrm_dma_alloc_coherent(struct device *dev, size_t size,
				     dma_addr_t *dma_addr, int flag);

extern void efrm_dma_free_coherent(struct device *dev, size_t size,
				   void *ptr, dma_addr_t dma_addr);

static inline void *efrm_pci_alloc_consistent(struct pci_dev *hwdev,
					      size_t size,
					      dma_addr_t *dma_addr)
{
	return efrm_dma_alloc_coherent(&hwdev->dev, size, dma_addr,
				       GFP_ATOMIC);
}

static inline void efrm_pci_free_consistent(struct pci_dev *hwdev, size_t size,
					    void *ptr, dma_addr_t dma_addr)
{
	efrm_dma_free_coherent(&hwdev->dev, size, ptr, dma_addr);
}

#endif /* DRIVER_LINUX_RESOURCE_KERNEL_COMPAT_H */
