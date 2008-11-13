/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains type definitions for VI resource.  These types
 * may be used outside of the SFC resource driver, but such use is not
 * recommended.
 *
 * Copyright 2005-2007: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed and maintained by Solarflare Communications:
 *                      <linux-xen-drivers@solarflare.com>
 *                      <onload-dev@solarflare.com>
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

#ifndef __CI_DRIVER_EFAB_VI_RESOURCE_MANAGER_H__
#define __CI_DRIVER_EFAB_VI_RESOURCE_MANAGER_H__

#include <ci/efhw/common.h>
#include <ci/efrm/vi_resource.h>
#include <ci/efrm/nic_set.h>

#define EFRM_VI_RM_DMA_QUEUE_COUNT 2
#define EFRM_VI_RM_DMA_QUEUE_TX    0
#define EFRM_VI_RM_DMA_QUEUE_RX    1

/** Numbers of bits which can be set in the evq_state member of
 * vi_resource_evq_info. */
enum {
  /** This bit is set if a wakeup has been requested on the NIC. */
	VI_RESOURCE_EVQ_STATE_WAKEUP_PENDING,
  /** This bit is set if the wakeup is valid for the sleeping
   * process. */
	VI_RESOURCE_EVQ_STATE_CALLBACK_REGISTERED,
  /** This bit is set if a wakeup or timeout event is currently being
   * processed. */
	VI_RESOURCE_EVQ_STATE_BUSY,
};
#define VI_RESOURCE_EVQ_STATE(X) \
	(((int32_t)1) << (VI_RESOURCE_EVQ_STATE_##X))

/** Information about an event queue. */
struct vi_resource_evq_info {
  /** Flag bits indicating the state of wakeups. */
	unsigned long evq_state;
  /** A pointer to the resource instance for this queue.  This member
   * is only valid if evq_state is non-zero or the resource is known
   * to have a non-zero reference count. */
	struct vi_resource *evq_virs;
};

#ifdef __ci_ul_driver__
#define EFRM_VI_USE_WORKQUEUE 0
#else
#define EFRM_VI_USE_WORKQUEUE 1
#endif

/*! Global information for the VI resource manager. */
struct vi_resource_manager {
	struct efrm_resource_manager rm;

	struct kfifo *instances_with_timer;
	int with_timer_base;
	int with_timer_limit;
	struct kfifo *instances_with_interrupt;
	int with_interrupt_base;
	int with_interrupt_limit;

	bool iscsi_dmaq_instance_is_free;
	struct vi_resource_evq_info *evq_infos;

	/* We keep VI resources which need flushing on these lists.  The VI
	 * is put on the outstanding list when the flush request is issued
	 * to the hardware and removed when the flush event arrives.  The
	 * hardware can only handle a limited number of RX flush requests at
	 * once, so VIs are placed in the waiting list until the flush can
	 * be issued.  Flushes can be requested by the client or internally
	 * by the VI resource manager.  In the former case, the reference
	 * count must be non-zero for the duration of the flush and in the
	 * later case, the reference count must be zero. */
	struct list_head rx_flush_waiting_list;
	struct list_head rx_flush_outstanding_list;
	struct list_head tx_flush_outstanding_list;
	int rx_flush_outstanding_count;

	/* once the flush has happened we push the close into the work queue
	 * so its OK on Windows to free the resources (Bug 3469).  Resources
	 * on this list have zero reference count.
	 */
	struct list_head close_pending;
	struct work_struct work_item;
#if EFRM_VI_USE_WORKQUEUE
	struct workqueue_struct *workqueue;
#endif
};

struct vi_resource_nic_info {
	struct eventq_resource_hardware evq_pages;
#if defined(__CI_HARDWARE_CONFIG_FALCON__)
	efhw_iopages_t dmaq_pages[EFRM_VI_RM_DMA_QUEUE_COUNT];
#endif
};

struct vi_resource {
	/* Some macros make the assumption that the struct efrm_resource is
	 * the first member of a struct vi_resource. */
	struct efrm_resource rs;
	atomic_t evq_refs;	/*!< Number of users of the event queue. */

	efrm_nic_set_t nic_set;

	uint32_t bar_mmap_bytes;
	uint32_t mem_mmap_bytes;

	int32_t evq_capacity;
	int32_t dmaq_capacity[EFRM_VI_RM_DMA_QUEUE_COUNT];

	uint8_t dmaq_tag[EFRM_VI_RM_DMA_QUEUE_COUNT];
	uint16_t flags;

	/* we keep PT endpoints that have been destroyed on a list
	 * until we have seen their TX and RX DMAQs flush complete
	 * (see Bug 1217)
	 */
	struct list_head rx_flush_link;
	struct list_head tx_flush_link;
	efrm_nic_set_t rx_flush_nic_set;
	efrm_nic_set_t rx_flush_outstanding_nic_set;
	efrm_nic_set_t tx_flush_nic_set;
	uint64_t flush_time;
	int flush_count;

	void (*flush_callback_fn)(void *);
	void *flush_callback_arg;

	void (*evq_callback_fn) (void *arg, int is_timeout,
				 struct efhw_nic *nic);
	void *evq_callback_arg;

	struct vi_resource *evq_virs;	/*!< EVQ for DMA queues */

#if defined(__CI_HARDWARE_CONFIG_FALCON__)
	 struct efhw_buffer_table_allocation
	    dmaq_buf_tbl_alloc[EFRM_VI_RM_DMA_QUEUE_COUNT];
#endif

	struct vi_resource_nic_info nic_info[EFHW_MAX_NR_DEVS];
};

#undef vi_resource
#define vi_resource(rs1)  container_of((rs1), struct vi_resource, rs)

static inline dma_addr_t
efrm_eventq_dma_addr(struct vi_resource *virs, uint32_t nic_index)
{
	struct eventq_resource_hardware *hw;
	EFRM_ASSERT(efrm_nic_set_read(&virs->nic_set, nic_index));

	hw = &(virs->nic_info[nic_index].evq_pages);

	return efhw_iopages_dma_addr(&(hw->iobuff)) + hw->iobuff_off;
}

#endif /* __CI_DRIVER_EFAB_VI_RESOURCE_MANAGER_H__ */
