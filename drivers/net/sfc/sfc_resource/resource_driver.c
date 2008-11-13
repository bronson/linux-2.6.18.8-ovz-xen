/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains main driver entry points.
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

#include "linux_resource_internal.h"
#include "kernel_compat.h"
#include <ci/efrm/nic_table.h>
#include <ci/driver/resource/efx_vi.h>
#include <ci/efhw/eventq.h>
#include <ci/efhw/nic.h>
#include <ci/efrm/buffer_table.h>
#include <ci/efrm/vi_resource_private.h>
#include <ci/efrm/driver_private.h>

#if EFRM_HAVE_IOMMU_LOCK
#ifdef EFRM_NEED_ALTERNATE_MAX_PFN
extern unsigned long blk_max_pfn;
#define max_pfn blk_max_pfn
#else
#include <linux/bootmem.h>
#endif
#endif

MODULE_AUTHOR("Solarflare Communications");
MODULE_LICENSE("GPL");

static struct efhw_ev_handler ev_handler = {
	.wakeup_fn = efrm_handle_wakeup_event,
	.timeout_fn = efrm_handle_timeout_event,
	.dmaq_flushed_fn = efrm_handle_dmaq_flushed,
};

#if EFRM_HAVE_IOMMU_LOCK
int efrm_need_iommu_lock;
EXPORT_SYMBOL(efrm_need_iommu_lock);
#endif

const int max_hardware_init_repeats = 10;

/*--------------------------------------------------------------------
 *
 * Module load time variables
 *
 *--------------------------------------------------------------------*/
/* See docs/notes/pci_alloc_consistent */
int use_pci_alloc = 1;		/* Use pci_alloc_consistent to alloc iopages */
static int do_irq = 1;		/* enable interrupts */

#if defined(CONFIG_X86_XEN)
static int irq_moderation = 60;	/* interrupt moderation (60 usec) */
#else
static int irq_moderation = 20;	/* interrupt moderation (20 usec) */
#endif
static int nic_options = NIC_OPT_DEFAULT;
int efx_vi_eventq_size = EFX_VI_EVENTQ_SIZE_DEFAULT;

module_param(do_irq, int, S_IRUGO);
MODULE_PARM_DESC(do_irq, "Enable interrupts.  "
		 "Do not turn it off unless you know what are you doing.");
module_param(irq_moderation, int, S_IRUGO);
MODULE_PARM_DESC(irq_moderation, "IRQ moderation in usec");
module_param(nic_options, int, S_IRUGO);
MODULE_PARM_DESC(nic_options, "Nic options -- see efhw_types.h");
module_param(use_pci_alloc, int, S_IRUGO);
MODULE_PARM_DESC(use_pci_alloc, "Use pci_alloc_consistent to alloc iopages "
		 "(autodetected by kernel version)");
module_param(efx_vi_eventq_size, int, S_IRUGO);
MODULE_PARM_DESC(efx_vi_eventq_size,
		 "Size of event queue allocated by efx_vi library");

/*--------------------------------------------------------------------
 *
 * Linux specific NIC initialisation
 *
 *--------------------------------------------------------------------*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
# define IRQ_PT_REGS_ARG   , struct pt_regs *regs __attribute__ ((unused))
#else
# define IRQ_PT_REGS_ARG
#endif

#ifndef IRQF_SHARED
# define IRQF_SHARED SA_SHIRQ
#endif

static inline irqreturn_t
linux_efrm_interrupt(int irr, void *dev_id IRQ_PT_REGS_ARG)
{
	return efhw_nic_interrupt((struct efhw_nic *)dev_id);
}

int linux_efrm_irq_ctor(struct linux_efhw_nic *lnic)
{
	struct efhw_nic *nic = &lnic->nic;

	nic->flags &= ~NIC_FLAG_MSI;
	if (nic->flags & NIC_FLAG_TRY_MSI) {
		int rc = pci_enable_msi(lnic->pci_dev);
		if (rc < 0) {
			EFRM_WARN("%s: Could not enable MSI (%d)",
				  __FUNCTION__, rc);
			EFRM_WARN("%s: Continuing with legacy interrupt mode",
				  __FUNCTION__);
		} else {
			EFRM_NOTICE("%s: MSI enabled", __FUNCTION__);
			nic->flags |= NIC_FLAG_MSI;
		}
	}

	if (request_irq(lnic->pci_dev->irq, linux_efrm_interrupt,
			IRQF_SHARED, "sfc_resource", nic)) {
		EFRM_ERR("Request for interrupt #%d failed",
			 lnic->pci_dev->irq);
		nic->flags &= ~NIC_FLAG_OS_IRQ_EN;
		return -EBUSY;
	}
	nic->flags |= NIC_FLAG_OS_IRQ_EN;

	return 0;
}

void linux_efrm_irq_dtor(struct linux_efhw_nic *lnic)
{
	EFRM_TRACE("linux_efrm_irq_dtor: start");

	if (lnic->nic.flags & NIC_FLAG_OS_IRQ_EN) {
		free_irq(lnic->pci_dev->irq, &lnic->nic);
		lnic->nic.flags &= ~NIC_FLAG_OS_IRQ_EN;
	}

	if (lnic->nic.flags & NIC_FLAG_MSI) {
		pci_disable_msi(lnic->pci_dev);
		lnic->nic.flags &= ~NIC_FLAG_MSI;
	}

	EFRM_TRACE("linux_efrm_irq_dtor: done");
}

/* Allocate buffer table entries for a particular NIC.
 */
static int efrm_nic_buffer_table_alloc(struct efhw_nic *nic)
{
	int capacity;
	int page_order;
	int i;
	int rc;

	/* Choose queue size. */
	for (capacity = 8192; capacity <= nic->evq_sizes; capacity <<= 1) {
		if (capacity > nic->evq_sizes) {
			EFRM_ERR
			    ("%s: Unable to choose EVQ size (supported=%x)",
			     __FUNCTION__, nic->evq_sizes);
			return -E2BIG;
		} else if (capacity & nic->evq_sizes)
			break;
	}
	for (i = 0; i < EFHW_KEVENTQ_MAX; ++i) {
		nic->evq[i].hw.capacity = capacity;
		nic->evq[i].hw.buf_tbl_alloc.base = (unsigned)-1;
	}

	/* allocate buffer table entries to map onto the iobuffer */
	page_order = get_order(capacity * sizeof(efhw_event_t));
	if (!(nic->flags & NIC_FLAG_NO_INTERRUPT)) {
		rc = efrm_buffer_table_alloc(page_order,
					     &nic->evq[0].hw.buf_tbl_alloc);
		if (rc < 0) {
			EFRM_WARN
			    ("%s: failed (%d) to alloc %d buffer table entries",
			     __FUNCTION__, rc, page_order);
			return rc;
		}
	}
	rc = efrm_buffer_table_alloc(page_order,
				     &nic->evq[FALCON_EVQ_NONIRQ].hw.
				     buf_tbl_alloc);
	if (rc < 0) {
		EFRM_WARN
		    ("%s: failed (%d) to alloc %d buffer table entries",
		     __FUNCTION__, rc, page_order);
		return rc;
	}

	return 0;
}

/* Free buffer table entries allocated for a particular NIC.
 */
static void efrm_nic_buffer_table_free(struct efhw_nic *nic)
{
	int i;
	for (i = 0; i <= FALCON_EVQ_NONIRQ; i++)
		if (nic->evq[i].hw.buf_tbl_alloc.base != (unsigned)-1)
			efrm_buffer_table_free(&nic->evq[i].hw.buf_tbl_alloc);

}

static int iomap_bar(struct linux_efhw_nic *lnic, size_t len)
{
	efhw_ioaddr_t ioaddr;

	ioaddr = ioremap_nocache(lnic->ctr_ap_pci_addr, len);
	if (ioaddr == 0)
		return -ENOMEM;

	lnic->nic.bar_ioaddr = ioaddr;
	return 0;
}

static int linux_efhw_nic_map_ctr_ap(struct linux_efhw_nic *lnic)
{
	struct efhw_nic *nic = &lnic->nic;
	int rc;

	rc = iomap_bar(lnic, nic->ctr_ap_bytes);

#if defined(__CI_HARDWARE_CONFIG_FALCON__)
	/* Bug 5195: workaround for now. */
	if (rc != 0 && nic->ctr_ap_bytes > 16 * 1024 * 1024) {
		/* Try half the size for now. */
		nic->ctr_ap_bytes /= 2;
		EFRM_WARN("Bug 5195 WORKAROUND: retrying iomap of %d bytes",
			  nic->ctr_ap_bytes);
		rc = iomap_bar(lnic, nic->ctr_ap_bytes);
	}
#endif

	if (rc < 0) {
		EFRM_ERR("Failed (%d) to map bar (%d bytes)",
			 rc, nic->ctr_ap_bytes);
		return rc;
	}

	return rc;
}

int
linux_efrm_nic_ctor(struct linux_efhw_nic *lnic, struct pci_dev *dev,
		    spinlock_t *reg_lock,
		    unsigned nic_flags, unsigned nic_options)
{
	struct efhw_device_type dev_type;
	struct efhw_nic *nic = &lnic->nic;
	u8 class_revision;
	int rc;

	rc = pci_read_config_byte(dev, PCI_CLASS_REVISION, &class_revision);
	if (rc != 0) {
		EFRM_ERR("%s: pci_read_config_byte failed (%d)",
			 __FUNCTION__, rc);
		return rc;
	}

	if (!efhw_device_type_init(&dev_type, dev->vendor, dev->device,
				   class_revision)) {
		EFRM_ERR("%s: efhw_device_type_init failed %04x:%04x(%d)",
			 __FUNCTION__, (unsigned) dev->vendor,
			 (unsigned) dev->device, (int) class_revision);
		return -ENODEV;
	}

	EFRM_NOTICE("attaching device type %04x:%04x %d:%c%d",
		    (unsigned) dev->vendor, (unsigned) dev->device,
		    dev_type.arch, dev_type.variant, dev_type.revision);

	/* Initialise the adapter-structure. */
	efhw_nic_init(nic, nic_flags, nic_options, dev_type);
	lnic->pci_dev = dev;

	rc = pci_enable_device(dev);
	if (rc < 0) {
		EFRM_ERR("%s: pci_enable_device failed (%d)",
			 __FUNCTION__, rc);
		return rc;
	}

	lnic->ctr_ap_pci_addr = pci_resource_start(dev, nic->ctr_ap_bar);

	if (!pci_dma_supported(dev, (dma_addr_t)EFHW_DMA_ADDRMASK)) {
		EFRM_ERR("%s: pci_dma_supported(%lx) failed", __FUNCTION__,
			 (unsigned long)EFHW_DMA_ADDRMASK);
		return -ENODEV;
	}

	if (pci_set_dma_mask(dev, (dma_addr_t)EFHW_DMA_ADDRMASK)) {
		EFRM_ERR("%s: pci_set_dma_mask(%lx) failed", __FUNCTION__,
			 (unsigned long)EFHW_DMA_ADDRMASK);
		return -ENODEV;
	}

	if (pci_set_consistent_dma_mask(dev, (dma_addr_t)EFHW_DMA_ADDRMASK)) {
		EFRM_ERR("%s: pci_set_consistent_dma_mask(%lx) failed",
			 __FUNCTION__, (unsigned long)EFHW_DMA_ADDRMASK);
		return -ENODEV;
	}

	rc = linux_efhw_nic_map_ctr_ap(lnic);
	if (rc < 0)
		return rc;

	/* By default struct efhw_nic contains its own lock for protecting
	 * access to nic registers.  We override it with a pointer to the
	 * lock in the net driver.  This is needed when resource and net
	 * drivers share a single PCI function (falcon B series).
	 */
	nic->reg_lock = reg_lock;
	return 0;
}

void linux_efrm_nic_dtor(struct linux_efhw_nic *lnic)
{
	struct efhw_nic *nic = &lnic->nic;
	efhw_ioaddr_t bar_ioaddr = nic->bar_ioaddr;

	efhw_nic_dtor(nic);

	efrm_nic_buffer_table_free(nic);

	/* Unmap the bar. */
	EFRM_ASSERT(bar_ioaddr);
	iounmap(bar_ioaddr);
	nic->bar_ioaddr = 0;
}

/****************************************************************************
 *
 * efrm_tasklet - used to poll the eventq which may result in further callbacks
 *
 ****************************************************************************/

static void efrm_tasklet(unsigned long pdev)
{
	struct efhw_nic *nic = (struct efhw_nic *)pdev;

	EFRM_ASSERT(!(nic->flags & NIC_FLAG_NO_INTERRUPT));

	efhw_keventq_poll(nic, &nic->evq[0]);
	EFRM_TRACE("tasklet complete");
}

/****************************************************************************
 *
 * char driver specific interrupt callbacks -- run at hard IRQL
 *
 ****************************************************************************/
static void efrm_handle_eventq_irq(struct efhw_nic *nic, int evq)
{
	/* NB. The interrupt must have already been acked (for legacy mode). */

	EFRM_TRACE("%s: starting tasklet", __FUNCTION__);
	EFRM_ASSERT(!(nic->flags & NIC_FLAG_NO_INTERRUPT));

	tasklet_schedule(&linux_efhw_nic(nic)->tasklet);
}

/* A count of how many NICs this driver knows about. */
static int n_nics_probed;

/****************************************************************************
 *
 * efrm_nic_add: add the NIC to the resource driver
 *
 * NOTE: the flow of control through this routine is quite subtle
 * because of the number of operations that can fail. We therefore
 * take the apporaching of keeping the return code (rc) variable
 * accurate, and only do operations while it is non-negative. Tear down
 * is done at the end if rc is negative, depending on what has been set up
 * by that point.
 *
 * So basically just make sure that any code you add checks rc>=0 before
 * doing any work and you'll be fine.
 *
 ****************************************************************************/
int
efrm_nic_add(struct pci_dev *dev, unsigned flags, const uint8_t *mac_addr,
	     struct linux_efhw_nic **lnic_out, spinlock_t *reg_lock,
	     int bt_min, int bt_max,
	     const struct vi_resource_dimensions *res_dim)
{
	struct linux_efhw_nic *lnic = NULL;
	struct efhw_nic *nic = NULL;
	int count = 0, rc = 0, resources_init = 0;
	int constructed = 0;
	int registered_nic = 0;
	int buffers_allocated = 0;
	static unsigned nic_index; /* = 0; */

	EFRM_TRACE("%s: device detected (Slot '%s', IRQ %d)", __FUNCTION__,
		   pci_name(dev) ? pci_name(dev) : "?", dev->irq);

	/* Ensure that we have room for the new adapter-structure. */
	if (efrm_nic_table.nic_count == EFHW_MAX_NR_DEVS) {
		EFRM_WARN("%s: WARNING: too many devices", __FUNCTION__);
		rc = -ENOMEM;
		goto failed;
	}

	if (n_nics_probed == 0) {
		rc = efrm_resources_init(res_dim, bt_min, bt_max);
		if (rc != 0)
			goto failed;
		resources_init = 1;
	}

	/* Allocate memory for the new adapter-structure. */
	lnic = kmalloc(sizeof(*lnic), GFP_KERNEL);
	if (lnic == NULL) {
		EFRM_ERR("%s: ERROR: failed to allocate memory", __FUNCTION__);
		rc = -ENOMEM;
		goto failed;
	}
	memset(lnic, 0, sizeof(*lnic));
	nic = &lnic->nic;

	lnic->ev_handlers = &ev_handler;

	/* OS specific hardware mappings */
	rc = linux_efrm_nic_ctor(lnic, dev, reg_lock, flags, nic_options);
	if (rc < 0) {
		EFRM_ERR("%s: ERROR: initialisation failed", __FUNCTION__);
		goto failed;
	}

	constructed = 1;

	/* Tell the driver about the NIC - this needs to be done before the
	   resources managers get created below. Note we haven't initialised
	   the hardware yet, and I don't like doing this before the perhaps
	   unreliable hardware initialisation. However, there's quite a lot
	   of code to review if we wanted to hardware init before bringing
	   up the resource managers. */
	rc = efrm_driver_register_nic(nic, nic_index++);
	if (rc < 0) {
		EFRM_ERR("%s: cannot register nic %d with nic error code %d",
			 __FUNCTION__, efrm_nic_table.nic_count, rc);
		goto failed;
	}
	registered_nic = 1;

	rc = efrm_nic_buffer_table_alloc(nic);
	if (rc < 0)
		goto failed;
	buffers_allocated = 1;

	/****************************************************/
	/* hardware bringup                                 */
	/****************************************************/
	/* Detecting hardware can be a slightly unreliable process;
	   we want to make sure that we maximise our chances, so we
	   loop a few times until all is good. */
	for (count = 0; count < max_hardware_init_repeats; count++) {
		rc = efhw_nic_init_hardware(nic, &ev_handler, mac_addr);
		if (rc >= 0)
			break;

		/* pain */
		EFRM_ERR
		    ("error - hardware initialisation failed code %d, "
		     "attempt %d of %d", rc, count + 1,
		     max_hardware_init_repeats);
	}
	if (rc < 0)
		goto failed;

	tasklet_init(&lnic->tasklet, efrm_tasklet, (ulong)nic);

	/* set up interrupt handlers (hard-irq) */
	nic->irq_handler = &efrm_handle_eventq_irq;

	/* this device can now take management interrupts */
	if (do_irq && !(nic->flags & NIC_FLAG_NO_INTERRUPT)) {
		rc = linux_efrm_irq_ctor(lnic);
		if (rc < 0) {
			EFRM_ERR("Interrupt initialisation failed (%d)", rc);
			goto failed;
		}
		efhw_nic_set_interrupt_moderation(nic, 0, irq_moderation);
		efhw_nic_interrupt_enable(nic, 0);
	}
	EFRM_TRACE("interrupts are %sregistered", do_irq ? "" : "not ");

#if EFRM_HAVE_IOMMU_LOCK
	/* Bug 4560: We need the lock if there is memory which cannot be
	 * accessed by the card and there is an IOMMU to access it.  In that
	 * case, the kernel will use the IOMMU to access the high memory. */
	if ((dev->dma_mask >> PAGE_SHIFT) < max_pfn && !EFRM_NO_IOMMU)
		efrm_need_iommu_lock = 1;
#endif

	*lnic_out = lnic;
	EFRM_ASSERT(rc == 0);
	++n_nics_probed;
	return 0;

failed:
	if (buffers_allocated)
		efrm_nic_buffer_table_free(nic);
	if (registered_nic)
		efrm_driver_unregister_nic(nic);
	if (constructed)
		linux_efrm_nic_dtor(lnic);
	kfree(lnic); /* safe in any case */
	if (resources_init)
		efrm_resources_fini();
	return rc;
}

/****************************************************************************
 *
 * efrm_nic_del: Remove the nic from the resource driver structures
 *
 ****************************************************************************/
void efrm_nic_del(struct linux_efhw_nic *lnic)
{
	struct efhw_nic *nic = &lnic->nic;

	EFRM_TRACE("%s:", __FUNCTION__);
	EFRM_ASSERT(nic);

	efrm_driver_unregister_nic(nic);

	/*
	 * Synchronise here with any running ISR.
	 * Remove the OS handler. There should be no IRQs being generated
	 * by our NIC at this point.
	 */
	if (efhw_nic_have_functional_units(nic)) {
		efhw_nic_close_interrupts(nic);
		linux_efrm_irq_dtor(lnic);
		tasklet_kill(&lnic->tasklet);
	}

	/* Close down hardware and free resources. */
	linux_efrm_nic_dtor(lnic);
	kfree(lnic);

	if (--n_nics_probed == 0)
		efrm_resources_fini();

	EFRM_TRACE("NIC teardown: Done");
}

/****************************************************************************
 *
 * init_module: register as a PCI driver.
 *
 ****************************************************************************/
static int init_sfc_resource(void)
{
	int rc = 0;

	EFRM_TRACE("%s: RESOURCE driver starting", __FUNCTION__);

	rc = efrm_driver_ctor();
	if (rc < 0) {
		EFRM_ERR("%s: efrm_driver_ctor: error %d", __FUNCTION__, rc);
		goto fail_driver_ctor;
	}

	/* Register the driver so that our 'probe' function is called for
	 * each EtherFabric device in the system.
	 */
	rc = efrm_driverlink_register();
	if (rc == -ENODEV)
		EFRM_ERR("%s: no devices found", __FUNCTION__);
	if (rc < 0)
		goto failed_driverlink;

	if (efrm_install_proc_entries() != 0) {
		/* Do not fail, but print a warning */
		EFRM_WARN("%s: WARNING: failed to install /proc entries",
			  __FUNCTION__);
	}

	return 0;

failed_driverlink:
	/* No need to release resource managers here since they register
	 * destructors with the driver. */
	efrm_driver_dtor();
fail_driver_ctor:
	EFRM_ASSERT(rc != 0);
	return rc;
}

/****************************************************************************
 *
 * cleanup_module: module-removal entry-point
 *
 ****************************************************************************/
static void cleanup_sfc_resource(void)
{
	efrm_uninstall_proc_entries();

	efrm_driverlink_unregister();

	/* Clean up char-driver specific initialisation.
	   - driver dtor can use both work queue and buffer table entries */
	efrm_driver_dtor();

	EFRM_TRACE("unloaded");
}

module_init(init_sfc_resource);
module_exit(cleanup_sfc_resource);
