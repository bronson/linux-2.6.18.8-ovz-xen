/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005:      Fen Systems Ltd.
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
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 ****************************************************************************
 */

#include <linux/module.h>
#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include "net_driver.h"
#include "efx.h"
#include "driverlink.h"

/* Driverlink semaphore
 * This semaphore must be held for any operation that modifies any of
 * the driverlink lists.
 */
static DEFINE_MUTEX(efx_driverlink_lock);

/* List of all registered drivers */
static LIST_HEAD(efx_driver_list);

/* List of all registered Efx ports */
static LIST_HEAD(efx_port_list);

/* Driver link handle used internally to track devices */
struct efx_dl_handle {
	/* The efx_dl_device consumers see */
	struct efx_dl_device efx_dev;
	/* The efx_nic providers provide */
	struct efx_nic *efx;
	/* Per-device list */
	struct list_head port_node;
	/* Per-driver list */
	struct list_head driver_node;
};

/* Get the handle for an efx_dl_device */
static struct efx_dl_handle *efx_dl_handle(struct efx_dl_device *efx_dev)
{
	return container_of(efx_dev, struct efx_dl_handle, efx_dev);
}

/* Remove an Efx device
 * You must hold the efx_driverlink_lock before calling this
 * function.
 */
static void efx_dl_del_device(struct efx_dl_device *efx_dev)
{
	struct efx_dl_handle *efx_handle = efx_dl_handle(efx_dev);

	EFX_INFO(efx_handle->efx, "%s driverlink client unregistering\n",
		 efx_dev->driver->name);

	/* Call driver's remove() routine */
	if (efx_dev->driver->remove)
		efx_dev->driver->remove(efx_dev);

	/* Remove handle from per-driver and per-NIC lists */
	list_del(&efx_handle->driver_node);
	list_del(&efx_handle->port_node);

	/* Free efx_handle structure */
	kfree(efx_handle);
}

/* Try to add an Efx device
 * Attempt to probe the given device with the driver, creating a
 * new efx_dl_device. If the probe routine fails, because the driver
 * doesn't support this port, then the efx_dl_device is destroyed,
 */
static void efx_dl_try_add_device(struct efx_nic *efx,
				  struct efx_dl_driver *driver)
{
	struct efx_dl_handle *efx_handle;
	struct efx_dl_device *efx_dev;
	int rc;

	/* Allocate and initialise new efx_dl_device structure */
	efx_handle = kzalloc(sizeof(*efx_handle), GFP_KERNEL);
	efx_dev = &efx_handle->efx_dev;
	efx_handle->efx = efx;
	efx_dev->driver = driver;
	efx_dev->pci_dev = efx->pci_dev;
	INIT_LIST_HEAD(&efx_handle->port_node);
	INIT_LIST_HEAD(&efx_handle->driver_node);

	/* Attempt driver probe */
	rc = driver->probe(efx_dev, efx->net_dev,
			   efx->dl_info, efx->silicon_rev);
	if (rc)
		goto fail;

	/* Add device to per-driver and per-NIC lists */
	list_add_tail(&efx_handle->driver_node, &driver->device_list);
	list_add_tail(&efx_handle->port_node, &efx->dl_device_list);

	EFX_INFO(efx, "%s driverlink client registered\n", driver->name);
	return;

 fail:
	EFX_INFO(efx, "%s driverlink client skipped\n", driver->name);

	kfree(efx_dev);
}

/**
 * efx_dl_unregister_driver - unregister an Efx device driver
 * @driver:		Efx driverlink driver
 *
 * Unregisters an Efx driver.  The driver's remove() method will be
 * called for all Efx devices currently claimed by the driver.
 */
void efx_dl_unregister_driver(struct efx_dl_driver *driver)
{
	struct efx_dl_handle *efx_handle, *efx_handle_n;

	printk(KERN_INFO "Efx driverlink unregistering %s driver\n",
		 driver->name);

	/* Acquire lock.  We can't return failure, so have to use
	 * down() instead of down_interruptible()
	 */
	mutex_lock(&efx_driverlink_lock);

	/* Remove all devices claimed by the driver */
	list_for_each_entry_safe(efx_handle, efx_handle_n,
				 &driver->device_list, driver_node)
		efx_dl_del_device(&efx_handle->efx_dev);

	/* Remove driver from driver list */
	list_del(&driver->node);

	/* Release lock */
	mutex_unlock(&efx_driverlink_lock);
}
EXPORT_SYMBOL(efx_dl_unregister_driver);

/**
 * efx_dl_register_driver - register an Efx device driver
 * @driver:		Efx driverlink driver
 *
 * Registers a new Efx driver.  The driver's probe() method will be
 * called for all Efx NICs currently registered.
 *
 * Return a negative error code or 0 on success.
 */
int efx_dl_register_driver(struct efx_dl_driver *driver)
{
	struct efx_nic *efx;
	int rc;

	printk(KERN_INFO "Efx driverlink registering %s driver\n",
		 driver->name);

	/* Initialise driver list structures */
	INIT_LIST_HEAD(&driver->node);
	INIT_LIST_HEAD(&driver->device_list);

	/* Acquire lock */
	rc = mutex_lock_interruptible(&efx_driverlink_lock);
	if (rc)
		return rc;

	/* Add driver to driver list */
	list_add_tail(&driver->node, &efx_driver_list);

	/* Feed all existing devices to driver */
	list_for_each_entry(efx, &efx_port_list, dl_node)
		efx_dl_try_add_device(efx, driver);

	/* Release locks */
	mutex_unlock(&efx_driverlink_lock);

	return 0;
}
EXPORT_SYMBOL(efx_dl_register_driver);

void efx_dl_unregister_nic(struct efx_nic *efx)
{
	struct efx_dl_handle *efx_handle, *efx_handle_n;

	if (!efx)
		return;

	/* Acquire lock.  We can't return failure, so have to use
	 * down() instead of down_interruptible()
	 */
	mutex_lock(&efx_driverlink_lock);

	/* Remove all devices related to this NIC */
	list_for_each_entry_safe_reverse(efx_handle, efx_handle_n,
					 &efx->dl_device_list,
					 port_node)
		efx_dl_del_device(&efx_handle->efx_dev);

	/* Remove port from port list */
	list_del(&efx->dl_node);

	/* Release lock */
	mutex_unlock(&efx_driverlink_lock);
}

int efx_dl_register_nic(struct efx_nic *efx)
{
	struct efx_dl_driver *driver;
	int rc;

	/* Acquire lock */
	rc = mutex_lock_interruptible(&efx_driverlink_lock);
	if (rc)
		return rc;

	/* Add port to port list */
	list_add_tail(&efx->dl_node, &efx_port_list);

	/* Feed port to all existing drivers */
	list_for_each_entry(driver, &efx_driver_list, node)
		efx_dl_try_add_device(efx, driver);

	/* Release lock */
	mutex_unlock(&efx_driverlink_lock);

	return 0;
}

/*
 * Dummy callback implementations.
 *
 * To avoid a branch point on the fast-path, the callbacks are always
 * implemented - they are never NULL.
 */
#if defined(EFX_USE_FASTCALL)
static enum efx_veto fastcall
#else
static enum efx_veto
#endif
efx_dummy_tx_packet_callback(struct efx_dl_device *efx_dev, struct sk_buff *skb)
{
	/* Never veto the packet */
	return EFX_ALLOW_PACKET;
}

#if defined(EFX_USE_FASTCALL)
static enum efx_veto fastcall
#else
static enum efx_veto
#endif
efx_dummy_rx_packet_callback(struct efx_dl_device *efx_dev,
			     const char *pkt_buf, int len)
{
	/* Never veto the packet */
	return EFX_ALLOW_PACKET;
}

static void
efx_dummy_link_change_callback(struct efx_dl_device *efx_dev, int link_up)
{
}

static int
efx_dummy_request_mtu_callback(struct efx_dl_device *efx_dev, int new_mtu)
{
	/* Always allow */
	return 0;
}

static void
efx_dummy_mtu_changed_callback(struct efx_dl_device *efx_dev, int mtu)
{
	return;
}

static void efx_dummy_event_callback(struct efx_dl_device *efx_dev, void *event)
{
	return;
}

struct efx_dl_callbacks efx_default_callbacks = {
	.tx_packet	= efx_dummy_tx_packet_callback,
	.rx_packet	= efx_dummy_rx_packet_callback,
	.link_change	= efx_dummy_link_change_callback,
	.request_mtu	= efx_dummy_request_mtu_callback,
	.mtu_changed	= efx_dummy_mtu_changed_callback,
	.event		= efx_dummy_event_callback,
};

#define EFX_DL_UNREGISTER_CALLBACK(_port, _dev, _member)		\
	do {								\
		BUG_ON((_port)->dl_cb_dev._member != (_dev));		\
		(_port)->dl_cb._member =				\
			efx_default_callbacks._member;			\
		(_port)->dl_cb_dev._member = NULL;			\
	} while (0)


#define EFX_DL_REGISTER_CALLBACK(_port, _dev, _from, _member)		\
	if ((_from)->_member) {						\
		BUG_ON((_port)->dl_cb_dev._member != NULL);		\
		(_port)->dl_cb._member = (_from)->_member;		\
		(_port)->dl_cb_dev._member = _dev;			\
	}

/**
 * efx_dl_unregister_callbacks - unregister callbacks for an Efx NIC
 * @efx_dev:		Efx driverlink device
 * @callbacks:		Callback list
 *
 * This removes a set of callbacks registered with
 * efx_dl_register_callbacks().  It should be called as part of the
 * client's remove() method.
 *
 * The net driver will ensure that all callback functions have
 * returned to the net driver before efx_dl_unregister_callbacks()
 * returns.  Note that the device itself may still be running when the
 * client's remove() method is called.  The client must therefore
 * unhook its callbacks using efx_dl_unregister_callbacks() and only
 * then ensure that any delayed tasks triggered by callback methods
 * (e.g. scheduled tasklets) have completed.
 */
void efx_dl_unregister_callbacks(struct efx_dl_device *efx_dev,
				 struct efx_dl_callbacks *callbacks)
{
	struct efx_dl_handle *efx_handle = efx_dl_handle(efx_dev);
	struct efx_nic *efx = efx_handle->efx;

	/* Suspend net driver operations */
	efx_suspend(efx);

	EFX_INFO(efx, "removing callback hooks into %s driver\n",
		 efx_dev->driver->name);

	if (callbacks->tx_packet)
		EFX_DL_UNREGISTER_CALLBACK(efx, efx_dev, tx_packet);

	if (callbacks->rx_packet)
		EFX_DL_UNREGISTER_CALLBACK(efx, efx_dev, rx_packet);

	if (callbacks->link_change)
		EFX_DL_UNREGISTER_CALLBACK(efx, efx_dev, link_change);

	if (callbacks->request_mtu)
		EFX_DL_UNREGISTER_CALLBACK(efx, efx_dev, request_mtu);

	if (callbacks->mtu_changed)
		EFX_DL_UNREGISTER_CALLBACK(efx, efx_dev, mtu_changed);

	if (callbacks->event)
		EFX_DL_UNREGISTER_CALLBACK(efx, efx_dev, event);

	/* Resume net driver operations */
	efx_resume(efx);
}
EXPORT_SYMBOL(efx_dl_unregister_callbacks);

/**
 * efx_dl_register_callbacks - register callbacks for an Efx NIC
 * @efx_dev:		Efx driverlink device
 * @callbacks:		Callback list
 *
 * This registers a set of callback functions with the net driver.
 * These functions will be called at various key points to allow
 * external code to monitor and/or modify the behaviour of the network
 * driver.  Any of the callback function pointers may be %NULL if a
 * callback is not required.  The intended user of this mechanism is
 * the SFC char driver.
 *
 * This client should call efx_dl_register_callbacks() during its
 * probe() method.  The client must ensure that it also calls
 * efx_dl_unregister_callbacks() as part of its remove() method.
 *
 * Only one function may be registered for each callback per NIC.
 * If a requested callback is already registered for this NIC, this
 * function will return -%EBUSY.
 *
 * The device may already be running, so the client must be prepared
 * for callbacks to be triggered immediately after calling
 * efx_dl_register_callbacks().
 *
 * Return a negative error code or 0 on success.
 */
int efx_dl_register_callbacks(struct efx_dl_device *efx_dev,
			      struct efx_dl_callbacks *callbacks)
{
	struct efx_dl_handle *efx_handle = efx_dl_handle(efx_dev);
	struct efx_nic *efx = efx_handle->efx;
	int rc = 0;

	/* Suspend net driver operations */
	efx_suspend(efx);

	/* Check that the requested callbacks are not already hooked. */
	if ((callbacks->tx_packet && efx->dl_cb_dev.tx_packet) ||
	    (callbacks->rx_packet && efx->dl_cb_dev.rx_packet) ||
	    (callbacks->link_change && efx->dl_cb_dev.link_change) ||
	    (callbacks->request_mtu && efx->dl_cb_dev.request_mtu) ||
	    (callbacks->mtu_changed && efx->dl_cb_dev.mtu_changed) ||
	    (callbacks->event && efx->dl_cb_dev.event)) {
		rc = -EBUSY;
		goto out;
	}

	EFX_INFO(efx, "adding callback hooks to %s driver\n",
		 efx_dev->driver->name);

	/* Hook in callbacks.  For maximum speed, we never check to
	 * see whether these are NULL before calling; therefore we
	 * must ensure that they are never NULL.  If the set we're
	 * being asked to hook in is sparse, we leave the default
	 * values in place for the empty hooks.
	 */
	EFX_DL_REGISTER_CALLBACK(efx, efx_dev, callbacks, tx_packet);
	EFX_DL_REGISTER_CALLBACK(efx, efx_dev, callbacks, rx_packet);
	EFX_DL_REGISTER_CALLBACK(efx, efx_dev, callbacks, link_change);
	EFX_DL_REGISTER_CALLBACK(efx, efx_dev, callbacks, request_mtu);
	EFX_DL_REGISTER_CALLBACK(efx, efx_dev, callbacks, mtu_changed);
	EFX_DL_REGISTER_CALLBACK(efx, efx_dev, callbacks, event);

 out:
	/* Resume net driver operations */
	efx_resume(efx);

	return rc;
}
EXPORT_SYMBOL(efx_dl_register_callbacks);

/**
 * efx_dl_schedule_reset - schedule an Efx NIC reset
 * @efx_dev:		Efx driverlink device
 *
 * This schedules a hardware reset for a short time in the future.  It
 * can be called from any context, and so can be used when
 * efx_dl_reset() cannot be called.
 */
void efx_dl_schedule_reset(struct efx_dl_device *efx_dev)
{
	struct efx_dl_handle *efx_handle = efx_dl_handle(efx_dev);
	struct efx_nic *efx = efx_handle->efx;

	efx_schedule_reset(efx, RESET_TYPE_ALL);
}
EXPORT_SYMBOL(efx_dl_schedule_reset);

/*
 * Lock the driverlink layer before a reset
 * To avoid deadlock, efx_driverlink_lock needs to be acquired before
 * efx->suspend_lock.
 */
void efx_dl_reset_lock(void)
{
	/* Acquire lock */
	mutex_lock(&efx_driverlink_lock);
}

/*
 * Unlock the driverlink layer after a reset
 * This call must be matched against efx_dl_reset_lock.
 */
void efx_dl_reset_unlock(void)
{
	/* Acquire lock */
	mutex_unlock(&efx_driverlink_lock);
}

/*
 * Suspend ready for reset
 * This calls the reset_suspend method of all drivers registered to
 * the specified NIC.  It must only be called between
 * efx_dl_reset_lock and efx_dl_reset_unlock.
 */
void efx_dl_reset_suspend(struct efx_nic *efx)
{
	struct efx_dl_handle *efx_handle;
	struct efx_dl_device *efx_dev;

	BUG_ON(!mutex_is_locked(&efx_driverlink_lock));

	/* Call suspend method of each driver in turn */
	list_for_each_entry_reverse(efx_handle,
				    &efx->dl_device_list,
				    port_node) {
		efx_dev = &efx_handle->efx_dev;
		if (efx_dev->driver->reset_suspend)
			efx_dev->driver->reset_suspend(efx_dev);
	}
}

/*
 * Resume after a reset
 * This calls the reset_resume method of all drivers registered to the
 * specified NIC.  It must only be called between efx_dl_reset_lock
 * and efx_dl_reset_unlock.
 */
void efx_dl_reset_resume(struct efx_nic *efx, int ok)
{
	struct efx_dl_handle *efx_handle;
	struct efx_dl_device *efx_dev;

	BUG_ON(!mutex_is_locked(&efx_driverlink_lock));

	/* Call resume method of each driver in turn */
	list_for_each_entry(efx_handle, &efx->dl_device_list,
			    port_node) {
		efx_dev = &efx_handle->efx_dev;
		if (efx_dev->driver->reset_resume)
			efx_dev->driver->reset_resume(efx_dev, ok);
	}
}

/**
 * efx_dl_get_nic - obtain the Efx NIC for the given driverlink device
 * @efx_dev:		Efx driverlink device
 *
 * Get a pointer to the &struct efx_nic corresponding to
 * @efx_dev.  This can be used by driverlink clients built along with
 * the sfc driver, which may have intimate knowledge of its internals.
 */
struct efx_nic *efx_dl_get_nic(struct efx_dl_device *efx_dev)
{
	return efx_dl_handle(efx_dev)->efx;
}
EXPORT_SYMBOL(efx_dl_get_nic);
