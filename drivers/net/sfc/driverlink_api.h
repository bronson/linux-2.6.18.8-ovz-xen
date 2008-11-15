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

#ifndef EFX_DRIVERLINK_API_H
#define EFX_DRIVERLINK_API_H

#include <linux/list.h> /* for struct list_head */
#if !defined(EFX_USE_FASTCALL)
	#include <linux/version.h>
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
		#define EFX_USE_FASTCALL yes
		#include <linux/linkage.h>
	#endif
#endif

/**
 * DOC: Efx driverlink API
 *
 * This file must be included by any driver that wishes to attach to
 * devices claimed by the Solarflare NIC driver (sfc). It allows separate
 * kernel modules to expose other functionality offered by the NIC, with
 * the sfc driver remaining in overall control.
 *
 * Overview:
 *
 * Driverlink clients define a &struct efx_dl_driver, and register
 * this structure with the driverlink layer using
 * efx_dl_register_driver(), which is exported by the sfc driver.
 *
 * The probe() routine of each driverlink client driver is called by
 * the driverlink layer for each physical port in the system, after
 * the sfc driver has performed start-of-day hardware initialisation
 * and self-test. If ports are added or removed via pci hotplug then
 * the &struct efx_dl_driver probe() or remove() routines are called
 * as appropriate.
 *
 * If the port doesn't provide the necessary hardware resources for a
 * client, then that client can return failure from its probe()
 * routine. Information provided to the client driver at probe time
 * includes
 *
 * Each probe() routine is given a unique &struct efx_dl_device per
 * port, which means it can safely use the @priv member to store any
 * useful state it needs. The probe routine also has the opportunity
 * to provide a &struct efx_dl_callbacks via
 * efx_dl_register_callbacks(), which allows the client to intercept
 * the sfc driver's operations at strategic points.
 *
 * Occasionally, the underlying Efx device may need to be reset to
 * recover from an error condition.  The client's reset_suspend() and
 * reset_resume() methods [if provided] will be called to enable the
 * client to suspend operations and preserve any state before the
 * reset.  The client can itself request a reset using efx_dl_reset()
 * or efx_dl_schedule_reset(), should it detect an error condition
 * necessitating a reset.
 *
 * Example:
 *
 * The MTD driver (mtd.c) uses the driverlink layer.
 */

/* Forward declarations */
struct pci_dev;
struct net_device;
struct sk_buff;
struct efx_dl_device;
struct efx_dl_device_info;

/*
 * This is used to guard against the registration of driverlink
 * clients using an incorrect version of the API.
 */
#define EFX_DRIVERLINK_API_VERSION 1


/**
 * struct efx_dl_driver - An Efx driverlink device driver
 *
 * This is the analogue of a struct pci_driver for a normal PCI
 * driver.  Driverlink clients should register themselves using
 * efx_dl_register_driver() at module initialisation, and deregister
 * themselves using efx_dl_unregister_driver() at module exit.
 *
 * All calls to members of efx_dl_driver are serialised by a single
 * semaphore, so you are allowed to sleep in these functions. Take care
 * to not call driverlink methods from within these callbacks, otherwise
 * a deadlock is possible.
 *
 * @name: Name of the driver
 * @probe: Called when device added
 * @remove: Called when device removed
 * @reset_suspend: Called before device is reset
 * @reset_resume: Called after device is reset
 */
struct efx_dl_driver {
	const char *name;

	/*
	 * probe - Handle device addition.
	 * @efx_dev:		Efx driverlink device
	 * @net_dev:		The net_dev relevant to this port
	 * @dev_info:		A linked list of device information.
	 * @silicon_rev:	Silicon revision name.
	 *
	 * This will be called after driverlink client registration for
	 * every port on the system, and for every port that appears
	 * thereafter via hotplug.
	 *
	 * The client may use either @efx_dev->pci_dev, the dev_info linked
	 * list of available driver information, or the silicon revision
	 * name to determine if they can support this port. If they can,
	 * they should return 0 to indicate the probe was successful. Any
	 * other return code indicates that the probe failed, and the
	 * @efx_dl_dev will be invalidated.
	 *
	 * The client should perform whatever initialisation it
	 * requires, and store a pointer to its private data in
	 * @efx_dl_dev->priv (which is not shared between clients).
	 * It may also wish to hook in a callbacks table using
	 * efx_dl_register_callbacks().
	 *
	 * Return a negative error code or 0 on success.
	 */
	int (*probe) (struct efx_dl_device *efx_dl_dev,
		      const struct net_device *net_dev,
		      const struct efx_dl_device_info *dev_info,
		      const char *silicon_rev);

	/*
	 * remove - Handle device removal.
	 * @efx_dev:		Efx driverlink device
	 *
	 * This will be called at driver exit (or hotplug removal) for
	 * each registered driverlink client.
	 *
	 * The client must ensure that it has finished all operations
	 * using this device before returning from this method.  If it
	 * has hooked in a callbacks table using
	 * efx_dl_register_callbacks(), it must unhook it using
	 * efx_dl_unregister_callbacks(), and then ensure that all
	 * callback-triggered operations (e.g. scheduled tasklets)
	 * have completed before returning.  (It does not need to
	 * explicitly wait for callback methods to finish executing,
	 * since efx_dl_unregister_callbacks() will sleep until all
	 * callbacks have returned anyway.)
	 *
	 * Note that the device itself may not have been removed; it
	 * may be simply that the client is being unloaded
	 * via efx_dl_unregister_driver(). In this case other clients
	 * (and the sfc driver itself) will still be using the device,
	 * so the client cannot assume that the device itself is quiescent.
	 * In particular, callbacks may continue to be triggered at any
	 * point until efx_dl_unregister_callbacks() is called.
	 */
	void (*remove) (struct efx_dl_device *efx_dev);

	/*
	 * reset_suspend - Suspend ready for reset.
	 * @efx_dev:		Efx driverlink device
	 *
	 * This method will be called immediately before a hardware
	 * reset (which may or may not have been initiated by the
	 * driverlink client).  This client must save any state that it
	 * will need to restore after the reset, and suspend all
	 * operations that might access the hardware.  It must not
	 * return until the client can guarantee to have stopped
	 * touching the hardware.
	 *
	 * It is guaranteed that callbacks will be inactive by the
	 * time this method is called; the driverlink layer will
	 * already have prevented new callbacks being made and waited
	 * for all callbacks functions to return before calling
	 * reset_suspend().  However, any delayed work scheduled by
	 * the callback functions (e.g. tasklets) may not yet have
	 * completed.
	 *
	 * This method is allowed to sleep, so waiting on tasklets,
	 * work queues etc. is permitted.  There will always be a
	 * corresponding call to the reset_resume() method, so it is
	 * safe to e.g. down a semaphore within reset_suspend() and up
	 * it within reset_resume().  (However, you obviously cannot
	 * do the same with a spinlock).
	 *
	 * Note that the reset operation may be being carried out in
	 * the context of scheduled work, so you cannot use
	 * flush_scheduled_work() to ensure that any work you may have
	 * scheduled has completed.
	 *
	 * During hardware reset, there is a chance of receiving
	 * spurious interrupts, so the client's ISR (if any) should be
	 * unhooked or otherwise disabled.
	 */
	void (*reset_suspend) (struct efx_dl_device *efx_dev);

	/*
	 * reset_resume - Restore after a reset.
	 * @efx_dev:		Efx driverlink device
	 * @ok:			Reset success indicator
	 *
	 * This method will be called after a hardware reset.  There
	 * will always have been a corresponding call to the
	 * reset_suspend() method beforehand.
	 *
	 * If @ok is non-zero, the client should restore the state
	 * that it saved during the call to reset_suspend() and resume
	 * normal operations.
	 *
	 * If @ok is zero, the reset operation has failed and the
	 * hardware is currently in an unusable state.  In this case,
	 * the client should release any locks taken out by
	 * reset_suspend(), but should not take any other action; in
	 * particular, it must not access the hardware, nor resume
	 * normal operations.  The hardware is effectively dead at
	 * this point, and our sole aim is to avoid deadlocking or
	 * crashing the host.
	 *
	 * The driverlink layer will still be locked when
	 * reset_resume() is called, so the client may not call
	 * driverlink functions.  In particular, if the reset failed,
	 * the client must not call efx_dl_unregister_callbacks() at
	 * this point; it should wait until remove() is called.
	 */
	void (*reset_resume) (struct efx_dl_device *efx_dev, int ok);

/* private: */
	struct list_head node;
	struct list_head device_list;
};

/**
 * DOC: Efx driverlink device information
 *
 * Each &struct efx_dl_device makes certain hardware resources visible
 * to driverlink clients, and they describe which resources are
 * available by passing a linked list of &struct efx_dl_device_info
 * into the probe() routine.
 *
 * The driverlink client's probe function can iterate through the linked list,
 * and provided that it understands the resources that are exported, it can
 * choose to make use of them through an external interface.
 */

/**
 * enum efx_dl_device_info_type - Device information identifier.
 *
 * Each distinct hardware resource API will have a member in this
 * enumeration.
 *
 * @EFX_DL_FALCON_RESOURCES: Information type is &struct efx_dl_falcon_resources
 */
enum efx_dl_device_info_type {
	/** Falcon resources available for export */
	EFX_DL_FALCON_RESOURCES = 0,
};

/**
 * struct efx_dl_device_info - device information structure
 * @next: Link to next structure, if any
 * @type: Type code for this structure
 *
 * This structure is embedded in other structures provided by the
 * driverlink device provider, and implements a linked list of
 * resources pertinent to a driverlink client.
 *
 * Example: &struct efx_dl_falcon_resources
 */
struct efx_dl_device_info {
	struct efx_dl_device_info *next;
	enum efx_dl_device_info_type type;
};

/**
 * enum efx_dl_falcon_resource_flags - Falcon resource information flags.
 *
 * Flags that describe hardware variations for the described Falcon based port.
 *
 * @EFX_DL_FALCON_DUAL_FUNC: Port is dual-function.
 *	Certain silicon revisions have two pci functions, and require
 *	certain hardware resources to be accessed via the secondary
 *	function. See the discussion of @pci_dev in &struct efx_dl_device
 *	below.
 * @EFX_DL_FALCON_USE_MSI: Port is initialised to use MSI/MSI-X interrupts.
 *	Falcon supports traditional legacy interrupts and MSI/MSI-X
 *	interrupts. Since the sfc driver supports either, as a run
 *	time configuration, driverlink drivers need to be aware of which
 *	one to use for their interrupting resources.
 */
enum efx_dl_falcon_resource_flags {
	EFX_DL_FALCON_DUAL_FUNC = 0x1,
	EFX_DL_FALCON_USE_MSI = 0x2,
};

/**
 * struct efx_dl_falcon_resources - Falcon resource information.
 *
 * This structure describes Falcon hardware resources available for
 * use by a driverlink driver.
 *
 * @hdr: Resource linked list header
 * @biu_lock: Register access lock.
 *	Some Falcon revisions require register access for configuration
 *	registers to be serialised between ports and PCI functions.
 *	The sfc driver will provide the appropriate lock semantics for
 *	the underlying hardware.
 * @buffer_table_min: First available buffer table entry
 * @buffer_table_max: Last available buffer table entry + 1
 * @evq_timer_min: First available event queue with timer
 * @evq_timer_max: Last available event queue with timer + 1
 * @evq_int_min: First available event queue with interrupt
 * @evq_int_max: Last available event queue with interrupt + 1
 * @rxq_min: First available RX queue
 * @rxq_max: Last available RX queue + 1
 * @txq_min: First available TX queue
 * @txq_max: Last available TX queue + 1
 * @flags: Hardware variation flags
 */
struct efx_dl_falcon_resources {
	struct efx_dl_device_info hdr;
	spinlock_t *biu_lock;
	unsigned buffer_table_min, buffer_table_max;
	unsigned evq_timer_min, evq_timer_max;
	unsigned evq_int_min, evq_int_max;
	unsigned rxq_min, rxq_max;
	unsigned txq_min, txq_max;
	enum efx_dl_falcon_resource_flags flags;
};

/**
 * struct efx_dl_device - An Efx driverlink device.
 *
 * @pci_dev: Underlying PCI device.
 *	This is the PCI device used by the sfc driver.  It will
 *	already have been enabled for bus-mastering DMA etc.
 * @priv: Driver private data
 *	Driverlink clients can use this to store a pointer to their
 *	internal per-device data structure. Each (driver, device)
 *	tuple has a separate &struct efx_dl_device, so clients can use
 *	this @priv field independently.
 * @driver: Efx driverlink driver for this device
 */
struct efx_dl_device {
	struct pci_dev *pci_dev;
	void *priv;
	struct efx_dl_driver *driver;
};

/**
 * enum efx_veto - Packet veto request flag.
 *
 * This is the return type for the rx_packet() and tx_packet() methods
 * in &struct efx_dl_callbacks.
 *
 * @EFX_ALLOW_PACKET: Packet may be transmitted/received
 * @EFX_VETO_PACKET: Packet must not be transmitted/received
 */
enum efx_veto {
	EFX_ALLOW_PACKET = 0,
	EFX_VETO_PACKET = 1,
};

/**
 * struct efx_dl_callbacks - Efx callbacks
 *
 * These methods can be hooked in to the sfc driver via
 * efx_dl_register_callbacks().  They allow clients to intercept and/or
 * modify the behaviour of the sfc driver at predetermined points.
 *
 * For efficiency, only one client can hook each callback.
 *
 * Since these callbacks are called on packet transmit and reception
 * paths, clients should avoid acquiring locks or allocating memory.
 *
 * @tx_packet: Called when packet is about to be transmitted
 * @rx_packet: Called when packet is received
 * @link_change: Called when link status has changed
 * @request_mtu: Called to request MTU change
 * @mtu_changed: Called when MTU has been changed
 * @event: Called when NIC event is not handled by the sfc driver
 */
struct efx_dl_callbacks {
	/*
	 * tx_packet - Packet about to be transmitted.
	 * @efx_dev:		Efx driverlink device
	 * @skb:		Socket buffer containing the packet to be sent
	 *
	 * This method is called for every packet about to be
	 * transmitted.  It allows the client to snoop on traffic sent
	 * via the kernel queues.
	 *
	 * The method may return %EFX_VETO_PACKET in order to prevent
	 * the sfc driver from transmitting the packet.  The net
	 * driver will then discard the packet.  If the client wishes
	 * to retain a reference to the packet data after returning
	 * %EFX_VETO_PACKET, it must obtain its own copy of the
	 * packet (e.g. by calling skb_get(), or by copying out the
	 * packet data to an external buffer).
	 *
	 * This method must return quickly, since it will have a
	 * direct performance impact upon the sfc driver.  It will be
	 * called with interrupts disabled (and may be called in
	 * interrupt context), so may not sleep. Since the sfc driver
	 * may have multiple TX queues, running in parallel, please avoid
	 * the need for locking if it all possible.
	 */
#if defined(EFX_USE_FASTCALL)
	enum efx_veto fastcall (*tx_packet) (struct efx_dl_device *efx_dev,
					     struct sk_buff *skb);
#else
	enum efx_veto (*tx_packet) (struct efx_dl_device *efx_dev,
				    struct sk_buff *skb);
#endif

	/*
	 * rx_packet - Packet received.
	 * @efx_dev:		Efx driverlink device
	 * @pkt_hdr:		Pointer to received packet
	 * @pkt_len:		Length of received packet
	 *
	 * This method is called for every received packet.  It allows
	 * the client to snoop on traffic received by the kernel
	 * queues.
	 *
	 * The method may return %EFX_VETO_PACKET in order to prevent
	 * the sfc driver from passing the packet to the kernel.  The net
	 * driver will then discard the packet.
	 *
	 * This method must return quickly, since it will have a
	 * direct performance impact upon the sfc driver.  It is
	 * called in tasklet context, so may not sleep.  Note that
	 * there are per-channel tasklets in the sfc driver, so
	 * rx_packet() may be called simultaneously on different CPUs
	 * and must lock appropriately.  The design of the sfc driver
	 * allows for lockless operation between receive channels, so
	 * please avoid the need for locking if at all possible.
	 */
#if defined(EFX_USE_FASTCALL)
	enum efx_veto fastcall (*rx_packet) (struct efx_dl_device *efx_dev,
					     const char *pkt_hdr, int pkt_len);
#else
	enum efx_veto (*rx_packet) (struct efx_dl_device *efx_dev,
				    const char *pkt_hdr, int pkt_len);
#endif

	/*
	 * link_change - Link status change.
	 * @efx_dev:		Efx driverlink device
	 * @link_up:		Link up indicator
	 *
	 * This method is called to inform the driverlink client
	 * whenever the PHY link status changes.  By the time this
	 * function is called, the MAC has already been reconfigured
	 * with the new autonegotiation settings from the PHY.
	 *
	 * This method is called from tasklet context and may not
	 * sleep.
	 */
	void (*link_change) (struct efx_dl_device *efx_dev, int link_up);

	/*
	 * request_mtu: Request MTU change.
	 * @efx_dev:		Efx driverlink device
	 * @new_mtu:		Requested new MTU
	 *
	 * This method is called whenever the user requests an MTU
	 * change on an interface.  The client may return an error, in
	 * which case the MTU change request will be denied.  If the
	 * client returns success, the MAC will be reconfigured with a
	 * new maxmimum frame length equal to
	 * EFX_MAX_FRAME_LEN(new_mtu).  The client will be notified
	 * via the mtu_changed() method once the MAC has been
	 * reconfigured.
	 *
	 * The current MTU for the port can be obtained via
	 * efx_dl_get_netdev(efx_dl_device)->mtu.
	 *
	 * The sfc driver guarantees that no other callback functions
	 * are in progress when this method is called.  This function
	 * is called in process context and may sleep.
	 *
	 * Return a negative error code or 0 on success.
	 */
	int (*request_mtu) (struct efx_dl_device *efx_dev, int new_mtu);

	/*
	 * mtu_changed - MTU has been changed.
	 * @efx_dev:		Efx driverlink device
	 * @mtu:		The new MTU
	 *
	 * This method is called once the MAC has been reconfigured
	 * with a new MTU.  There will have been a preceding call to
	 * request_mtu().
	 *
	 * The sfc driver guarantees that no other callback functions
	 * are in progress when this method is called.  This function
	 * is called in process context and may sleep.
	 */
	void (*mtu_changed) (struct efx_dl_device *efx_dev, int mtu);

	/*
	 * event - Event callback.
	 * @efx_dev:		Efx driverlink device
	 * @p_event:		Pointer to event
	 *
	 * This method is called for each event that is not handled by the
	 * sfc driver.
	 */
	void (*event) (struct efx_dl_device *efx_dev, void *p_event);
};

/* Include API version number in symbol used for efx_dl_register_driver */
#define efx_dl_stringify_1(x, y) x ## y
#define efx_dl_stringify_2(x, y) efx_dl_stringify_1(x, y)
#define efx_dl_register_driver					\
	efx_dl_stringify_2(efx_dl_register_driver_api_ver_,	\
			   EFX_DRIVERLINK_API_VERSION)

extern int efx_dl_register_driver(struct efx_dl_driver *driver);

extern void efx_dl_unregister_driver(struct efx_dl_driver *driver);

extern int efx_dl_register_callbacks(struct efx_dl_device *efx_dev,
				     struct efx_dl_callbacks *callbacks);

extern void efx_dl_unregister_callbacks(struct efx_dl_device *efx_dev,
					struct efx_dl_callbacks *callbacks);

extern void efx_dl_schedule_reset(struct efx_dl_device *efx_dev);

/**
 * efx_dl_for_each_device_info_matching - iterate an efx_dl_device_info list
 * @_dev_info: Pointer to first &struct efx_dl_device_info
 * @_type: Type code to look for
 * @_info_type: Structure type corresponding to type code
 * @_field: Name of &struct efx_dl_device_info field in the type
 * @_p: Iterator variable
 *
 * Example:
 *
 * static int driver_dl_probe(... const struct efx_dl_device_info *dev_info ...)
 * {
 *        struct efx_dl_falcon_resources *res;
 *
 *        efx_dl_for_each_device_info_matching(dev_info,EFX_DL_FALCON_RESOURCES,
 *                                             struct efx_dl_falcon_resources,
 *                                             hdr, res) {
 *                if (res->flags & EFX_DL_FALCON_DUAL_FUNC) {
 *                          .....
 *                }
 *        }
 * }
 */
#define efx_dl_for_each_device_info_matching(_dev_info, _type,		\
					     _info_type, _field, _p)	\
	for ((_p) = container_of((_dev_info), _info_type, _field);	\
	     (_p) != NULL;						\
	     (_p) = container_of((_p)->_field.next, _info_type, _field))\
		if ((_p)->_field.type != _type)				\
			continue;					\
		else

/**
 * efx_dl_search_device_info - search an efx_dl_device_info list
 * @_dev_info: Pointer to first &struct efx_dl_device_info
 * @_type: Type code to look for
 * @_info_type: Structure type corresponding to type code
 * @_field: Name of &struct efx_dl_device_info member in this type
 * @_p: Result variable
 *
 * Example:
 *
 * static int driver_dl_probe(... const struct efx_dl_device_info *dev_info ...)
 * {
 *        struct efx_dl_falcon_resources *res;
 *
 *        efx_dl_search_device_info(dev_info, EFX_DL_FALCON_RESOURCES,
 *                                  struct efx_dl_falcon_resources, hdr, res);
 *        if (res != NULL) {
 *                 ....
 *        }
 * }
 */
#define efx_dl_search_device_info(_dev_info, _type, _info_type,		\
				  _field, _p)				\
	efx_dl_for_each_device_info_matching((_dev_info), (_type),	\
					     _info_type, _field, (_p))	\
		break;

#endif /* EFX_DRIVERLINK_API_H */
