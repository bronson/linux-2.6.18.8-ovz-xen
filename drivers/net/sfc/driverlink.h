/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005:      Fen Systems Ltd.
 * Copyright 2006:      Solarflare Communications Inc,
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

#ifndef EFX_DRIVERLINK_H
#define EFX_DRIVERLINK_H

/* Forward declarations */
struct efx_dl_device;
struct efx_nic;

/*
 * Efx driverlink
 *
 * This header file defines the portions of the Efx driverlink
 * interface that are used only within the sfc module.  It also
 * declares efx_dl_get_nic(), which may be used by sfc_mtd
 * and any other module built along with sfc.
 */


/* Efx callback devices
 *
 * A list of the devices that own each callback. The partner to
 * struct efx_dl_callbacks
 */
struct efx_dl_cb_devices {
	/* Device owning the tx_packet callback */
	struct efx_dl_device *tx_packet;
	/* Device owning the rx_packet callback */
	struct efx_dl_device *rx_packet;
	/* Device owning the link_change callback. */
	struct efx_dl_device *link_change;
	/* Device owning the request_mtu callback. */
	struct efx_dl_device *request_mtu;
	/* Device owning the mtu_changed callback. */
	struct efx_dl_device *mtu_changed;
	/* Device owning the event callback. */
	struct efx_dl_device *event;
};

/* No-op callbacks used for initialisation */
extern struct efx_dl_callbacks efx_default_callbacks;

/* Macro used to invoke callbacks */
#define EFX_DL_CALLBACK(_port, _name, ...)				\
	(_port)->dl_cb._name((_port)->dl_cb_dev._name, __VA_ARGS__)

/* Register an Efx NIC */
extern int efx_dl_register_nic(struct efx_nic *efx);

/* Unregister an Efx NIC */
extern void efx_dl_unregister_nic(struct efx_nic *efx);

/* Lock the driverlink layer prior to a reset */
extern void efx_dl_reset_lock(void);

/* Unlock the driverlink layer following a reset */
extern void efx_dl_reset_unlock(void);

/* Suspend all drivers prior to a hardware reset */
extern void efx_dl_reset_suspend(struct efx_nic *efx);

/* Resume all drivers after a hardware reset */
extern void efx_dl_reset_resume(struct efx_nic *efx, int ok);

/* Obtain the Efx NIC for the given driverlink device. */
extern struct efx_nic *efx_dl_get_nic(struct efx_dl_device *efx_dev);

#endif /* EFX_DRIVERLINK_H */
