/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2007:      Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed by Solarflare Communications <linux-net-drivers@solarflare.com>
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

#ifndef EFX_ENUM_H
#define EFX_ENUM_H

/**
 * enum efx_loopback_mode - loopback modes
 * @LOOPBACK_NONE: no loopback
 * @LOOPBACK_NEAR: loopback nearest to bus
 * @LOOPBACK_MAC: loopback within MAC unspecified level
 * @LOOPBACK_XGMII: loopback within MAC at XGMII level
 * @LOOPBACK_XGXS: loopback within MAC at XGXS level
 * @LOOPBACK_XAUI: loopback within MAC at XAUI level
 * @LOOPBACK_PHY: loopback within PHY unspecified level
 * @LOOPBACK_PHYXS: loopback within PHY at PHYXS level
 * @LOOPBACK_PCS: loopback within PHY at PCS level
 * @LOOPBACK_PMAPMD: loopback within PHY at PMAPMD level
 * @LOOPBACK_FAR: loopback furthest from bus
 * @LOOPBACK_NETWORK: reflecting loopback (even further than furthest!)
 */
/* Please keep in order and up-to-date w.r.t the following two #defines */
enum efx_loopback_mode {
	LOOPBACK_NONE = 0,
	LOOPBACK_NEAR = 1,
	LOOPBACK_MAC = 2,
	LOOPBACK_XGMII = 3,
	LOOPBACK_XGXS = 4,
	LOOPBACK_XAUI = 5,
	LOOPBACK_PHY = 6,
	LOOPBACK_PHYXS = 7,
	LOOPBACK_PCS = 8,
	LOOPBACK_PMAPMD = 9,
	LOOPBACK_FAR = 10,
	LOOPBACK_NETWORK = 11,
	LOOPBACK_MAX
};
#define LOOPBACK_TEST_MAX LOOPBACK_FAR

/* These loopbacks occur within the controller */
#define LOOPBACKS_10G_INTERNAL ((1 << LOOPBACK_XGMII)| \
				(1 << LOOPBACK_XGXS) | \
				(1 << LOOPBACK_XAUI))

#define LOOPBACKS_1G_INTERNAL (1 << LOOPBACK_MAC)

#define LOOPBACK_MASK(_efx)			\
	(1 << (_efx)->loopback_mode)

#define LOOPBACK_INTERNAL(_efx)					\
	(((LOOPBACKS_10G_INTERNAL | LOOPBACKS_1G_INTERNAL) &	\
	  LOOPBACK_MASK(_efx)) ? 1 : 0)

#define LOOPBACK_CHANGED(_from, _to, _mask)		\
	((LOOPBACK_MASK(_from) ^ LOOPBACK_MASK(_to)) &	\
	 (_mask) ? 1 : 0)

#define LOOPBACK_OUT_OF(_from, _to, _mask)		\
	(((LOOPBACK_MASK(_from) & (_mask)) &&		\
	  ((LOOPBACK_MASK(_to) & (_mask)) == 0)) ? 1 : 0)

/*****************************************************************************/

/**
 * enum reset_type - reset types
 *
 * %RESET_TYPE_INVSIBLE, %RESET_TYPE_ALL, %RESET_TYPE_WORLD and
 * %RESET_TYPE_DISABLE specify the method/scope of the reset.  The
 * other valuesspecify reasons, which efx_schedule_reset() will choose
 * a method for.
 *
 * @RESET_TYPE_INVISIBLE: don't reset the PHYs or interrupts
 * @RESET_TYPE_ALL: reset everything but PCI core blocks
 * @RESET_TYPE_WORLD: reset everything, save & restore PCI config
 * @RESET_TYPE_DISABLE: disable NIC
 * @RESET_TYPE_MONITOR: reset due to hardware monitor
 * @RESET_TYPE_INT_ERROR: reset due to internal error
 * @RESET_TYPE_RX_RECOVERY: reset to recover from RX datapath errors
 */
enum reset_type {
	RESET_TYPE_NONE = -1,
	RESET_TYPE_INVISIBLE = 0,
	RESET_TYPE_ALL = 1,
	RESET_TYPE_WORLD = 2,
	RESET_TYPE_DISABLE = 3,
	RESET_TYPE_MAX_METHOD,
	RESET_TYPE_MONITOR,
	RESET_TYPE_INT_ERROR,
	RESET_TYPE_RX_RECOVERY,
	RESET_TYPE_RX_DESC_FETCH,
	RESET_TYPE_TX_DESC_FETCH,
	RESET_TYPE_MAX,
};

#endif /* EFX_ENUM_H */
