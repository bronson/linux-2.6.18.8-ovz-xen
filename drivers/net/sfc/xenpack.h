/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2006:      Solarflare Communications Inc,
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

#ifndef EFX_XENPACK_H
#define EFX_XENPACK_H

/* Exported functions from Xenpack standard PHY control */

#include "mdio_10g.h"

/****************************************************************************/
/* XENPACK MDIO register extensions */
#define MDIO_XP_LASI_RX_CTRL	(0x9000)
#define MDIO_XP_LASI_TX_CTRL	(0x9001)
#define MDIO_XP_LASI_CTRL	(0x9002)
#define MDIO_XP_LASI_RX_STAT	(0x9003)
#define MDIO_XP_LASI_TX_STAT	(0x9004)
#define MDIO_XP_LASI_STAT	(0x9005)

/* Control/Status bits */
#define XP_LASI_LS_ALARM	(1 << 0)
#define XP_LASI_TX_ALARM	(1 << 1)
#define XP_LASI_RX_ALARM	(1 << 2)
/* These two are Quake vendor extensions to the standard XENPACK defines */
#define XP_LASI_LS_INTB		(1 << 3)
#define XP_LASI_TEST		(1 << 7)

/* Enable LASI interrupts for PHY */
static inline void xenpack_enable_lasi_irqs(struct efx_nic *efx)
{
	int reg;
	int phy_id = efx->mii.phy_id;
	/* Read to clear LASI status register */
	reg = mdio_clause45_read(efx, phy_id, MDIO_MMD_PMAPMD,
				 MDIO_XP_LASI_STAT);

	/* Enable LASI interrupts from PMA/PMD */
	mdio_clause45_write(efx, phy_id, MDIO_MMD_PMAPMD,
			    MDIO_XP_LASI_CTRL, XP_LASI_LS_ALARM);
}

/* Read the LASI interrupt status to clear the interrupt. */
static inline int xenpack_clear_lasi_irqs(struct efx_nic *efx)
{
	/* Read to clear link status alarm */
	return mdio_clause45_read(efx, efx->mii.phy_id,
				  MDIO_MMD_PMAPMD, MDIO_XP_LASI_STAT);
}

/* Turn off LASI interrupts */
static inline void xenpack_disable_lasi_irqs(struct efx_nic *efx)
{
	/* Turn LASI interrupts off */
	mdio_clause45_write(efx, efx->mii.phy_id, MDIO_MMD_PMAPMD,
			    MDIO_XP_LASI_CTRL, 0);
}

#endif /* EFX_XENPACK_H */
