/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005:      Fen Systems Ltd.
 * Copyright 2006-2007: Solarflare Communications Inc,
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

#include "net_driver.h"
#include <linux/ethtool.h>
#include "gmii.h"
#include "phy.h"

/* Marvell 88E1111 "Alaska" PHY control */
#define ALASKA_PHY_SPECIFIC 16
#define ALASKA_ALLOW_SLEEP 0x0200

#define ALASKA_EXTENDED_CONTROL 20
#define EXTENDED_LINE_LOOPBACK 0x8000

#define ALASKA_LED_CONTROL 24
#define LED_BLINK_MASK 0x0700
#define LED_BLINK_FAST 0x0100
#define LED_BLINK_SLOW 0x0300
#define LED_TX_CTRL_MASK 0x0041
#define LED_TX_CTRL_LINK_AND_ACTIVITY 0x0001

#define ALASKA_LED_OVERRIDE 25
#define LED_LINK1000_MASK 0x0030
#define LED_LINK1000_BLINK 0x0010
#define LED_TX_MASK 0x0003
#define LED_TX_BLINK 0x0001

static void alaska_reconfigure(struct efx_nic *efx)
{
	struct mii_if_info *gmii = &efx->mii;
	u32 bmcr, phy_ext;

	/* Configure line loopback if requested */
	phy_ext = gmii->mdio_read(gmii->dev, gmii->phy_id,
				  ALASKA_EXTENDED_CONTROL);
	if (efx->loopback_mode == LOOPBACK_NETWORK)
		phy_ext |= EXTENDED_LINE_LOOPBACK;
	else
		phy_ext &= ~EXTENDED_LINE_LOOPBACK;
	gmii->mdio_write(gmii->dev, gmii->phy_id, ALASKA_EXTENDED_CONTROL,
			 phy_ext);

	/* Configure PHY loopback if requested */
	bmcr = gmii->mdio_read(gmii->dev, gmii->phy_id, MII_BMCR);
	if (efx->loopback_mode == LOOPBACK_PHY)
		bmcr |= BMCR_LOOPBACK;
	else
		bmcr &= ~BMCR_LOOPBACK;
	gmii->mdio_write(gmii->dev, gmii->phy_id, MII_BMCR, bmcr);

	/* Read link up status */
	if (efx->loopback_mode == LOOPBACK_NONE)
		efx->link_up = mii_link_ok(gmii);
	else
		efx->link_up = 1;

	/* Determine link options from PHY */
	if (gmii->force_media) {
		efx->link_options = gmii_forced_result(bmcr);
	} else {
		int lpa = gmii_lpa(gmii);
		int adv = gmii_advertised(gmii);
		efx->link_options = gmii_nway_result(adv & lpa);
	}
}

static void alaska_clear_interrupt(struct efx_nic *efx)
{
	struct mii_if_info *gmii = &efx->mii;

	/* Read interrupt status register to clear */
	gmii->mdio_read(gmii->dev, gmii->phy_id, GMII_ISR);
}

static int alaska_init(struct efx_nic *efx)
{
	struct mii_if_info *gmii = &efx->mii;
	u32 ier, leds, ctrl_1g, phy_spec;

	/* Read ISR to clear any outstanding PHY interrupts */
	gmii->mdio_read(gmii->dev, gmii->phy_id, GMII_ISR);

	/* Enable PHY interrupts */
	ier = gmii->mdio_read(gmii->dev, gmii->phy_id, GMII_IER);
	ier |= IER_LINK_CHG;
	gmii->mdio_write(gmii->dev, gmii->phy_id, GMII_IER, ier);

	/* Remove 1G half-duplex as unsupported in Mentor MAC */
	ctrl_1g = gmii->mdio_read(gmii->dev, gmii->phy_id, MII_CTRL1000);
	ctrl_1g &= ~(ADVERTISE_1000HALF);
	gmii->mdio_write(gmii->dev, gmii->phy_id, MII_CTRL1000, ctrl_1g);

	/*
	 * The PHY can save power when there is no external connection
	 * (sleep mode).  However, this is incompatible with PHY
	 * loopback, and if enable and disable it quickly the PHY can
	 * go to sleep even when sleep mode is disabled.  (SFC bug
	 * 9309.)  Therefore we disable it all the time.
	 */
	phy_spec = gmii->mdio_read(gmii->dev, gmii->phy_id,
				   ALASKA_PHY_SPECIFIC);
	phy_spec &= ~ALASKA_ALLOW_SLEEP;
	gmii->mdio_write(gmii->dev, gmii->phy_id, ALASKA_PHY_SPECIFIC,
			 phy_spec);

	/* Configure LEDs */
	leds = gmii->mdio_read(gmii->dev, gmii->phy_id, ALASKA_LED_CONTROL);
	leds &= ~(LED_BLINK_MASK | LED_TX_CTRL_MASK);
	leds |= (LED_BLINK_FAST | LED_TX_CTRL_LINK_AND_ACTIVITY);
	gmii->mdio_write(gmii->dev, gmii->phy_id, ALASKA_LED_CONTROL, leds);

	return 0;
}

static void alaska_fini(struct efx_nic *efx)
{
	struct mii_if_info *gmii = &efx->mii;
	u32 ier;

	/* Disable PHY interrupts */
	ier = gmii->mdio_read(gmii->dev, gmii->phy_id, GMII_IER);
	ier &= ~IER_LINK_CHG;
	gmii->mdio_write(gmii->dev, gmii->phy_id, GMII_IER, ier);
}


struct efx_phy_operations alaska_phy_operations = {
	.init            = alaska_init,
	.fini            = alaska_fini,
	.reconfigure     = alaska_reconfigure,
	.clear_interrupt = alaska_clear_interrupt,
	.loopbacks       = (1 << LOOPBACK_PHY) | (1 << LOOPBACK_NETWORK),
	.startup_loopback = LOOPBACK_PHY,
};
