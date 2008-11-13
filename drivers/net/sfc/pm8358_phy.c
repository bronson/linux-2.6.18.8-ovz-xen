/* Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2006-2008: Solarflare Communications Inc,
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
/*
 *  Driver for PMC-Sierra PM8358 XAUI PHY
 */

#include <linux/delay.h>
#include "efx.h"
#include "gmii.h"
#include "mdio_10g.h"
#include "phy.h"

#define PM8358_REQUIRED_DEVS (MDIO_MMDREG_DEVS0_DTEXS)
#define PM8358_LOOPBACKS (1 << LOOPBACK_PHY)

/* PHY-specific definitions */
/* Master ID and Global Performance Monitor Update */
#define PMC_MASTER_REG (0xd000)
/* Analog TX/RX settings under software control */
#define PMC_MASTER_ANLG_CTRL (1 << 11)

#define PMC_MCONF2_REG	(0xd002)
/* Drive TX off centre of data eye (1) vs. clock edge (0) */
#define	PMC_MCONF2_TEDGE (1 << 2)
/* Drive RX off centre of data eye (1) vs. clock edge (0) */
#define PMC_MCONF2_REDGE (1 << 3)

/* Analog RX settings */
#define PMC_ANALOG_RX_CFG0   (0xd025)
#define PMC_ANALOG_RX_CFG1   (0xd02d)
#define PMC_ANALOG_RX_CFG2   (0xd035)
#define PMC_ANALOG_RX_CFG3   (0xd03d)
#define PMC_ANALOG_RX_TERM     (1 << 15) /* Bit 15 of RX CFG: 0 for 100 ohms
					    float, 1 for 50 to 1.2V */
#define PMC_ANALOG_RX_EQ_MASK (3 << 8)
#define PMC_ANALOG_RX_EQ_NONE (0 << 8)
#define PMC_ANALOG_RX_EQ_HALF (1 << 8)
#define PMC_ANALOG_RX_EQ_FULL (2 << 8)
#define PMC_ANALOG_RX_EQ_RSVD (3 << 8)

/* Reset the DTE XS MMD. */
#define PMC_MAX_RESET_TIME 500
#define PMC_RESET_WAIT 10

static int pmc_reset_phy(struct efx_nic *efx)
{
	int rc = mdio_clause45_reset_mmd(efx, MDIO_MMD_DTEXS,
					 PMC_MAX_RESET_TIME / PMC_RESET_WAIT,
					 PMC_RESET_WAIT);
	if (rc >= 0) {
		EFX_TRACE(efx, "PMC8358: came out of reset with "
			  "%d0 ms left\n", rc);
		rc = 0;
	} else {
		EFX_ERR(efx, "PMC8358: reset timed out!\n");
	}
	return rc;
}


static void pmc_full_rx_eq(struct efx_nic *efx)
{
	int i, reg;

	/* Enable software control of analog settings */
	reg = mdio_clause45_read(efx, efx->mii.phy_id,
				 MDIO_MMD_DTEXS, PMC_MASTER_REG);
	reg |= PMC_MASTER_ANLG_CTRL;

	mdio_clause45_write(efx, efx->mii.phy_id,
			    MDIO_MMD_DTEXS, PMC_MASTER_REG, reg);

	/* Turn RX eq on full for all channels. */
	for (i = 0; i < 3; i++) {
		/* The analog CFG registers are evenly spaced 8 apart */
		u16 addr = PMC_ANALOG_RX_CFG0 + 8 * i;

		reg = mdio_clause45_read(efx, efx->mii.phy_id,
					 MDIO_MMD_DTEXS, addr);
		reg = (reg & ~PMC_ANALOG_RX_EQ_MASK) | PMC_ANALOG_RX_EQ_FULL;
		mdio_clause45_write(efx, efx->mii.phy_id,
				    MDIO_MMD_DTEXS, addr, reg);
	}
}

static void pmc_set_data_edges(struct efx_nic *efx)
{
	int reg;
	/* Set TEDGE, clear REDGE */
	reg = mdio_clause45_read(efx, efx->mii.phy_id,
				 MDIO_MMD_DTEXS, PMC_MCONF2_REG);
	reg &= ~PMC_MCONF2_REDGE;
	reg |= PMC_MCONF2_TEDGE;

	mdio_clause45_write(efx, efx->mii.phy_id,
			    MDIO_MMD_DTEXS, PMC_MCONF2_REG, reg);
}

static int pm8358_phy_init(struct efx_nic *efx)
{
	u32 devid;
	int rc;

	/* The GLB_CTL reset line has been pulled before this is called,
	 * and it may take up to 5ms for the PLL's to synchronise after
	 * this is done. Best to wait 10ms here */
	schedule_timeout_uninterruptible(HZ / 100);

	rc = pmc_reset_phy(efx);
	if (rc < 0)
		return rc;

	/* Check that all the MMDs we expect are present and responding. We
	 * expect faults on some if the link is down, but not on the PHY XS */
	rc = mdio_clause45_check_mmds(efx, PM8358_REQUIRED_DEVS, 0);
	if (rc < 0)
		return rc;

	devid = mdio_clause45_read_id(efx, MDIO_MMD_DTEXS);
	EFX_LOG(efx, "PM8358: PHY ID reg %x (OUI %x model %x revision"
		" %x)\n", devid, MDIO_ID_OUI(devid), MDIO_ID_MODEL(devid),
		MDIO_ID_REV(devid));

	/* Turn on full RX equalisation */
	pmc_full_rx_eq(efx);

	/* Adjust RX and TX data edge position */
	pmc_set_data_edges(efx);

	EFX_LOG(efx, "PM8358: PHY init successful.\n");
	return rc;
}

static int pm8358_link_ok(struct efx_nic *efx)
{
	return mdio_clause45_links_ok(efx, PM8358_REQUIRED_DEVS);
}

static int pm8358_phy_check_hw(struct efx_nic *efx)
{
	int rc = 0;
	int link_up = pm8358_link_ok(efx);
	/* Simulate a PHY event if link state has changed */
	if (link_up != efx->link_up) {
		efx->link_up = link_up;
		efx->mac_op->fake_phy_event(efx);
	}

	return rc;
}

static void pm8358_phy_reconfigure(struct efx_nic *efx)
{
	int phy_id = efx->mii.phy_id;
	int ctrl;
	/* Handle DTE loopback */
	ctrl = mdio_clause45_read(efx, phy_id, MDIO_MMD_DTEXS,
				  MDIO_MMDREG_CTRL1);
	if (efx->loopback_mode == LOOPBACK_PHY) {
		EFX_TRACE(efx, "PM8358: setting DTE loopback\n");
		ctrl |= (1 << MDIO_MMDREG_CTRL1_LBACK_LBN);
	} else {
		if (ctrl & (1 << MDIO_MMDREG_CTRL1_LBACK_LBN))
			EFX_TRACE(efx,
				  "PM8358: clearing DTE loopback\n");
		ctrl &= ~(1 << MDIO_MMDREG_CTRL1_LBACK_LBN);
	}
	mdio_clause45_write(efx, phy_id, MDIO_MMD_DTEXS,
			    MDIO_MMDREG_CTRL1, ctrl);

	efx->link_up = pm8358_link_ok(efx);
	efx->link_options = GM_LPA_10000FULL;
}

struct efx_phy_operations falcon_pm8358_phy_ops = {
	.init            = pm8358_phy_init,
	.reconfigure     = pm8358_phy_reconfigure,
	.check_hw        = pm8358_phy_check_hw,
	.fini            = efx_port_dummy_op_void,
	.clear_interrupt = efx_port_dummy_op_void,
	.reset_xaui      = efx_port_dummy_op_void,
	.mmds            = PM8358_REQUIRED_DEVS,
	.loopbacks       = PM8358_LOOPBACKS,
	.startup_loopback = LOOPBACK_PHY,
};
