/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005-2006: Fen Systems Ltd.
 * Copyright 2006-2008: Solarflare Communications Inc,
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

#include <linux/delay.h>
#include "net_driver.h"
#include "efx.h"
#include "falcon.h"
#include "mac.h"
#include "falcon_hwdefs.h"
#include "falcon_io.h"
#include "gmii.h"

/**************************************************************************
 *
 * MAC register access
 *
 **************************************************************************/

/* Offset of a GMAC register within Falcon */
#define FALCON_GMAC_REG(mac_reg)					\
	(FALCON_GMAC_REGBANK + ((mac_reg) * FALCON_GMAC_REG_SIZE))

static void falcon_gmac_writel(struct efx_nic *efx,
			       efx_dword_t *value, unsigned int mac_reg)
{
	efx_oword_t temp;

	EFX_POPULATE_OWORD_1(temp, MAC_DATA, EFX_DWORD_FIELD(*value, MAC_DATA));
	falcon_write(efx, &temp, FALCON_GMAC_REG(mac_reg));
}

static void falcon_gmac_readl(struct efx_nic *efx,
			      efx_dword_t *value, unsigned int mac_reg)
{
	efx_oword_t temp;

	falcon_read(efx, &temp, FALCON_GMAC_REG(mac_reg));
	EFX_POPULATE_DWORD_1(*value, MAC_DATA, EFX_OWORD_FIELD(temp, MAC_DATA));
}

/**************************************************************************
 *
 * MAC operations
 *
 *************************************************************************/

static int falcon_init_gmac(struct efx_nic *efx)
{
	int rc;

	/* Reset the MAC */
	mentormac_reset(efx);

	/* Initialise PHY */
	rc = efx->phy_op->init(efx);
	if (rc)
		return rc;

	return 0;
}

static void falcon_reconfigure_gmac(struct efx_nic *efx)
{
	/* Reconfigure PHY and pick up PHY parameters.  This updates
	 * the link status. */
	efx->phy_op->reconfigure(efx);

	/* Isolate the MAC. */
	falcon_deconfigure_mac_wrapper(efx);

	/* Reconfigure MAC */
	mentormac_reconfigure(efx);

	/* Reconfigure MAC wrapper */
	falcon_reconfigure_mac_wrapper(efx);
}

static void falcon_fini_gmac(struct efx_nic *efx)
{
	/* Isolate the MAC - PHY */
	falcon_deconfigure_mac_wrapper(efx);

	/* Shut down PHY */
	efx->phy_op->fini(efx);

	/* Reset MAC */
	mentormac_reset(efx);
}

static void falcon_update_stats_gmac(struct efx_nic *efx)
{
	struct efx_mac_stats *mac_stats = &efx->mac_stats;
	unsigned long old_rx_pause, old_tx_pause;
	unsigned long new_rx_pause, new_tx_pause;
	int rc;

	rc = falcon_dma_stats(efx, GDmaDone_offset);
	if (rc)
		return;

	/* Pause frames are erroneously counted as errors (SFC bug 3269) */
	old_rx_pause = mac_stats->rx_pause;
	old_tx_pause = mac_stats->tx_pause;

	/* Update MAC stats from DMAed values */
	FALCON_STAT(efx, GRxGoodOct, rx_good_bytes);
	FALCON_STAT(efx, GRxBadOct, rx_bad_bytes);
	FALCON_STAT(efx, GRxMissPkt, rx_missed);
	FALCON_STAT(efx, GRxFalseCRS, rx_false_carrier);
	FALCON_STAT(efx, GRxPausePkt, rx_pause);
	FALCON_STAT(efx, GRxBadPkt, rx_bad);
	FALCON_STAT(efx, GRxUcastPkt, rx_unicast);
	FALCON_STAT(efx, GRxMcastPkt, rx_multicast);
	FALCON_STAT(efx, GRxBcastPkt, rx_broadcast);
	FALCON_STAT(efx, GRxGoodLt64Pkt, rx_good_lt64);
	FALCON_STAT(efx, GRxBadLt64Pkt, rx_bad_lt64);
	FALCON_STAT(efx, GRx64Pkt, rx_64);
	FALCON_STAT(efx, GRx65to127Pkt, rx_65_to_127);
	FALCON_STAT(efx, GRx128to255Pkt, rx_128_to_255);
	FALCON_STAT(efx, GRx256to511Pkt, rx_256_to_511);
	FALCON_STAT(efx, GRx512to1023Pkt, rx_512_to_1023);
	FALCON_STAT(efx, GRx1024to15xxPkt, rx_1024_to_15xx);
	FALCON_STAT(efx, GRx15xxtoJumboPkt, rx_15xx_to_jumbo);
	FALCON_STAT(efx, GRxGtJumboPkt, rx_gtjumbo);
	FALCON_STAT(efx, GRxFcsErr64to15xxPkt, rx_bad_64_to_15xx);
	FALCON_STAT(efx, GRxFcsErr15xxtoJumboPkt, rx_bad_15xx_to_jumbo);
	FALCON_STAT(efx, GRxFcsErrGtJumboPkt, rx_bad_gtjumbo);
	FALCON_STAT(efx, GTxGoodBadOct, tx_bytes);
	FALCON_STAT(efx, GTxGoodOct, tx_good_bytes);
	FALCON_STAT(efx, GTxSglColPkt, tx_single_collision);
	FALCON_STAT(efx, GTxMultColPkt, tx_multiple_collision);
	FALCON_STAT(efx, GTxExColPkt, tx_excessive_collision);
	FALCON_STAT(efx, GTxDefPkt, tx_deferred);
	FALCON_STAT(efx, GTxLateCol, tx_late_collision);
	FALCON_STAT(efx, GTxExDefPkt, tx_excessive_deferred);
	FALCON_STAT(efx, GTxPausePkt, tx_pause);
	FALCON_STAT(efx, GTxBadPkt, tx_bad);
	FALCON_STAT(efx, GTxUcastPkt, tx_unicast);
	FALCON_STAT(efx, GTxMcastPkt, tx_multicast);
	FALCON_STAT(efx, GTxBcastPkt, tx_broadcast);
	FALCON_STAT(efx, GTxLt64Pkt, tx_lt64);
	FALCON_STAT(efx, GTx64Pkt, tx_64);
	FALCON_STAT(efx, GTx65to127Pkt, tx_65_to_127);
	FALCON_STAT(efx, GTx128to255Pkt, tx_128_to_255);
	FALCON_STAT(efx, GTx256to511Pkt, tx_256_to_511);
	FALCON_STAT(efx, GTx512to1023Pkt, tx_512_to_1023);
	FALCON_STAT(efx, GTx1024to15xxPkt, tx_1024_to_15xx);
	FALCON_STAT(efx, GTx15xxtoJumboPkt, tx_15xx_to_jumbo);
	FALCON_STAT(efx, GTxGtJumboPkt, tx_gtjumbo);
	FALCON_STAT(efx, GTxNonTcpUdpPkt, tx_non_tcpudp);
	FALCON_STAT(efx, GTxMacSrcErrPkt, tx_mac_src_error);
	FALCON_STAT(efx, GTxIpSrcErrPkt, tx_ip_src_error);

	/* Pause frames are erroneously counted as errors (SFC bug 3269) */
	new_rx_pause = mac_stats->rx_pause;
	new_tx_pause = mac_stats->tx_pause;
	mac_stats->rx_bad -= (new_rx_pause - old_rx_pause);
	mac_stats->tx_bad -= (new_tx_pause - old_tx_pause);

	/* Derive stats that the MAC doesn't provide directly */
	mac_stats->tx_bad_bytes =
		mac_stats->tx_bytes - mac_stats->tx_good_bytes;
	mac_stats->tx_packets =
		mac_stats->tx_lt64 + mac_stats->tx_64 +
		mac_stats->tx_65_to_127 + mac_stats->tx_128_to_255 +
		mac_stats->tx_256_to_511 + mac_stats->tx_512_to_1023 +
		mac_stats->tx_1024_to_15xx + mac_stats->tx_15xx_to_jumbo +
		mac_stats->tx_gtjumbo;
	mac_stats->tx_collision =
		mac_stats->tx_single_collision +
		mac_stats->tx_multiple_collision +
		mac_stats->tx_excessive_collision +
		mac_stats->tx_late_collision;
	mac_stats->rx_bytes =
		mac_stats->rx_good_bytes + mac_stats->rx_bad_bytes;
	mac_stats->rx_packets =
		mac_stats->rx_good_lt64 + mac_stats->rx_bad_lt64 +
		mac_stats->rx_64 + mac_stats->rx_65_to_127 +
		mac_stats->rx_128_to_255 + mac_stats->rx_256_to_511 +
		mac_stats->rx_512_to_1023 + mac_stats->rx_1024_to_15xx +
		mac_stats->rx_15xx_to_jumbo + mac_stats->rx_gtjumbo;
	mac_stats->rx_good = mac_stats->rx_packets - mac_stats->rx_bad;
	mac_stats->rx_lt64 = mac_stats->rx_good_lt64 + mac_stats->rx_bad_lt64;
}

static int falcon_check_gmac(struct efx_nic *efx)
{
	/* Nothing to do */
	return 0;
}

static void falcon_gmac_sim_phy_event(struct efx_nic *efx)
{
	efx_qword_t phy_event;

	EFX_POPULATE_QWORD_2(phy_event,
			     EV_CODE, GLOBAL_EV_DECODE,
			     G_PHY0_INTR, 1);
	falcon_generate_event(&efx->channel[0], &phy_event);
}

static void falcon_gmac_reset_phy(struct efx_nic *efx)
{
	struct mii_if_info *gmii = &efx->mii;
	int bmcr, i;

	/* Perform software reset to make new settings take effect */
	bmcr = gmii->mdio_read(gmii->dev, gmii->phy_id, MII_BMCR);
	bmcr |= BMCR_RESET;
	gmii->mdio_write(gmii->dev, gmii->phy_id, MII_BMCR, bmcr);

	/* Wait for the reset to deassert */
	for (i = 20; i; --i) {
		udelay(10);
		if ((gmii->mdio_read(gmii->dev, gmii->phy_id, MII_BMCR) &
		    BMCR_RESET) == 0)
			return;
	}

	EFX_ERR(efx, "wait for PHY reset timed out\n");
}


static int falcon_gmac_get_settings(struct efx_nic *efx,
				    struct ethtool_cmd *ecmd)
{
	struct mii_if_info *gmii = &efx->mii;
	int rc;

	rc = mii_ethtool_gset(gmii, ecmd);
	ecmd->supported &= ~(SUPPORTED_1000baseT_Half);
	return rc;
}

static int falcon_gmac_set_settings(struct efx_nic *efx,
				    struct ethtool_cmd *ecmd)
{
	struct mii_if_info *gmii = &efx->mii;
	int rc;

	/* 1000Mbps half-duplex is technically legal, but none of our
	 * current hardware supports it, so just disallow it. */
	if (ecmd->speed == SPEED_1000 && ecmd->duplex != DUPLEX_FULL) {
		EFX_LOG(efx, "rejecting unsupported 1000Mbps HD"
			" setting\n");
		return -EINVAL;
	}

	/* Use MII to set all other settings */
	rc = mii_ethtool_sset(gmii, ecmd);
	if (rc)
		return rc;

	/* Reset the PHY */
	falcon_gmac_reset_phy(efx);

	return 0;
}

static int falcon_gmac_set_pause(struct efx_nic *efx,
				 enum efx_fc_type flow_control)
{
	struct mii_if_info *gmii = &efx->mii;
	int adv;

	/* GMAC has tiny MAC FIFO, so TX flow control won't work */
	if (flow_control & EFX_FC_TX)
		return -EINVAL;

	efx->flow_control = flow_control;

	/* Push autonegotiation to PHY */
	adv = gmii->mdio_read(gmii->dev, gmii->phy_id, MII_ADVERTISE);
	adv &= ~GM_ADVERTISE_PAUSE_CAP;
	adv |= (flow_control & EFX_FC_AUTO) ? GM_ADVERTISE_PAUSE_CAP : 0;
	gmii->mdio_write(gmii->dev, gmii->phy_id, MII_ADVERTISE, adv);

	falcon_gmac_reset_phy(efx);

	return 0;
}


struct efx_mac_operations falcon_gmac_operations = {
	.mac_writel	= falcon_gmac_writel,
	.mac_readl	= falcon_gmac_readl,
	.init		= falcon_init_gmac,
	.reconfigure	= falcon_reconfigure_gmac,
	.update_stats	= falcon_update_stats_gmac,
	.fini		= falcon_fini_gmac,
	.check_hw	= falcon_check_gmac,
	.fake_phy_event	= falcon_gmac_sim_phy_event,
	.get_settings   = falcon_gmac_get_settings,
	.set_settings   = falcon_gmac_set_settings,
	.set_pause      = falcon_gmac_set_pause,
};
