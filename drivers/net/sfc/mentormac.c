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
#include "gmii.h"
#include "mac.h"

/*
 * Mentor MAC control
 */

/**************************************************************************
 *
 * Mentor MAC registers
 *
 **************************************************************************
 *
 * Register addresses are Mentor MAC register numbers.  Falcon maps these
 * registers in at 16-byte intervals.  The mac_writel() and mac_readl()
 * methods take care of abstracting away this difference.
 */

/* GMAC configuration register 1 */
#define GM_CFG1_REG_MAC 0x00
#define GM_SW_RST_LBN 31
#define GM_SW_RST_WIDTH 1
#define GM_SIM_RST_LBN 30
#define GM_SIM_RST_WIDTH 1
#define GM_RST_RX_MAC_CTL_LBN 19
#define GM_RST_RX_MAC_CTL_WIDTH 1
#define GM_RST_TX_MAC_CTL_LBN 18
#define GM_RST_TX_MAC_CTL_WIDTH 1
#define GM_RST_RX_FUNC_LBN 17
#define GM_RST_RX_FUNC_WIDTH 1
#define GM_RST_TX_FUNC_LBN 16
#define GM_RST_TX_FUNC_WIDTH 1
#define GM_LOOP_LBN 8
#define GM_LOOP_WIDTH 1
#define GM_RX_FC_EN_LBN 5
#define GM_RX_FC_EN_WIDTH 1
#define GM_TX_FC_EN_LBN 4
#define GM_TX_FC_EN_WIDTH 1
#define GM_SYNC_RXEN_LBN 3
#define GM_SYNC_RXEN_WIDTH 1
#define GM_RX_EN_LBN 2
#define GM_RX_EN_WIDTH 1
#define GM_SYNC_TXEN_LBN 1
#define GM_SYNC_TXEN_WIDTH 1
#define GM_TX_EN_LBN 0
#define GM_TX_EN_WIDTH 1

/* GMAC configuration register 2 */
#define GM_CFG2_REG_MAC 0x01
#define GM_PAMBL_LEN_LBN 12
#define GM_PAMBL_LEN_WIDTH 4
#define GM_IF_MODE_LBN 8
#define GM_IF_MODE_WIDTH 2
#define GM_HUGE_FRM_EN_LBN 5
#define GM_HUGE_FRM_EN_WIDTH 1
#define GM_LEN_CHK_LBN 4
#define GM_LEN_CHK_WIDTH 1
#define GM_PAD_CRC_EN_LBN 2
#define GM_PAD_CRC_EN_WIDTH 1
#define GM_CRC_EN_LBN 1
#define GM_CRC_EN_WIDTH 1
#define GM_FD_LBN 0
#define GM_FD_WIDTH 1

/* GMAC maximum frame length register */
#define GM_MAX_FLEN_REG_MAC 0x04
#define GM_MAX_FLEN_LBN 0
#define GM_MAX_FLEN_WIDTH 16

/* GMAC MII management configuration register */
#define GM_MII_MGMT_CFG_REG_MAC 0x08
#define GM_RST_MII_MGMT_LBN 31
#define GM_RST_MII_MGMT_WIDTH 1
#define GM_MGMT_SCAN_AUTO_INC_LBN 5
#define GM_MGMT_SCAN_AUTO_INC_WIDTH 1
#define GM_MGMT_PREM_SUPR_LBN 4
#define GM_MGMT_PREM_SUPR_WIDTH 1
#define GM_MGMT_CLK_SEL_LBN 0
#define GM_MGMT_CLK_SEL_WIDTH 3

/* GMAC MII management command register */
#define GM_MII_MGMT_CMD_REG_MAC 0x09
#define GM_MGMT_SCAN_CYC_LBN 1
#define GM_MGMT_SCAN_CYC_WIDTH 1
#define GM_MGMT_RD_CYC_LBN 0
#define GM_MGMT_RD_CYC_WIDTH 1

/* GMAC MII management address register */
#define GM_MII_MGMT_ADR_REG_MAC 0x0a
#define GM_MGMT_PHY_ADDR_LBN 8
#define GM_MGMT_PHY_ADDR_WIDTH 5
#define GM_MGMT_REG_ADDR_LBN 0
#define GM_MGMT_REG_ADDR_WIDTH 5

/* GMAC MII management control register */
#define GM_MII_MGMT_CTL_REG_MAC 0x0b
#define GM_MGMT_CTL_LBN 0
#define GM_MGMT_CTL_WIDTH 16

/* GMAC MII management status register */
#define GM_MII_MGMT_STAT_REG_MAC 0x0c
#define GM_MGMT_STAT_LBN 0
#define GM_MGMT_STAT_WIDTH 16

/* GMAC MII management indicators register */
#define GM_MII_MGMT_IND_REG_MAC 0x0d
#define GM_MGMT_NOT_VLD_LBN 2
#define GM_MGMT_NOT_VLD_WIDTH 1
#define GM_MGMT_SCANNING_LBN 1
#define GM_MGMT_SCANNING_WIDTH 1
#define GM_MGMT_BUSY_LBN 0
#define GM_MGMT_BUSY_WIDTH 1

/* GMAC station address register 1 */
#define GM_ADR1_REG_MAC 0x10
#define GM_HWADDR_5_LBN 24
#define GM_HWADDR_5_WIDTH 8
#define GM_HWADDR_4_LBN 16
#define GM_HWADDR_4_WIDTH 8
#define GM_HWADDR_3_LBN 8
#define GM_HWADDR_3_WIDTH 8
#define GM_HWADDR_2_LBN 0
#define GM_HWADDR_2_WIDTH 8

/* GMAC station address register 2 */
#define GM_ADR2_REG_MAC 0x11
#define GM_HWADDR_1_LBN 24
#define GM_HWADDR_1_WIDTH 8
#define GM_HWADDR_0_LBN 16
#define GM_HWADDR_0_WIDTH 8

/* GMAC FIFO configuration register 0 */
#define GMF_CFG0_REG_MAC 0x12
#define GMF_FTFENRPLY_LBN 20
#define GMF_FTFENRPLY_WIDTH 1
#define GMF_STFENRPLY_LBN 19
#define GMF_STFENRPLY_WIDTH 1
#define GMF_FRFENRPLY_LBN 18
#define GMF_FRFENRPLY_WIDTH 1
#define GMF_SRFENRPLY_LBN 17
#define GMF_SRFENRPLY_WIDTH 1
#define GMF_WTMENRPLY_LBN 16
#define GMF_WTMENRPLY_WIDTH 1
#define GMF_FTFENREQ_LBN 12
#define GMF_FTFENREQ_WIDTH 1
#define GMF_STFENREQ_LBN 11
#define GMF_STFENREQ_WIDTH 1
#define GMF_FRFENREQ_LBN 10
#define GMF_FRFENREQ_WIDTH 1
#define GMF_SRFENREQ_LBN 9
#define GMF_SRFENREQ_WIDTH 1
#define GMF_WTMENREQ_LBN 8
#define GMF_WTMENREQ_WIDTH 1
#define GMF_HSTRSTFT_LBN 4
#define GMF_HSTRSTFT_WIDTH 1
#define GMF_HSTRSTST_LBN 3
#define GMF_HSTRSTST_WIDTH 1
#define GMF_HSTRSTFR_LBN 2
#define GMF_HSTRSTFR_WIDTH 1
#define GMF_HSTRSTSR_LBN 1
#define GMF_HSTRSTSR_WIDTH 1
#define GMF_HSTRSTWT_LBN 0
#define GMF_HSTRSTWT_WIDTH 1

/* GMAC FIFO configuration register 1 */
#define GMF_CFG1_REG_MAC 0x13
#define GMF_CFGFRTH_LBN 16
#define GMF_CFGFRTH_WIDTH 5
#define GMF_CFGXOFFRTX_LBN 0
#define GMF_CFGXOFFRTX_WIDTH 16

/* GMAC FIFO configuration register 2 */
#define GMF_CFG2_REG_MAC 0x14
#define GMF_CFGHWM_LBN 16
#define GMF_CFGHWM_WIDTH 6
#define GMF_CFGLWM_LBN 0
#define GMF_CFGLWM_WIDTH 6

/* GMAC FIFO configuration register 3 */
#define GMF_CFG3_REG_MAC 0x15
#define GMF_CFGHWMFT_LBN 16
#define GMF_CFGHWMFT_WIDTH 6
#define GMF_CFGFTTH_LBN 0
#define GMF_CFGFTTH_WIDTH 6

/* GMAC FIFO configuration register 4 */
#define GMF_CFG4_REG_MAC 0x16
#define GMF_HSTFLTRFRM_LBN 0
#define GMF_HSTFLTRFRM_WIDTH 18
#define GMF_HSTFLTRFRM_PAUSE_LBN 12
#define GMF_HSTFLTRFRM_PAUSE_WIDTH 12

/* GMAC FIFO configuration register 5 */
#define GMF_CFG5_REG_MAC 0x17
#define GMF_CFGHDPLX_LBN 22
#define GMF_CFGHDPLX_WIDTH 1
#define GMF_SRFULL_LBN 21
#define GMF_SRFULL_WIDTH 1
#define GMF_HSTSRFULLCLR_LBN 20
#define GMF_HSTSRFULLCLR_WIDTH 1
#define GMF_CFGBYTMODE_LBN 19
#define GMF_CFGBYTMODE_WIDTH 1
#define GMF_HSTDRPLT64_LBN 18
#define GMF_HSTDRPLT64_WIDTH 1
#define GMF_HSTFLTRFRMDC_LBN 0
#define GMF_HSTFLTRFRMDC_WIDTH 18
#define GMF_HSTFLTRFRMDC_PAUSE_LBN 12
#define GMF_HSTFLTRFRMDC_PAUSE_WIDTH 1

/* TX total octet count */
#define GM_TX_OCT_CNT_REG_MAC 0x40
#define GM_STAT_LBN 0
#define GM_STAT_WIDTH 32

/* TX good octet count */
#define GM_TX_GOOD_OCT_CNT_REG_MAC 0x41

/* TX single collision packet count */
#define GM_TX_SGLCOL_PKT_CNT_REG_MAC 0x42

/* TX multiple collision packet count */
#define GM_TX_MULTCOL_PKT_CNT_REG_MAC 0x43

/* TX excessive collision packet count */
#define GM_TX_EXCOL_PKT_CNT_REG_MAC 0x44

/* TX deferred packet count */
#define GM_TX_DEF_PKT_CNT_REG_MAC 0x45

/* TX late packet count */
#define GM_TX_LATECOL_PKT_CNT_REG_MAC 0x46

/* TX excessive deferral packet count */
#define GM_TX_EXDEF_PKT_CNT_REG_MAC 0x47

/* TX pause packet count */
#define GM_TX_PAUSE_PKT_CNT_REG_MAC 0x48

/* TX bad packet count */
#define GM_TX_BAD_PKT_CNT_REG_MAC 0x49

/* TX unicast packet count */
#define GM_TX_UCAST_PKT_CNT_REG_MAC 0x4a

/* TX multicast packet count */
#define GM_TX_MCAST_PKT_CNT_REG_MAC 0x4b

/* TX broadcast packet count */
#define GM_TX_BCAST_PKT_CNT_REG_MAC 0x4c

/* TX <64-byte packet count */
#define GM_TX_LT64_PKT_CNT_REG_MAC 0x4d

/* TX 64-byte packet count */
#define GM_TX_64_PKT_CNT_REG_MAC 0x4e

/* TX 65-byte to 127-byte packet count */
#define GM_TX_65_TO_127_PKT_CNT_REG_MAC 0x4f

/* TX 128-byte to 255-byte packet count */
#define GM_TX_128_TO_255_PKT_CNT_REG_MAC 0x50

/* TX 256-byte to 511-byte packet count */
#define GM_TX_256_TO_511_PKT_CNT_REG_MAC 0x51

/* TX 512-byte to 1023-byte packet count */
#define GM_TX_512_TO_1023_PKT_CNT_REG_MAC 0x52

/* TX 1024-byte to 15xx-byte packet count */
#define GM_TX_1024_TO_15XX_PKT_CNT_REG_MAC 0x53

/* TX 15xx-byte to jumbo packet count */
#define GM_TX_15XX_TO_JUMBO_PKT_CNT_REG_MAC 0x54

/* TX >jumbo packet count */
#define GM_TX_GTJUMBO_PKT_CNT_REG_MAC 0x55

/* RX good octet count */
#define GM_RX_GOOD_OCT_CNT_REG_MAC 0x60

/* RX bad octet count */
#define GM_RX_BAD_OCT_CNT_REG_MAC 0x61

/* RX missed packet count */
#define GM_RX_MISS_PKT_CNT_REG_MAC 0x62

/* RX false carrier count */
#define GM_RX_FALSE_CRS_CNT_REG_MAC 0x63

/* RX pause packet count */
#define GM_RX_PAUSE_PKT_CNT_REG_MAC 0x64

/* RX bad packet count */
#define GM_RX_BAD_PKT_CNT_REG_MAC 0x65

/* RX unicast packet count */
#define GM_RX_UCAST_PKT_CNT_REG_MAC 0x66

/* RX multicast packet count */
#define GM_RX_MCAST_PKT_CNT_REG_MAC 0x67

/* RX broadcast packet count */
#define GM_RX_BCAST_PKT_CNT_REG_MAC 0x68

/* RX <64-byte good packet count */
#define GM_RX_GOOD_LT64_PKT_CNT_REG_MAC 0x69

/* RX <64-byte bad packet count */
#define GM_RX_BAD_LT64_PKT_CNT_REG_MAC 0x6a

/* RX 64-byte packet count */
#define GM_RX_64_PKT_CNT_REG_MAC 0x6b

/* RX 65-byte to 127-byte packet count */
#define GM_RX_65_TO_127_PKT_CNT_REG_MAC 0x6c

/* RX 128-byte to 255-byte packet count*/
#define GM_RX_128_TO_255_PKT_CNT_REG_MAC 0x6d

/* RX 256-byte to 511-byte packet count */
#define GM_RX_256_TO_511_PKT_CNT_REG_MAC 0x6e

/* RX 512-byte to 1023-byte packet count */
#define GM_RX_512_TO_1023_PKT_CNT_REG_MAC 0x6f

/* RX 1024-byte to 15xx-byte packet count */
#define GM_RX_1024_TO_15XX_PKT_CNT_REG_MAC 0x70

/* RX 15xx-byte to jumbo packet count */
#define GM_RX_15XX_TO_JUMBO_PKT_CNT_REG_MAC 0x71

/* RX >jumbo packet count */
#define GM_RX_GTJUMBO_PKT_CNT_REG_MAC 0x72

/* RX 64-byte to 15xx-byte bad crc packet count */
#define GM_RX_BAD_64_TO_15XX_PKT_CNT_REG_MAC 0x73

/* RX 15xx-byte to jumbo bad crc packet count */
#define GM_RX_BAD_15XX_TO_JUMBO_PKT_CNT_REG_MAC 0x74

/* RX >jumbo bad crc packet count */
#define GM_RX_BAD_GTJUMBO_PKT_CNT_REG_MAC 0x75

/**************************************************************************
 *
 * GMII access to PHY
 *
 **************************************************************************
 */

/* This does not reset the PHY, only the MAC.  However, TX and RX will
 * both be disabled on the MAC after this, so the state of the PHY is
 * somewhat irrelevant until the MAC is reinitialised.
 */
void mentormac_reset(struct efx_nic *efx)
{
	efx_dword_t reg;

	EFX_POPULATE_DWORD_1(reg, GM_SW_RST, 1);
	efx->mac_op->mac_writel(efx, &reg, GM_CFG1_REG_MAC);
	udelay(1000);

	EFX_POPULATE_DWORD_1(reg, GM_SW_RST, 0);
	efx->mac_op->mac_writel(efx, &reg, GM_CFG1_REG_MAC);
	udelay(1000);

	/* Configure GMII interface so PHY is accessible */
	EFX_POPULATE_DWORD_1(reg, GM_MGMT_CLK_SEL, 0x4);
	efx->mac_op->mac_writel(efx, &reg,
				     GM_MII_MGMT_CFG_REG_MAC);
	udelay(10);
}

void mentormac_reconfigure(struct efx_nic *efx)
{
	int loopback, tx_fc, rx_fc, if_mode, full_duplex, bytemode, half_duplex;
	unsigned int max_frame_len;
	efx_dword_t reg;

	/* Configuration register 1 */
	tx_fc = (efx->flow_control & EFX_FC_TX) ? 1 : 0;
	rx_fc = (efx->flow_control & EFX_FC_RX) ? 1 : 0;
	loopback = (efx->loopback_mode == LOOPBACK_MAC) ? 1 : 0;
	bytemode = (efx->link_options & GM_LPA_1000) ? 1 : 0;

	if (efx->loopback_mode != LOOPBACK_NONE)
		bytemode = 1;
	if (!(efx->link_options & GM_LPA_DUPLEX))
		/* Half-duplex operation requires TX flow control */
		tx_fc = 1;
	EFX_POPULATE_DWORD_5(reg,
			     GM_LOOP, loopback,
			     GM_TX_EN, 1,
			     GM_TX_FC_EN, tx_fc,
			     GM_RX_EN, 1,
			     GM_RX_FC_EN, rx_fc);
	efx->mac_op->mac_writel(efx, &reg, GM_CFG1_REG_MAC);
	udelay(10);

	/* Configuration register 2 */
	if_mode = (bytemode) ? 2 : 1;
	full_duplex = (efx->link_options & GM_LPA_DUPLEX) ? 1 : 0;
	EFX_POPULATE_DWORD_4(reg,
			     GM_IF_MODE, if_mode,
			     GM_PAD_CRC_EN, 1,
			     GM_FD, full_duplex,
			     GM_PAMBL_LEN, 0x7/*datasheet recommended */);

	efx->mac_op->mac_writel(efx, &reg, GM_CFG2_REG_MAC);
	udelay(10);

	/* Max frame len register */
	max_frame_len = EFX_MAX_FRAME_LEN(efx->net_dev->mtu);
	EFX_POPULATE_DWORD_1(reg, GM_MAX_FLEN, max_frame_len);
	efx->mac_op->mac_writel(efx, &reg, GM_MAX_FLEN_REG_MAC);
	udelay(10);

	/* FIFO configuration register 0 */
	EFX_POPULATE_DWORD_5(reg,
			     GMF_FTFENREQ, 1,
			     GMF_STFENREQ, 1,
			     GMF_FRFENREQ, 1,
			     GMF_SRFENREQ, 1,
			     GMF_WTMENREQ, 1);
	efx->mac_op->mac_writel(efx, &reg, GMF_CFG0_REG_MAC);
	udelay(10);

	/* FIFO configuration register 1 */
	EFX_POPULATE_DWORD_2(reg,
			     GMF_CFGFRTH, 0x12,
			     GMF_CFGXOFFRTX, 0xffff);
	efx->mac_op->mac_writel(efx, &reg, GMF_CFG1_REG_MAC);
	udelay(10);

	/* FIFO configuration register 2 */
	EFX_POPULATE_DWORD_2(reg,
			     GMF_CFGHWM, 0x3f,
			     GMF_CFGLWM, 0xa);
	efx->mac_op->mac_writel(efx, &reg, GMF_CFG2_REG_MAC);
	udelay(10);

	/* FIFO configuration register 3 */
	EFX_POPULATE_DWORD_2(reg,
			     GMF_CFGHWMFT, 0x1c,
			     GMF_CFGFTTH, 0x08);
	efx->mac_op->mac_writel(efx, &reg, GMF_CFG3_REG_MAC);
	udelay(10);

	/* FIFO configuration register 4 */
	EFX_POPULATE_DWORD_1(reg, GMF_HSTFLTRFRM_PAUSE, 1);
	efx->mac_op->mac_writel(efx, &reg, GMF_CFG4_REG_MAC);
	udelay(10);

	/* FIFO configuration register 5 */
	half_duplex = (efx->link_options & GM_LPA_DUPLEX) ? 0 : 1;
	efx->mac_op->mac_readl(efx, &reg, GMF_CFG5_REG_MAC);
	EFX_SET_DWORD_FIELD(reg, GMF_CFGBYTMODE, bytemode);
	EFX_SET_DWORD_FIELD(reg, GMF_CFGHDPLX, half_duplex);
	EFX_SET_DWORD_FIELD(reg, GMF_HSTDRPLT64, half_duplex);
	EFX_SET_DWORD_FIELD(reg, GMF_HSTFLTRFRMDC_PAUSE, 0);
	efx->mac_op->mac_writel(efx, &reg, GMF_CFG5_REG_MAC);
	udelay(10);

	/* MAC address */
	EFX_POPULATE_DWORD_4(reg,
			     GM_HWADDR_5, efx->net_dev->dev_addr[5],
			     GM_HWADDR_4, efx->net_dev->dev_addr[4],
			     GM_HWADDR_3, efx->net_dev->dev_addr[3],
			     GM_HWADDR_2, efx->net_dev->dev_addr[2]);
	efx->mac_op->mac_writel(efx, &reg, GM_ADR1_REG_MAC);
	udelay(10);
	EFX_POPULATE_DWORD_2(reg,
			     GM_HWADDR_1, efx->net_dev->dev_addr[1],
			     GM_HWADDR_0, efx->net_dev->dev_addr[0]);
	efx->mac_op->mac_writel(efx, &reg, GM_ADR2_REG_MAC);
	udelay(10);
}
