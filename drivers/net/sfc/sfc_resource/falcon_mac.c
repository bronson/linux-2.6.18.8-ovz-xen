/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains MACs (Mentor MAC & GDACT1 ) support for Falcon.
 *
 * Copyright 2005-2007: Solarflare Communications Inc,
 *                      9501 Jeronimo Road, Suite 250,
 *                      Irvine, CA 92618, USA
 *
 * Developed and maintained by Solarflare Communications:
 *                      <linux-xen-drivers@solarflare.com>
 *                      <onload-dev@solarflare.com>
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

#include <ci/efhw/falcon.h>
#include <ci/driver/efab/hardware.h>

/********************************************************************
 * Mentor MAC
 */

#define _PRE(x)       GM##x

/*--------------------------------------------------------------------
 *
 * Debug Support
 *
 *--------------------------------------------------------------------*/

#define MENTOR_MAC_ASSERT_VALID()					\
    EFHW_ASSERT(nic);							\
    EFHW_ASSERT(EFHW_KVA(nic));						\
    EFHW_ASSERT(_PRE(_CFG1_REG_OFST) == _PRE(_CFG1_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(_CFG2_REG_OFST) == _PRE(_CFG2_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(_IPG_REG_OFST) == _PRE(_IPG_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(_HD_REG_OFST) == _PRE(_HD_REG_KER_OFST));		\
    EFHW_ASSERT(_PRE(_MAX_FLEN_REG_OFST) == _PRE(_MAX_FLEN_REG_KER_OFST)); \
    EFHW_ASSERT(_PRE(_TEST_REG_OFST) == _PRE(_TEST_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(_ADR1_REG_OFST) == _PRE(_ADR1_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(_ADR2_REG_OFST) == _PRE(_ADR2_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(F_CFG0_REG_OFST) == _PRE(F_CFG0_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(F_CFG1_REG_OFST) == _PRE(F_CFG1_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(F_CFG2_REG_OFST) == _PRE(F_CFG2_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(F_CFG3_REG_OFST) == _PRE(F_CFG3_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(F_CFG4_REG_OFST) == _PRE(F_CFG4_REG_KER_OFST));	\
    EFHW_ASSERT(_PRE(F_CFG5_REG_OFST) == _PRE(F_CFG5_REG_KER_OFST));

/*! Get MAC current address - i.e not necessarily the one in the EEPROM */
static inline void mentormac_get_mac_addr(struct efhw_nic *nic)
{
	efhw_ioaddr_t mac_kva;
	uint val1, val2;

	MENTOR_MAC_ASSERT_VALID();

	mac_kva = GM_P0_BASE + EFHW_KVA(nic);

	val1 = readl(mac_kva + _PRE(_ADR1_REG_OFST));
	val2 = readl(mac_kva + _PRE(_ADR2_REG_OFST));

#if 0
	nic->mac_addr[0] = (val1 & 0xff000000) >> 24;
	nic->mac_addr[1] = (val1 & 0x00ff0000) >> 16;
	nic->mac_addr[2] = (val1 & 0x0000ff00) >> 8;
	nic->mac_addr[3] = (val1 & 0x000000ff) >> 0;
	nic->mac_addr[4] = (val2 & 0xff000000) >> 24;
	nic->mac_addr[5] = (val2 & 0x00ff0000) >> 16;
#else
	nic->mac_addr[5] = (val1 & 0xff000000) >> 24;
	nic->mac_addr[4] = (val1 & 0x00ff0000) >> 16;
	nic->mac_addr[3] = (val1 & 0x0000ff00) >> 8;
	nic->mac_addr[2] = (val1 & 0x000000ff) >> 0;
	nic->mac_addr[1] = (val2 & 0xff000000) >> 24;
	nic->mac_addr[0] = (val2 & 0x00ff0000) >> 16;
#endif
}


/********************************************************************
 * GDACT10 MAC
 */

/*--------------------------------------------------------------------
 *
 * Debug Support
 *
 *--------------------------------------------------------------------*/

#define GDACT10_MAC_ASSERT_VALID()					\
    EFHW_ASSERT(nic);							\
    EFHW_ASSERT(EFHW_KVA(nic));						\
    EFHW_ASSERT(XM_GLB_CFG_REG_P0_OFST == XM_GLB_CFG_REG_KER_P0_OFST);	\
    EFHW_ASSERT(XM_TX_CFG_REG_P0_OFST  == XM_TX_CFG_REG_KER_P0_OFST);	\
    EFHW_ASSERT(XM_RX_CFG_REG_P0_OFST  == XM_RX_CFG_REG_KER_P0_OFST);	\
    EFHW_ASSERT(MAC0_SPEED_LBN         == MAC1_SPEED_LBN);		\
    EFHW_ASSERT(MAC0_SPEED_WIDTH       == MAC1_SPEED_WIDTH);		\
    EFHW_ASSERT(MAC0_LINK_STATUS_LBN   == MAC1_LINK_STATUS_LBN);	\
    EFHW_ASSERT(MAC0_LINK_STATUS_WIDTH == MAC1_LINK_STATUS_WIDTH);	\
    EFHW_ASSERT(MAC1_BCAD_ACPT_LBN     == MAC0_BCAD_ACPT_LBN);		\
    EFHW_ASSERT(MAC1_UC_PROM_LBN       == MAC0_UC_PROM_LBN);		\
    EFHW_ASSERT(MAC0_CTRL_REG_KER_OFST == MAC0_CTRL_REG_OFST);		\
    EFHW_ASSERT(MAC1_CTRL_REG_KER_OFST == MAC1_CTRL_REG_OFST);		\
    EFHW_ASSERT(XM_ADR_LO_REG_KER_P0_OFST  == XM_ADR_LO_REG_P0_OFST);	\
    EFHW_ASSERT(XM_ADR_HI_REG_KER_P0_OFST  == XM_ADR_HI_REG_P0_OFST);	\
    EFHW_ASSERT(XM_RX_PARAM_REG_KER_P0_OFST == XM_RX_PARAM_REG_P0_OFST);

/*--------------------------------------------------------------------
 *
 * Information gathering
 *
 *--------------------------------------------------------------------*/

/*! Get MAC current address - i.e not necessarily the one in the EEPROM */
static inline void GDACT10mac_get_mac_addr(struct efhw_nic *nic)
{
	uint val1, val2;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	FALCON_LOCK_DECL;

	GDACT10_MAC_ASSERT_VALID();

	EFHW_ASSERT(XM_ADR_LO_LBN == 0);
	EFHW_ASSERT(XM_ADR_LO_WIDTH == 32);
	EFHW_ASSERT(XM_ADR_HI_LBN == 0);
	EFHW_ASSERT(XM_ADR_HI_WIDTH == 16);

	FALCON_LOCK_LOCK(nic);

	val1 = readl(efhw_kva + XM_ADR_LO_REG_P0_OFST);
	val2 = readl(efhw_kva + XM_ADR_HI_REG_P0_OFST);

	FALCON_LOCK_UNLOCK(nic);

	/* The HW scores no points for consistency */
	nic->mac_addr[5] = (val2 & 0x0000ff00) >> 8;
	nic->mac_addr[4] = (val2 & 0x000000ff) >> 0;
	nic->mac_addr[3] = (val1 & 0xff000000) >> 24;
	nic->mac_addr[2] = (val1 & 0x00ff0000) >> 16;
	nic->mac_addr[1] = (val1 & 0x0000ff00) >> 8;
	nic->mac_addr[0] = (val1 & 0x000000ff) >> 0;
}


/********************************************************************
 * Call one or another function
 */

void falcon_get_mac_addr(struct efhw_nic *nic)
{
	if (nic->flags & NIC_FLAG_10G)
		GDACT10mac_get_mac_addr(nic);
	else
		mentormac_get_mac_addr(nic);
}
