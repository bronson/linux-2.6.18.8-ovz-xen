/****************************************************************************
 * Driver for Solarflare network controllers -
 *          resource management for Xen backend, OpenOnload, etc
 *           (including support for SFE4001 10GBT NIC)
 *
 * This file contains Falcon hardware support.
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

#include <ci/driver/efab/hardware.h>
#include <ci/efhw/debug.h>
#include <ci/efhw/iopage.h>
#include <ci/efhw/falcon.h>
#include <ci/efhw/falcon_hash.h>
#include <ci/efhw/nic.h>
#include <ci/efhw/eventq.h>
#include <ci/efhw/checks.h>


/*----------------------------------------------------------------------------
 *
 * Workarounds and options
 *
 *---------------------------------------------------------------------------*/

/* on for debug builds */
#ifndef NDEBUG
#  define FALCON_FULL_FILTER_CACHE 1	/* complete SW shadow of filter tbl */
#  define FALCON_VERIFY_FILTERS    0
#else /* Also adds duplicate filter check */
#  define FALCON_FULL_FILTER_CACHE 1	/* keep this on for some security */
#  define FALCON_VERIFY_FILTERS    0
#endif

/* options */
#define RX_FILTER_CTL_SRCH_LIMIT_TCP_FULL 8	/* default search limit */
#define RX_FILTER_CTL_SRCH_LIMIT_TCP_WILD 8	/* default search limit */
#define RX_FILTER_CTL_SRCH_LIMIT_UDP_FULL 8	/* default search limit */
#define RX_FILTER_CTL_SRCH_LIMIT_UDP_WILD 8	/* default search limit */
#define RX_FILTER_CTL_SRCH_FUDGE_WILD 3	/* increase the search limit */
#define RX_FILTER_CTL_SRCH_FUDGE_FULL 1	/* increase the search limit */

#define FALCON_MAC_SET_TYPE_BY_SPEED           1

/* FIXME: We should detect mode at runtime. */
#define FALCON_BUFFER_TABLE_FULL_MODE          1

/*----------------------------------------------------------------------------
 *
 * Debug Macros
 *
 *---------------------------------------------------------------------------*/

#ifndef __KERNEL__
#define _DEBUG_SYM_ extern
#else
#define _DEBUG_SYM_ static inline
#endif

 /*----------------------------------------------------------------------------
  *
  * Macros and forward declarations
  *
  *--------------------------------------------------------------------------*/

#define FALCON_REGION_NUM 4	/* number of supported memory regions */

#define FALCON_BUFFER_TBL_HALF_BYTES 4
#define FALCON_BUFFER_TBL_FULL_BYTES 8

/* Shadow buffer table - hack for testing only */
#if FALCON_BUFFER_TABLE_FULL_MODE == 0
# define FALCON_USE_SHADOW_BUFFER_TABLE 1
#else
# define FALCON_USE_SHADOW_BUFFER_TABLE 0
#endif

#if FALCON_USE_SHADOW_BUFFER_TABLE
static uint64_t _falcon_buffer_table[FALCON_BUFFER_TBL_NUM];
#endif

/*----------------------------------------------------------------------------
 *
 * Header assertion checks
 *
 *---------------------------------------------------------------------------*/

#define FALCON_ASSERT_VALID()	/* nothing yet */

/* Falcon has a 128bit register model but most registers have useful
   defaults or only implement a small number of bits. Some registers
   can be programmed 32bits UNLOCKED all others should be interlocked
   against other threads within the same protection domain.

   Aim is for software to perform the minimum number of writes and
   also to minimise the read-modify-write activity (which generally
   indicates a lack of clarity in the use model).

   Registers which are programmed in this module are listed below
   together with the method of access. Care must be taken to ensure
   remain adequate if the register spec changes.

   All 128bits programmed
    FALCON_BUFFER_TBL_HALF
    RX_FILTER_TBL
    TX_DESC_PTR_TBL
    RX_DESC_PTR_TBL
    DRV_EV_REG

   All 64bits programmed
    FALCON_BUFFER_TBL_FULL

   32 bits are programmed (UNLOCKED)
    EVQ_RPTR_REG

   Low 64bits programmed remainder are written with a random number
    RX_DC_CFG_REG
    TX_DC_CFG_REG
    SRM_RX_DC_CFG_REG
    SRM_TX_DC_CFG_REG
    BUF_TBL_CFG_REG
    BUF_TBL_UPD_REG
    SRM_UPD_EVQ_REG
    EVQ_PTR_TBL
    TIMER_CMD_REG
    TX_PACE_TBL
    FATAL_INTR_REG
    INT_EN_REG (When enabling interrupts)
    TX_FLUSH_DESCQ_REG
    RX_FLUSH_DESCQ

  Read Modify Write on low 32bits remainder are written with a random number
    INT_EN_REG (When sending a driver interrupt)
    DRIVER_REGX

  Read Modify Write on low 64bits remainder are written with a random number
   SRM_CFG_REG_OFST
   RX_CFG_REG_OFST
   RX_FILTER_CTL_REG

  Read Modify Write on full 128bits
   TXDP_RESERVED_REG  (aka TXDP_UNDOCUMENTED)
   TX_CFG_REG

*/

/*----------------------------------------------------------------------------
 *
 * Filters static data
 *
 *---------------------------------------------------------------------------*/

/* Defaults are set here to support dma.c */
static unsigned tcp_full_srch_limit = RX_FILTER_CTL_SRCH_LIMIT_TCP_FULL;
static unsigned tcp_wild_srch_limit = RX_FILTER_CTL_SRCH_LIMIT_TCP_WILD;
static unsigned udp_full_srch_limit = RX_FILTER_CTL_SRCH_LIMIT_UDP_FULL;
static unsigned udp_wild_srch_limit = RX_FILTER_CTL_SRCH_LIMIT_UDP_WILD;

#if FALCON_VERIFY_FILTERS
static void _falcon_nic_ipfilter_sanity(struct efhw_nic *nic);
#endif

/*----------------------------------------------------------------------------
 *
 * Filters low-level register interface
 *
 *---------------------------------------------------------------------------*/

/* Build the filter entry */
static void
_falcon_nic_ipfilter_build(struct efhw_nic *nic,
			   int tcp, int full, int rss_b0, int scat_b0,
			   uint filter_i, uint dmaq_id,
			   unsigned saddr_le32, unsigned sport_le16,
			   unsigned daddr_le32, unsigned dport_le16,
			   uint64_t *q0, uint64_t *q1)
{
	uint64_t v1, v2, v3, v4;
	int type = tcp << 4 | full;

	v4 = (((!tcp) << __DW4(TCP_UDP_1_LBN)) |
	      (dmaq_id << __DW4(RXQ_ID_1_LBN)));

	switch (nic->devtype.variant) {
	case 'A':
		EFHW_ASSERT(!rss_b0);
		break;
	case 'B':
		v4 |= scat_b0 << __DW4(SCATTER_EN_1_B0_LBN);
		v4 |= rss_b0 << __DW4(RSS_EN_1_B0_LBN);
		break;
	default:
		EFHW_ASSERT(0);
		break;
	}

	v3 = daddr_le32;

	switch (type) {

	case 0x11:		/* TCP_FULL */
	case 0x01:		/* UDP_FULL */
		v2 = ((dport_le16 << __DW2(DEST_PORT_TCP_1_LBN)) |
		      (__HIGH(saddr_le32, SRC_IP_1_LBN, SRC_IP_1_WIDTH)));
		v1 = ((__LOW(saddr_le32, SRC_IP_1_LBN, SRC_IP_1_WIDTH)) |
		      (sport_le16 << SRC_TCP_DEST_UDP_1_LBN));
		break;

	case 0x10:		/* TCP_WILD */
		v2 = ((uint64_t) dport_le16 << __DW2(DEST_PORT_TCP_1_LBN));
		v1 = 0;
		break;

	case 0x00:		/* UDP_WILD */
		v2 = 0;
		v1 = ((uint64_t) dport_le16 << SRC_TCP_DEST_UDP_0_LBN);
		break;

	default:
		EFHW_ASSERT(0);
		v2 = 0;
		v1 = 0;
	}

	*q0 = (v2 << 32) | v1;
	*q1 = (v4 << 32) | v3;
}

static void
_falcon_nic_ipfilter_set(struct efhw_nic *nic, int tcp,
			 int full, int rss_b0, int scat_b0,
			 uint filter_i, uint dmaq_id,
			 unsigned saddr_le32, unsigned sport_le16,
			 unsigned daddr_le32, unsigned dport_le16)
{
	uint64_t q0, q1;

	/* wish you wouldn't do this */
	EFHW_BUILD_ASSERT(RX_FILTER_TBL1_OFST ==
			  RX_FILTER_TBL0_OFST + FALCON_REGISTER128);
	EFHW_BUILD_ASSERT(TCP_UDP_1_LBN == TCP_UDP_0_LBN);
	EFHW_BUILD_ASSERT(RXQ_ID_1_LBN == RXQ_ID_0_LBN);
	EFHW_BUILD_ASSERT(DEST_IP_1_LBN == DEST_IP_0_LBN);
	EFHW_BUILD_ASSERT(DEST_PORT_TCP_1_LBN == DEST_PORT_TCP_0_LBN);
	EFHW_BUILD_ASSERT(SRC_IP_1_LBN == SRC_IP_0_LBN);
	EFHW_BUILD_ASSERT(SRC_TCP_DEST_UDP_1_LBN == SRC_TCP_DEST_UDP_0_LBN);
	EFHW_BUILD_ASSERT(SCATTER_EN_1_B0_LBN == SCATTER_EN_0_B0_LBN);
	EFHW_BUILD_ASSERT(RSS_EN_1_B0_LBN == RSS_EN_0_B0_LBN);

	EFHW_BUILD_ASSERT(TCP_UDP_1_WIDTH == TCP_UDP_0_WIDTH);
	EFHW_BUILD_ASSERT(RXQ_ID_1_WIDTH == RXQ_ID_0_WIDTH);
	EFHW_BUILD_ASSERT(DEST_IP_1_WIDTH == DEST_IP_0_WIDTH);
	EFHW_BUILD_ASSERT(DEST_PORT_TCP_1_WIDTH == DEST_PORT_TCP_0_WIDTH);
	EFHW_BUILD_ASSERT(SRC_IP_1_WIDTH == SRC_IP_0_WIDTH);
	EFHW_BUILD_ASSERT(SRC_TCP_DEST_UDP_1_WIDTH == SRC_TCP_DEST_UDP_0_WIDTH);
	EFHW_BUILD_ASSERT(SCATTER_EN_1_B0_WIDTH == SCATTER_EN_0_B0_WIDTH);
	EFHW_BUILD_ASSERT(RSS_EN_1_B0_WIDTH == RSS_EN_0_B0_WIDTH);

	/* TODO: Use filter table 1 as well */
	ulong offset = RX_FILTER_TBL0_OFST + filter_i * 2 * FALCON_REGISTER128;

	EFHW_TRACE("%s[%x]: offset=%lx", __FUNCTION__, filter_i, offset);

	EFHW_TRACE("%s[%x]: filter %d tcp %d full %d src=%x:%x dest=%x:%x%s%s",
		   __FUNCTION__, filter_i, tcp, full, dmaq_id,
		   saddr_le32, sport_le16, daddr_le32, dport_le16,
		   rss_b0 ? " RSS" : "", scat_b0 ? " SCAT" : "");

	EFHW_ASSERT(filter_i < nic->filter_tbl_size);

	/* dword 4 */
	__DW4CHCK(TCP_UDP_1_LBN, TCP_UDP_1_WIDTH);
	__DW4CHCK(RXQ_ID_1_LBN, RXQ_ID_1_WIDTH);

	__RANGECHCK(tcp, TCP_UDP_1_WIDTH);
	__RANGECHCK(dmaq_id, RXQ_ID_1_WIDTH);

	/* dword 3 */
	__DW3CHCK(DEST_IP_1_LBN, DEST_IP_1_WIDTH);
	__RANGECHCK(daddr_le32, DEST_IP_1_WIDTH);

	/* dword 2 */
	__DW2CHCK(DEST_PORT_TCP_1_LBN, DEST_PORT_TCP_1_WIDTH);
	__LWCHK(SRC_IP_1_LBN, SRC_IP_1_WIDTH);
	__RANGECHCK(saddr_le32, SRC_IP_1_WIDTH);

	/* dword 1 */
	__DWCHCK(SRC_TCP_DEST_UDP_1_LBN, SRC_TCP_DEST_UDP_1_WIDTH);
	__RANGECHCK(sport_le16, SRC_TCP_DEST_UDP_1_WIDTH);
	__RANGECHCK(dport_le16, SRC_TCP_DEST_UDP_1_WIDTH);

	/* Falcon requires 128 bit atomic access for this register */
	_falcon_nic_ipfilter_build(nic, tcp, full, rss_b0, scat_b0,
				   filter_i, dmaq_id, saddr_le32, sport_le16,
				   daddr_le32, dport_le16, &q0, &q1);

	EFHW_TRACE("%s[%x]@%p+%lx: %" PRIx64 " %" PRIx64, __FUNCTION__,
		   filter_i, EFHW_KVA(nic), offset, q0, q1);

	falcon_write_qq(EFHW_KVA(nic) + offset, q0, q1);
	mmiowb();

#if FALCON_VERIFY_FILTERS
	{
		uint64_t q0read, q1read;

		/* Read a different entry first - entry BIU flushed shadow */
		falcon_read_qq(EFHW_KVA(nic) + offset+0x10, &q0read, &q1read);
		falcon_read_qq(EFHW_KVA(nic) + offset, &q0read, &q1read);
		EFHW_ASSERT(q0read == q0);
		EFHW_ASSERT(q1read == q1);

		_falcon_nic_ipfilter_sanity(nic);
	}
#endif
}

static void _falcon_nic_ipfilter_clear(struct efhw_nic *nic, uint filter_i)
{
	/* TODO: Use filter table 1 as well */
	ulong offset = RX_FILTER_TBL0_OFST + filter_i * 2 * FALCON_REGISTER128;

	EFHW_ASSERT(filter_i < nic->filter_tbl_size);

	EFHW_TRACE("%s[%x]", __FUNCTION__, filter_i);

	/* Falcon requires 128 bit atomic access for this register */
	falcon_write_qq(EFHW_KVA(nic) + offset, 0, 0);
	mmiowb();
#if FALCON_VERIFY_FILTERS
	{
		uint64_t q0read, q1read;

		/* Read a different entry first - entry BIU flushed shadow */
		falcon_read_qq(EFHW_KVA(nic) + offset+0x10, &q0read, &q1read);
		falcon_read_qq(EFHW_KVA(nic) + offset, &q0read, &q1read);
		EFHW_ASSERT(q0read == 0);
		EFHW_ASSERT(q1read == 0);

		_falcon_nic_ipfilter_sanity(nic);
	}
#endif
}

/*----------------------------------------------------------------------------
 *
 * DMAQ low-level register interface
 *
 *---------------------------------------------------------------------------*/

static unsigned dmaq_sizes[] = {
	512,
	EFHW_1K,
	EFHW_2K,
	EFHW_4K,
};

#define N_DMAQ_SIZES  (sizeof(dmaq_sizes) / sizeof(dmaq_sizes[0]))

static inline ulong falcon_dma_tx_q_offset(struct efhw_nic *nic, unsigned dmaq)
{
	EFHW_ASSERT(dmaq < FALCON_DMAQ_NUM);
	return TX_DESC_PTR_TBL_OFST + dmaq * FALCON_REGISTER128;
}

static inline uint falcon_dma_tx_q_size_index(uint dmaq_size)
{
	uint i;

	/* size must be one of the various options, otherwise we assert */
	for (i = 0; i < N_DMAQ_SIZES; i++) {
		if (dmaq_size == dmaq_sizes[i])
			break;
	}
	EFHW_ASSERT(i < N_DMAQ_SIZES);
	return i;
}

static void
falcon_dmaq_tx_q_init(struct efhw_nic *nic,
		      uint dmaq, uint evq_id, uint own_id,
		      uint tag, uint dmaq_size, uint buf_idx, uint flags)
{
	FALCON_LOCK_DECL;
	uint index, desc_type;
	uint64_t val1, val2, val3;
	ulong offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	/* Q attributes */
	int iscsi_hdig_en = ((flags & EFHW_VI_ISCSI_TX_HDIG_EN) != 0);
	int iscsi_ddig_en = ((flags & EFHW_VI_ISCSI_TX_DDIG_EN) != 0);
	int csum_ip_dis = ((flags & EFHW_VI_TX_IP_CSUM_DIS) != 0);
	int csum_tcp_dis = ((flags & EFHW_VI_TX_TCPUDP_CSUM_DIS) != 0);
	int non_ip_drop_dis = ((flags & EFHW_VI_TX_TCPUDP_ONLY) == 0);

	/* initialise the TX descriptor queue pointer table */

	/* NB physical vs buffer addressing is determined by the Queue ID. */

	offset = falcon_dma_tx_q_offset(nic, dmaq);
	index = falcon_dma_tx_q_size_index(dmaq_size);

	/* allow VI flag to override this queue's descriptor type */
	desc_type = (flags & EFHW_VI_TX_PHYS_ADDR_EN) ? 0 : 1;

	/* bug9403: It is dangerous to allow buffer-addressed queues to
	 * have owner_id=0. */
	EFHW_ASSERT((own_id > 0) || desc_type == 0);

	/* dword 1 */
	__DWCHCK(TX_DESCQ_FLUSH_LBN, TX_DESCQ_FLUSH_WIDTH);
	__DWCHCK(TX_DESCQ_TYPE_LBN, TX_DESCQ_TYPE_WIDTH);
	__DWCHCK(TX_DESCQ_SIZE_LBN, TX_DESCQ_SIZE_WIDTH);
	__DWCHCK(TX_DESCQ_LABEL_LBN, TX_DESCQ_LABEL_WIDTH);
	__DWCHCK(TX_DESCQ_OWNER_ID_LBN, TX_DESCQ_OWNER_ID_WIDTH);

	__LWCHK(TX_DESCQ_EVQ_ID_LBN, TX_DESCQ_EVQ_ID_WIDTH);

	__RANGECHCK(1, TX_DESCQ_FLUSH_WIDTH);
	__RANGECHCK(desc_type, TX_DESCQ_TYPE_WIDTH);
	__RANGECHCK(index, TX_DESCQ_SIZE_WIDTH);
	__RANGECHCK(tag, TX_DESCQ_LABEL_WIDTH);
	__RANGECHCK(own_id, TX_DESCQ_OWNER_ID_WIDTH);
	__RANGECHCK(evq_id, TX_DESCQ_EVQ_ID_WIDTH);

	val1 = ((desc_type << TX_DESCQ_TYPE_LBN) |
		(index << TX_DESCQ_SIZE_LBN) |
		(tag << TX_DESCQ_LABEL_LBN) |
		(own_id << TX_DESCQ_OWNER_ID_LBN) |
		(__LOW(evq_id, TX_DESCQ_EVQ_ID_LBN, TX_DESCQ_EVQ_ID_WIDTH)));

	/* dword 2 */
	__DW2CHCK(TX_DESCQ_BUF_BASE_ID_LBN, TX_DESCQ_BUF_BASE_ID_WIDTH);
	__RANGECHCK(buf_idx, TX_DESCQ_BUF_BASE_ID_WIDTH);

	val2 = ((__HIGH(evq_id, TX_DESCQ_EVQ_ID_LBN, TX_DESCQ_EVQ_ID_WIDTH)) |
		(buf_idx << __DW2(TX_DESCQ_BUF_BASE_ID_LBN)));

	/* dword 3 */
	__DW3CHCK(TX_ISCSI_HDIG_EN_LBN, TX_ISCSI_HDIG_EN_WIDTH);
	__DW3CHCK(TX_ISCSI_DDIG_EN_LBN, TX_ISCSI_DDIG_EN_WIDTH);
	__RANGECHCK(iscsi_hdig_en, TX_ISCSI_HDIG_EN_WIDTH);
	__RANGECHCK(iscsi_ddig_en, TX_ISCSI_DDIG_EN_WIDTH);

	val3 = ((iscsi_hdig_en << __DW3(TX_ISCSI_HDIG_EN_LBN)) |
		(iscsi_ddig_en << __DW3(TX_ISCSI_DDIG_EN_LBN)) |
		(1 << __DW3(TX_DESCQ_EN_LBN)));	/* queue enable bit */

	switch (nic->devtype.variant) {
	case 'B':
		__DW3CHCK(TX_NON_IP_DROP_DIS_B0_LBN,
			  TX_NON_IP_DROP_DIS_B0_WIDTH);
		__DW3CHCK(TX_IP_CHKSM_DIS_B0_LBN, TX_IP_CHKSM_DIS_B0_WIDTH);
		__DW3CHCK(TX_TCP_CHKSM_DIS_B0_LBN, TX_TCP_CHKSM_DIS_B0_WIDTH);

		val3 |= ((non_ip_drop_dis << __DW3(TX_NON_IP_DROP_DIS_B0_LBN))|
			 (csum_ip_dis << __DW3(TX_IP_CHKSM_DIS_B0_LBN)) |
			 (csum_tcp_dis << __DW3(TX_TCP_CHKSM_DIS_B0_LBN)));
		break;
	case 'A':
		if (csum_ip_dis || csum_tcp_dis || !non_ip_drop_dis)
			EFHW_WARN
				("%s: bad settings for A1 csum_ip_dis=%d "
				 "csum_tcp_dis=%d non_ip_drop_dis=%d",
				 __FUNCTION__, csum_ip_dis,
				 csum_tcp_dis, non_ip_drop_dis);
		break;
	default:
		EFHW_ASSERT(0);
		break;
	}

	EFHW_TRACE("%s: txq %x evq %u tag %x id %x buf %x "
		   "%x:%x:%x->%" PRIx64 ":%" PRIx64 ":%" PRIx64,
		   __FUNCTION__,
		   dmaq, evq_id, tag, own_id, buf_idx, dmaq_size,
		   iscsi_hdig_en, iscsi_ddig_en, val1, val2, val3);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + offset, ((val2 << 32) | val1), val3);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
	return;
}

static inline ulong
falcon_dma_rx_q_offset(struct efhw_nic *nic, unsigned dmaq)
{
	EFHW_ASSERT(dmaq < FALCON_DMAQ_NUM);
	return RX_DESC_PTR_TBL_OFST + dmaq * FALCON_REGISTER128;
}

static void
falcon_dmaq_rx_q_init(struct efhw_nic *nic,
		      uint dmaq, uint evq_id, uint own_id,
		      uint tag, uint dmaq_size, uint buf_idx, uint flags)
{
	FALCON_LOCK_DECL;
	uint i, desc_type = 1;
	uint64_t val1, val2, val3;
	ulong offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	/* Q attributes */
#if BUG5762_WORKAROUND
	int jumbo = 1;		/* Queues must not have mixed types */
#else
	int jumbo = ((flags & EFHW_VI_JUMBO_EN) != 0);
#endif
	int iscsi_hdig_en = ((flags & EFHW_VI_ISCSI_RX_HDIG_EN) != 0);
	int iscsi_ddig_en = ((flags & EFHW_VI_ISCSI_RX_DDIG_EN) != 0);

	/* initialise the TX descriptor queue pointer table */
	offset = falcon_dma_rx_q_offset(nic, dmaq);

	/* size must be one of the various options, otherwise we assert */
	for (i = 0; i < N_DMAQ_SIZES; i++) {
		if (dmaq_size == dmaq_sizes[i])
			break;
	}
	EFHW_ASSERT(i < N_DMAQ_SIZES);

	/* allow VI flag to override this queue's descriptor type */
	desc_type = (flags & EFHW_VI_RX_PHYS_ADDR_EN) ? 0 : 1;

	/* bug9403: It is dangerous to allow buffer-addressed queues to have
	 * owner_id=0 */
	EFHW_ASSERT((own_id > 0) || desc_type == 0);

	/* dword 1 */
	__DWCHCK(RX_DESCQ_EN_LBN, RX_DESCQ_EN_WIDTH);
	__DWCHCK(RX_DESCQ_JUMBO_LBN, RX_DESCQ_JUMBO_WIDTH);
	__DWCHCK(RX_DESCQ_TYPE_LBN, RX_DESCQ_TYPE_WIDTH);
	__DWCHCK(RX_DESCQ_SIZE_LBN, RX_DESCQ_SIZE_WIDTH);
	__DWCHCK(RX_DESCQ_LABEL_LBN, RX_DESCQ_LABEL_WIDTH);
	__DWCHCK(RX_DESCQ_OWNER_ID_LBN, RX_DESCQ_OWNER_ID_WIDTH);

	__LWCHK(RX_DESCQ_EVQ_ID_LBN, RX_DESCQ_EVQ_ID_WIDTH);

	__RANGECHCK(1, RX_DESCQ_EN_WIDTH);
	__RANGECHCK(jumbo, RX_DESCQ_JUMBO_WIDTH);
	__RANGECHCK(desc_type, RX_DESCQ_TYPE_WIDTH);
	__RANGECHCK(i, RX_DESCQ_SIZE_WIDTH);
	__RANGECHCK(tag, RX_DESCQ_LABEL_WIDTH);
	__RANGECHCK(own_id, RX_DESCQ_OWNER_ID_WIDTH);
	__RANGECHCK(evq_id, RX_DESCQ_EVQ_ID_WIDTH);

	val1 = ((1 << RX_DESCQ_EN_LBN) |
		(jumbo << RX_DESCQ_JUMBO_LBN) |
		(desc_type << RX_DESCQ_TYPE_LBN) |
		(i << RX_DESCQ_SIZE_LBN) |
		(tag << RX_DESCQ_LABEL_LBN) |
		(own_id << RX_DESCQ_OWNER_ID_LBN) |
		(__LOW(evq_id, RX_DESCQ_EVQ_ID_LBN, RX_DESCQ_EVQ_ID_WIDTH)));

	/* dword 2 */
	__DW2CHCK(RX_DESCQ_BUF_BASE_ID_LBN, RX_DESCQ_BUF_BASE_ID_WIDTH);
	__RANGECHCK(buf_idx, RX_DESCQ_BUF_BASE_ID_WIDTH);

	val2 = ((__HIGH(evq_id, RX_DESCQ_EVQ_ID_LBN, RX_DESCQ_EVQ_ID_WIDTH)) |
		(buf_idx << __DW2(RX_DESCQ_BUF_BASE_ID_LBN)));

	/* dword 3 */
	__DW3CHCK(RX_ISCSI_HDIG_EN_LBN, RX_ISCSI_HDIG_EN_WIDTH);
	__DW3CHCK(RX_ISCSI_DDIG_EN_LBN, RX_ISCSI_DDIG_EN_WIDTH);
	__RANGECHCK(iscsi_hdig_en, RX_ISCSI_HDIG_EN_WIDTH);
	__RANGECHCK(iscsi_ddig_en, RX_ISCSI_DDIG_EN_WIDTH);

	val3 = (iscsi_hdig_en << __DW3(RX_ISCSI_HDIG_EN_LBN)) |
	    (iscsi_ddig_en << __DW3(RX_ISCSI_DDIG_EN_LBN));

	EFHW_TRACE("%s: rxq %x evq %u tag %x id %x buf %x %s "
		   "%x:%x:%x -> %" PRIx64 ":%" PRIx64 ":%" PRIx64,
		   __FUNCTION__,
		   dmaq, evq_id, tag, own_id, buf_idx,
		   jumbo ? "jumbo" : "normal", dmaq_size,
		   iscsi_hdig_en, iscsi_ddig_en, val1, val2, val3);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + offset, ((val2 << 32) | val1), val3);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
	return;
}

static void falcon_dmaq_tx_q_disable(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	uint64_t val1, val2, val3;
	ulong offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	/* initialise the TX descriptor queue pointer table */

	offset = falcon_dma_tx_q_offset(nic, dmaq);

	/* dword 1 */
	__DWCHCK(TX_DESCQ_TYPE_LBN, TX_DESCQ_TYPE_WIDTH);

	val1 = ((uint64_t) 1 << TX_DESCQ_TYPE_LBN);

	/* dword 2 */
	val2 = 0;

	/* dword 3 */
	val3 = (0 << __DW3(TX_DESCQ_EN_LBN));	/* queue enable bit */

	EFHW_TRACE("%s: %x->%" PRIx64 ":%" PRIx64 ":%" PRIx64,
		   __FUNCTION__, dmaq, val1, val2, val3);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + offset, ((val2 << 32) | val1), val3);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
	return;
}

static void falcon_dmaq_rx_q_disable(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	uint64_t val1, val2, val3;
	ulong offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	/* initialise the TX descriptor queue pointer table */
	offset = falcon_dma_rx_q_offset(nic, dmaq);

	/* dword 1 */
	__DWCHCK(RX_DESCQ_EN_LBN, RX_DESCQ_EN_WIDTH);
	__DWCHCK(RX_DESCQ_TYPE_LBN, RX_DESCQ_TYPE_WIDTH);

	val1 = ((0 << RX_DESCQ_EN_LBN) | (1 << RX_DESCQ_TYPE_LBN));

	/* dword 2 */
	val2 = 0;

	/* dword 3 */
	val3 = 0;

	EFHW_TRACE("falcon_dmaq_rx_q_disable: %x->%"
		   PRIx64 ":%" PRIx64 ":%" PRIx64,
		   dmaq, val1, val2, val3);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + offset, ((val2 << 32) | val1), val3);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
	return;
}


/*----------------------------------------------------------------------------
 *
 * Buffer Table low-level register interface
 *
 *---------------------------------------------------------------------------*/

/*! Convert a (potentially) 64-bit physical address to 32-bits.  Every use
** of this function is a place where we're not 64-bit clean.
*/
static inline uint32_t dma_addr_to_u32(dma_addr_t addr)
{
	/* Top bits had better be zero! */
	EFHW_ASSERT(addr == (addr & 0xffffffff));
	return (uint32_t) addr;
}

static inline uint32_t
falcon_nic_buffer_table_entry32_mk(dma_addr_t dma_addr, int own_id)
{
	uint32_t dma_addr32 = FALCON_BUFFER_4K_PAGE(dma_addr_to_u32(dma_addr));

	/* don't do this to me */
	EFHW_BUILD_ASSERT(BUF_ADR_HBUF_ODD_LBN == BUF_ADR_HBUF_EVEN_LBN + 32);
	EFHW_BUILD_ASSERT(BUF_OWNER_ID_HBUF_ODD_LBN ==
			  BUF_OWNER_ID_HBUF_EVEN_LBN + 32);

	EFHW_BUILD_ASSERT(BUF_OWNER_ID_HBUF_ODD_WIDTH ==
			  BUF_OWNER_ID_HBUF_EVEN_WIDTH);
	EFHW_BUILD_ASSERT(BUF_ADR_HBUF_ODD_WIDTH == BUF_ADR_HBUF_EVEN_WIDTH);

	__DWCHCK(BUF_ADR_HBUF_EVEN_LBN, BUF_ADR_HBUF_EVEN_WIDTH);
	__DWCHCK(BUF_OWNER_ID_HBUF_EVEN_LBN, BUF_OWNER_ID_HBUF_EVEN_WIDTH);

	__RANGECHCK(dma_addr32, BUF_ADR_HBUF_EVEN_WIDTH);
	__RANGECHCK(own_id, BUF_OWNER_ID_HBUF_EVEN_WIDTH);

	return ((dma_addr32 << BUF_ADR_HBUF_EVEN_LBN) |
		(own_id << BUF_OWNER_ID_HBUF_EVEN_LBN));
}

static inline uint64_t
falcon_nic_buffer_table_entry64_mk(dma_addr_t dma_addr,
				   int bufsz,	/* bytes */
				   int region, int own_id)
{
	__DW2CHCK(IP_DAT_BUF_SIZE_LBN, IP_DAT_BUF_SIZE_WIDTH);
	__DW2CHCK(BUF_ADR_REGION_LBN, BUF_ADR_REGION_WIDTH);
	__LWCHK(BUF_ADR_FBUF_LBN, BUF_ADR_FBUF_WIDTH);
	__DWCHCK(BUF_OWNER_ID_FBUF_LBN, BUF_OWNER_ID_FBUF_WIDTH);

	EFHW_ASSERT((bufsz == EFHW_4K) || (bufsz == EFHW_8K));

	dma_addr = (dma_addr >> 12) & __FALCON_MASK64(BUF_ADR_FBUF_WIDTH);

	__RANGECHCK(dma_addr, BUF_ADR_FBUF_WIDTH);
	__RANGECHCK(1, IP_DAT_BUF_SIZE_WIDTH);
	__RANGECHCK(region, BUF_ADR_REGION_WIDTH);
	__RANGECHCK(own_id, BUF_OWNER_ID_FBUF_WIDTH);

	return (((uint64_t) (bufsz == EFHW_8K) << IP_DAT_BUF_SIZE_LBN) |
		((uint64_t) region << BUF_ADR_REGION_LBN) |
		((uint64_t) dma_addr << BUF_ADR_FBUF_LBN) |
		((uint64_t) own_id << BUF_OWNER_ID_FBUF_LBN));
}

static inline void
_falcon_nic_buffer_table_set32(struct efhw_nic *nic,
			       dma_addr_t dma_addr, uint bufsz,
			       uint region, /* not used */
			       int own_id, int buffer_id)
{
	/* programming the half table needs to be done in pairs. */
	uint64_t entry, val, shift;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	efhw_ioaddr_t offset;

	EFHW_BUILD_ASSERT(BUF_ADR_HBUF_ODD_LBN == BUF_ADR_HBUF_EVEN_LBN + 32);
	EFHW_BUILD_ASSERT(BUF_OWNER_ID_HBUF_ODD_LBN ==
			  BUF_OWNER_ID_HBUF_EVEN_LBN + 32);

	shift = (buffer_id & 1) ? 32 : 0;

	offset = (efhw_kva + BUF_HALF_TBL_OFST +
		  ((buffer_id & ~1) * FALCON_BUFFER_TBL_HALF_BYTES));

	entry = falcon_nic_buffer_table_entry32_mk(dma_addr_to_u32(dma_addr),
						   own_id);

#if FALCON_USE_SHADOW_BUFFER_TABLE
	val = _falcon_buffer_table[buffer_id & ~1];
#else
	/* This will not work unless we've completed
	 * the buffer table updates */
	falcon_read_q(offset, &val);
#endif
	val &= ~(((uint64_t) 0xffffffff) << shift);
	val |= (entry << shift);

	EFHW_TRACE("%s[%x]: " ci_dma_addr_fmt ":%x:%" PRIx64 "->%x = %"
		   PRIx64, __FUNCTION__, buffer_id, dma_addr, own_id, entry,
		   (unsigned)(offset - efhw_kva), val);

	/* Falcon requires that access to this register is serialised */
	falcon_write_q(offset, val);

	/* NB. No mmiowb().  Caller should do that e.g by calling commit  */

#if FALCON_USE_SHADOW_BUFFER_TABLE
	_falcon_buffer_table[buffer_id & ~1] = val;
#endif

	/* Confirm the entry if the event queues haven't been set up. */
	if (!nic->irq_handler) {
		uint64_t new_val;
		int count = 0;
		while (1) {
			mmiowb();
			falcon_read_q(offset, &new_val);
			if (new_val == val)
				break;
			count++;
			if (count > 1000) {
				EFHW_WARN("%s: poll Timeout", __FUNCTION__);
				break;
			}
			udelay(1);
		}
	}
}

static inline void
_falcon_nic_buffer_table_set64(struct efhw_nic *nic,
			       dma_addr_t dma_addr, uint bufsz,
			       uint region, int own_id, int buffer_id)
{
	efhw_ioaddr_t offset;
	uint64_t entry;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	EFHW_ASSERT(region < FALCON_REGION_NUM);

	EFHW_ASSERT((bufsz == EFHW_4K) ||
		    (bufsz == EFHW_8K && FALCON_BUFFER_TABLE_FULL_MODE));

	offset = (efhw_kva + BUF_FULL_TBL_OFST +
		  (buffer_id * FALCON_BUFFER_TBL_FULL_BYTES));

	entry = falcon_nic_buffer_table_entry64_mk(dma_addr, bufsz, region,
						   own_id);

	EFHW_TRACE("%s[%x]: " ci_dma_addr_fmt
		   ":bufsz=%x:region=%x:ownid=%x",
		   __FUNCTION__, buffer_id, dma_addr, bufsz, region, own_id);

	EFHW_TRACE("%s: BUF[%x]:NIC[%x]->%" PRIx64,
		   __FUNCTION__, buffer_id,
		   (unsigned int)(offset - efhw_kva), entry);

	/* Falcon requires that access to this register is serialised */
	falcon_write_q(offset, entry);

	/* NB. No mmiowb().  Caller should do that e.g by calling commit */

	/* Confirm the entry if the event queues haven't been set up. */
	if (!nic->irq_handler) {
		uint64_t new_entry;
		int count = 0;
		while (1) {
			mmiowb();
			falcon_read_q(offset, &new_entry);
			if (new_entry == entry)
				return;
			count++;
			if (count > 1000) {
				EFHW_WARN("%s: poll Timeout waiting for "
					  "value %"PRIx64
					  " (last was %"PRIx64")",
					  __FUNCTION__, entry, new_entry);
				break;
			}
			udelay(1);
		}
	}
}

#if FALCON_BUFFER_TABLE_FULL_MODE
#define _falcon_nic_buffer_table_set _falcon_nic_buffer_table_set64
#else
#define _falcon_nic_buffer_table_set _falcon_nic_buffer_table_set32
#endif

static inline void _falcon_nic_buffer_table_commit(struct efhw_nic *nic)
{
	/* MUST be called holding the FALCON_LOCK */
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	uint64_t cmd;

	EFHW_BUILD_ASSERT(BUF_TBL_UPD_REG_KER_OFST == BUF_TBL_UPD_REG_OFST);

	__DW2CHCK(BUF_UPD_CMD_LBN, BUF_UPD_CMD_WIDTH);
	__RANGECHCK(1, BUF_UPD_CMD_WIDTH);

	cmd = ((uint64_t) 1 << BUF_UPD_CMD_LBN);

	/* Falcon requires 128 bit atomic access for this register */
	falcon_write_qq(efhw_kva + BUF_TBL_UPD_REG_OFST,
			cmd, FALCON_ATOMIC_UPD_REG);
	mmiowb();

	nic->buf_commit_outstanding++;
	EFHW_TRACE("COMMIT REQ out=%d", nic->buf_commit_outstanding);
}

static void falcon_nic_buffer_table_commit(struct efhw_nic *nic)
{
	/* nothing to do */
}

static inline void
_falcon_nic_buffer_table_clear(struct efhw_nic *nic, int buffer_id, int num)
{
	uint64_t cmd;
	uint64_t start_id = buffer_id;
	uint64_t end_id = buffer_id + num - 1;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	efhw_ioaddr_t offset = (efhw_kva + BUF_TBL_UPD_REG_OFST);

	EFHW_BUILD_ASSERT(BUF_TBL_UPD_REG_KER_OFST == BUF_TBL_UPD_REG_OFST);

#if !FALCON_BUFFER_TABLE_FULL_MODE
	/* buffer_ids in half buffer mode reference pairs of buffers */
	EFHW_ASSERT(buffer_id % 1 == 0);
	EFHW_ASSERT(num % 1 == 0);
	start_id = start_id >> 1;
	end_id = end_id >> 1;
#endif

	EFHW_ASSERT(num >= 1);

	__DWCHCK(BUF_CLR_START_ID_LBN, BUF_CLR_START_ID_WIDTH);
	__DW2CHCK(BUF_CLR_END_ID_LBN, BUF_CLR_END_ID_WIDTH);

	__DW2CHCK(BUF_CLR_CMD_LBN, BUF_CLR_CMD_WIDTH);
	__RANGECHCK(1, BUF_CLR_CMD_WIDTH);

	__RANGECHCK(start_id, BUF_CLR_START_ID_WIDTH);
	__RANGECHCK(end_id, BUF_CLR_END_ID_WIDTH);

	cmd = (((uint64_t) 1 << BUF_CLR_CMD_LBN) |
	       (start_id << BUF_CLR_START_ID_LBN) |
	       (end_id << BUF_CLR_END_ID_LBN));

	/* Falcon requires 128 bit atomic access for this register */
	falcon_write_qq(offset, cmd, FALCON_ATOMIC_UPD_REG);
	mmiowb();

	nic->buf_commit_outstanding++;
	EFHW_TRACE("COMMIT CLEAR out=%d", nic->buf_commit_outstanding);
}

/*----------------------------------------------------------------------------
 *
 * Events low-level register interface
 *
 *---------------------------------------------------------------------------*/

static unsigned eventq_sizes[] = {
	512,
	EFHW_1K,
	EFHW_2K,
	EFHW_4K,
	EFHW_8K,
	EFHW_16K,
	EFHW_32K
};

#define N_EVENTQ_SIZES  (sizeof(eventq_sizes) / sizeof(eventq_sizes[0]))

static inline void falcon_nic_srm_upd_evq(struct efhw_nic *nic, int evq)
{
	/* set up the eventq which will receive events from the SRAM module.
	 * i.e buffer table updates and clears, TX and RX aperture table
	 * updates */

	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	EFHW_BUILD_ASSERT(SRM_UPD_EVQ_REG_OFST == SRM_UPD_EVQ_REG_KER_OFST);

	EFHW_ASSERT((evq == FALCON_EVQ_KERNEL0) || (evq == FALCON_EVQ_CHAR) ||
		    (evq == FALCON_EVQ_NONIRQ));

	__DWCHCK(SRM_UPD_EVQ_ID_LBN, SRM_UPD_EVQ_ID_WIDTH);
	__RANGECHCK(evq, SRM_UPD_EVQ_ID_WIDTH);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + SRM_UPD_EVQ_REG_OFST,
			((uint64_t) evq << SRM_UPD_EVQ_ID_LBN),
			FALCON_ATOMIC_SRPM_UDP_EVQ_REG);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

static inline void
falcon_nic_evq_ptr_tbl(struct efhw_nic *nic,
		       uint evq,	/* evq id */
		       uint enable,	/* 1 to enable, 0 to disable */
		       uint buf_base_id,/* Buffer table base for EVQ */
		       uint evq_size	/* Number of events */ )
{
	FALCON_LOCK_DECL;
	uint i, val;
	ulong offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	/* size must be one of the various options, otherwise we assert */
	for (i = 0; i < N_EVENTQ_SIZES; i++) {
		if (evq_size <= eventq_sizes[i])
			break;
	}
	EFHW_ASSERT(i < N_EVENTQ_SIZES);

	__DWCHCK(EVQ_BUF_BASE_ID_LBN, EVQ_BUF_BASE_ID_WIDTH);
	__DWCHCK(EVQ_SIZE_LBN, EVQ_SIZE_WIDTH);
	__DWCHCK(EVQ_EN_LBN, EVQ_EN_WIDTH);

	__RANGECHCK(i, EVQ_SIZE_WIDTH);
	__RANGECHCK(buf_base_id, EVQ_BUF_BASE_ID_WIDTH);
	__RANGECHCK(1, EVQ_EN_WIDTH);

	/* if !enable then only evq needs to be correct, although valid
	 * values need to be passed in for other arguments to prevent
	 * assertions */

	val = ((i << EVQ_SIZE_LBN) | (buf_base_id << EVQ_BUF_BASE_ID_LBN) |
	       (enable ? (1 << EVQ_EN_LBN) : 0));

	EFHW_ASSERT(evq < FALCON_EVQ_TBL_NUM);

	offset = EVQ_PTR_TBL_CHAR_OFST;
	offset += evq * FALCON_REGISTER128;

	EFHW_TRACE("%s: evq %u en=%x:buf=%x:size=%x->%x at %lx",
		   __FUNCTION__, evq, enable, buf_base_id, evq_size, val,
		   offset);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + offset, val, FALCON_ATOMIC_PTR_TBL_REG);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);

	/* caller must wait for an update done event before writing any more
	   table entries */

	return;
}

void
falcon_nic_evq_ack(struct efhw_nic *nic,
		   uint evq,	/* evq id */
		   uint rptr,	/* new read pointer update */
		   bool wakeup	/* request a wakeup event if ptr's != */
    )
{
	uint val;
	ulong offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	EFHW_BUILD_ASSERT(FALCON_EVQ_CHAR == 4);

	__DWCHCK(EVQ_RPTR_LBN, EVQ_RPTR_WIDTH);
	__RANGECHCK(rptr, EVQ_RPTR_WIDTH);

	val = (rptr << EVQ_RPTR_LBN);

	EFHW_ASSERT(evq < FALCON_EVQ_TBL_NUM);

	if (evq < FALCON_EVQ_CHAR) {
		offset = EVQ_RPTR_REG_KER_OFST;
		offset += evq * FALCON_REGISTER128;

		EFHW_ASSERT(!wakeup);	/* don't try this at home */
	} else {
		offset = EVQ_RPTR_REG_OFST + (FALCON_EVQ_CHAR *
					      FALCON_REGISTER128);
		offset += (evq - FALCON_EVQ_CHAR) * FALCON_REGISTER128;

		/* nothing to do for interruptless event queues which do
		 * not want a wakeup */
		if (evq != FALCON_EVQ_CHAR && !wakeup)
			return;
	}

	EFHW_TRACE("%s: %x %x %x->%x", __FUNCTION__, evq, rptr, wakeup, val);

	writel(val, efhw_kva + offset);
	mmiowb();
}

/*----------------------------------------------------------------------------
 *
 * Helper for evq mapping
 *
 * idx = 0 && char   => hw eventq[4]
 * idx = 0 && net    => hw eventq[0]
 *   0 < idx < 5     => hw eventq[idx]  (5 is non-interrupting)
 *
 *
 *---------------------------------------------------------------------------*/

int falcon_idx_to_evq(struct efhw_nic *nic, uint idx)
{
	EFHW_BUILD_ASSERT(FALCON_EVQ_CHAR == 4);
	EFHW_ASSERT(idx <= FALCON_EVQ_NONIRQ);
	return (idx > 0) ? idx : FALCON_EVQ_CHAR;
}

static inline int falcon_evq_is_interrupting(struct efhw_nic *nic, uint idx)
{
	EFHW_BUILD_ASSERT(FALCON_EVQ_CHAR == 4);
	EFHW_ASSERT(idx <= FALCON_EVQ_NONIRQ);

	/* only the first CHAR driver event queue is interrupting */
	return (idx == FALCON_EVQ_CHAR);
}

static inline void
falcon_drv_ev(struct efhw_nic *nic, uint64_t data, uint qid)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	/* send an event from one driver to the other */
	EFHW_BUILD_ASSERT(DRV_EV_REG_KER_OFST == DRV_EV_REG_OFST);
	EFHW_BUILD_ASSERT(DRV_EV_DATA_LBN == 0);
	EFHW_BUILD_ASSERT(DRV_EV_DATA_WIDTH == 64);
	EFHW_BUILD_ASSERT(DRV_EV_QID_LBN == 64);
	EFHW_BUILD_ASSERT(DRV_EV_QID_WIDTH == 12);

	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + DRV_EV_REG_OFST, data, qid);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

_DEBUG_SYM_ void
falcon_timer_cmd(struct efhw_nic *nic,
		 uint evq,	/* timer id */
		 uint mode,	/* mode bits */
		 uint countdown /* counting value to set */ )
{
	FALCON_LOCK_DECL;
	uint val;
	ulong offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	EFHW_BUILD_ASSERT(TIMER_VAL_LBN == 0);

	__DWCHCK(TIMER_MODE_LBN, TIMER_MODE_WIDTH);
	__DWCHCK(TIMER_VAL_LBN, TIMER_VAL_WIDTH);

	__RANGECHCK(mode, TIMER_MODE_WIDTH);
	__RANGECHCK(countdown, TIMER_VAL_WIDTH);

	val = ((mode << TIMER_MODE_LBN) | (countdown << TIMER_VAL_LBN));

	if (evq < FALCON_EVQ_CHAR) {
		offset = TIMER_CMD_REG_KER_OFST;
		offset += evq * EFHW_8K;	/* PAGE mapped register */
	} else {
		offset = TIMER_TBL_OFST;
		offset += evq * FALCON_REGISTER128;
	}
	EFHW_ASSERT(evq < FALCON_EVQ_TBL_NUM);

	EFHW_TRACE("%s: evq %u mode %x (%s) time %x -> %08x",
		   __FUNCTION__, evq, mode,
		   mode == 0 ? "DISABLE" :
		   mode == 1 ? "IMMED" :
		   mode == 2 ? (evq < 5 ? "HOLDOFF" : "RX_TRIG") :
		   "<BAD>", countdown, val);

	/* Falcon requires 128 bit atomic access for this register when
	 * accessed from the driver. User access to timers is paged mapped
	 */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + offset, val, FALCON_ATOMIC_TIMER_CMD_REG);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
	return;
}

/*--------------------------------------------------------------------
 *
 * Rate pacing - Low level interface
 *
 *--------------------------------------------------------------------*/
void falcon_nic_pace(struct efhw_nic *nic, uint dmaq, uint pace)
{
	/* Pace specified in 2^(units of microseconds). This is the minimum
	   additional delay imposed over and above the IPG.

	   Pacing only available on the virtual interfaces
	 */
	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	ulong offset;

	if (pace > 20)
		pace = 20;	/* maxm supported value */

	__DWCHCK(TX_PACE_LBN, TX_PACE_WIDTH);
	__RANGECHCK(pace, TX_PACE_WIDTH);

	switch (nic->devtype.variant) {
	case 'A':
		EFHW_ASSERT(dmaq >= TX_PACE_TBL_FIRST_QUEUE_A1);
		offset = TX_PACE_TBL_A1_OFST;
		offset += (dmaq - TX_PACE_TBL_FIRST_QUEUE_A1) * 16;
		break;
	case 'B':
		/* Would be nice to assert this, but as dmaq is unsigned and
		 * TX_PACE_TBL_FIRST_QUEUE_B0 is 0, it makes no sense
		 * EFHW_ASSERT(dmaq >= TX_PACE_TBL_FIRST_QUEUE_B0);
		 */
		offset = TX_PACE_TBL_B0_OFST;
		offset += (dmaq - TX_PACE_TBL_FIRST_QUEUE_B0) * 16;
		break;
	default:
		EFHW_ASSERT(0);
		offset = 0;
		break;
	}

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + offset, pace, FALCON_ATOMIC_PACE_REG);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);

	EFHW_TRACE("%s: txq %d offset=%lx pace=2^%x",
		   __FUNCTION__, dmaq, offset, pace);
}

/*--------------------------------------------------------------------
 *
 * Interrupt - Low level interface
 *
 *--------------------------------------------------------------------*/

static void falcon_nic_handle_fatal_int(struct efhw_nic *nic)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	uint64_t val;

	offset = (efhw_kva + FATAL_INTR_REG_OFST);

	/* Falcon requires 32 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	val = readl(offset);
	FALCON_LOCK_UNLOCK(nic);

	/* ?? BUG3249 - need to disable illegal address interrupt */
	/* ?? BUG3114 - need to backport interrupt storm protection code */
	EFHW_ERR("fatal interrupt: %s%s%s%s%s%s%s%s%s%s%s%s[%" PRIx64 "]",
		 val & (1 << PCI_BUSERR_INT_CHAR_LBN) ? "PCI-bus-error " : "",
		 val & (1 << SRAM_OOB_INT_CHAR_LBN) ? "SRAM-oob " : "",
		 val & (1 << BUFID_OOB_INT_CHAR_LBN) ? "bufid-oob " : "",
		 val & (1 << MEM_PERR_INT_CHAR_LBN) ? "int-parity " : "",
		 val & (1 << RBUF_OWN_INT_CHAR_LBN) ? "rx-bufid-own " : "",
		 val & (1 << TBUF_OWN_INT_CHAR_LBN) ? "tx-bufid-own " : "",
		 val & (1 << RDESCQ_OWN_INT_CHAR_LBN) ? "rx-desc-own " : "",
		 val & (1 << TDESCQ_OWN_INT_CHAR_LBN) ? "tx-desc-own " : "",
		 val & (1 << EVQ_OWN_INT_CHAR_LBN) ? "evq-own " : "",
		 val & (1 << EVFF_OFLO_INT_CHAR_LBN) ? "evq-fifo " : "",
		 val & (1 << ILL_ADR_INT_CHAR_LBN) ? "ill-addr " : "",
		 val & (1 << SRM_PERR_INT_CHAR_LBN) ? "sram-parity " : "", val);
}

static void falcon_nic_interrupt_hw_enable(struct efhw_nic *nic)
{
	FALCON_LOCK_DECL;
	uint val;
	efhw_ioaddr_t offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	EFHW_BUILD_ASSERT(DRV_INT_EN_CHAR_WIDTH == 1);

	if (nic->flags & NIC_FLAG_NO_INTERRUPT)
		return;

	offset = (efhw_kva + INT_EN_REG_CHAR_OFST);
	val = 1 << DRV_INT_EN_CHAR_LBN;

	EFHW_NOTICE("%s: %x -> %x", __FUNCTION__, (int)(offset - efhw_kva),
		    val);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(offset, val, FALCON_ATOMIC_INT_EN_REG);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

static void falcon_nic_interrupt_hw_disable(struct efhw_nic *nic)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	EFHW_BUILD_ASSERT(SRAM_PERR_INT_KER_WIDTH == 1);
	EFHW_BUILD_ASSERT(DRV_INT_EN_KER_LBN == 0);
	EFHW_BUILD_ASSERT(SRAM_PERR_INT_CHAR_WIDTH == 1);
	EFHW_BUILD_ASSERT(DRV_INT_EN_CHAR_LBN == 0);
	EFHW_BUILD_ASSERT(SRAM_PERR_INT_KER_LBN == SRAM_PERR_INT_CHAR_LBN);
	EFHW_BUILD_ASSERT(DRV_INT_EN_KER_LBN == DRV_INT_EN_CHAR_LBN);

	if (nic->flags & NIC_FLAG_NO_INTERRUPT)
		return;

	offset = (efhw_kva + INT_EN_REG_CHAR_OFST);

	EFHW_NOTICE("%s: %x -> 0", __FUNCTION__, (int)(offset - efhw_kva));

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(offset, 0, FALCON_ATOMIC_INT_EN_REG);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

#ifndef __ci_ul_driver__

static void falcon_nic_irq_addr_set(struct efhw_nic *nic, dma_addr_t dma_addr)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t offset;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);

	offset = (efhw_kva + INT_ADR_REG_CHAR_OFST);

	EFHW_NOTICE("%s: %x -> " DMA_ADDR_T_FMT, __FUNCTION__,
		    (int)(offset - efhw_kva), dma_addr);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(offset, dma_addr, FALCON_ATOMIC_INT_ADR_REG);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

#endif


/*--------------------------------------------------------------------
 *
 * RXDP - low level interface
 *
 *--------------------------------------------------------------------*/

void
falcon_nic_set_rx_usr_buf_size(struct efhw_nic *nic, int usr_buf_bytes)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	uint64_t val, val2, usr_buf_size = usr_buf_bytes / 32;
	int rubs_lbn, rubs_width, roec_lbn;

	EFHW_BUILD_ASSERT(RX_CFG_REG_OFST == RX_CFG_REG_KER_OFST);

	switch (nic->devtype.variant) {
	default:
		EFHW_ASSERT(0);
		/* Fall-through to avoid compiler warnings. */
	case 'A':
		rubs_lbn = RX_USR_BUF_SIZE_A1_LBN;
		rubs_width = RX_USR_BUF_SIZE_A1_WIDTH;
		roec_lbn = RX_OWNERR_CTL_A1_LBN;
		break;
	case 'B':
		rubs_lbn = RX_USR_BUF_SIZE_B0_LBN;
		rubs_width = RX_USR_BUF_SIZE_B0_WIDTH;
		roec_lbn = RX_OWNERR_CTL_B0_LBN;
		break;
	}

	__DWCHCK(rubs_lbn, rubs_width);
	__QWCHCK(roec_lbn, 1);
	__RANGECHCK(usr_buf_size, rubs_width);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_read_qq(efhw_kva + RX_CFG_REG_OFST, &val, &val2);

	val &= ~((__FALCON_MASK64(rubs_width)) << rubs_lbn);
	val |= (usr_buf_size << rubs_lbn);

	/* shouldn't be needed for a production driver */
	val |= ((uint64_t) 1 << roec_lbn);

	falcon_write_qq(efhw_kva + RX_CFG_REG_OFST, val, val2);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}
EXPORT_SYMBOL(falcon_nic_set_rx_usr_buf_size);

void
falcon_nic_rx_filter_ctl_get(struct efhw_nic *nic, uint32_t *tcp_full,
			     uint32_t *tcp_wild,
			     uint32_t *udp_full, uint32_t *udp_wild)
{
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	FALCON_LOCK_DECL;
	uint64_t val;

	FALCON_LOCK_LOCK(nic);
	falcon_read_q(efhw_kva + RX_FILTER_CTL_REG_OFST, &val);
	FALCON_LOCK_UNLOCK(nic);

	*tcp_full = (uint32_t)((val >> TCP_FULL_SRCH_LIMIT_LBN) &
			       (__FALCON_MASK64(TCP_FULL_SRCH_LIMIT_WIDTH)));

	*tcp_wild = (uint32_t)((val >> TCP_WILD_SRCH_LIMIT_LBN) &
			       (__FALCON_MASK64(TCP_WILD_SRCH_LIMIT_WIDTH)));

	*udp_full = (uint32_t)((val >> UDP_FULL_SRCH_LIMIT_LBN) &
			       (__FALCON_MASK64(UDP_FULL_SRCH_LIMIT_WIDTH)));

	*udp_wild = (uint32_t)((val >> UDP_WILD_SRCH_LIMIT_LBN) &
			       (__FALCON_MASK64(UDP_WILD_SRCH_LIMIT_WIDTH)));
}
EXPORT_SYMBOL(falcon_nic_rx_filter_ctl_get);

void
falcon_nic_rx_filter_ctl_set(struct efhw_nic *nic, uint32_t tcp_full,
			     uint32_t tcp_wild,
			     uint32_t udp_full, uint32_t udp_wild)
{
	uint64_t val, val2;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	FALCON_LOCK_DECL;

	EFHW_ASSERT(tcp_full < nic->filter_tbl_size);
	EFHW_ASSERT(tcp_wild < nic->filter_tbl_size);
	EFHW_ASSERT(udp_full < nic->filter_tbl_size);
	EFHW_ASSERT(udp_wild < nic->filter_tbl_size);

	/* until we implement a dynamic scaling of search limits we wish to
	 * maintain the same limits set up by default in the net driver
	 * when we initialize the char driver */
	tcp_full_srch_limit = tcp_full;
	tcp_wild_srch_limit = tcp_wild;
	udp_full_srch_limit = udp_full;
	udp_wild_srch_limit = udp_wild;

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_read_qq(efhw_kva + RX_FILTER_CTL_REG_OFST, &val, &val2);

	/* Search limits */
	val &= ~((__FALCON_MASK64(TCP_FULL_SRCH_LIMIT_WIDTH))
		 << TCP_FULL_SRCH_LIMIT_LBN);

	val |= ((uint64_t)tcp_full + RX_FILTER_CTL_SRCH_FUDGE_FULL)
	    << TCP_FULL_SRCH_LIMIT_LBN;

	val &= ~((__FALCON_MASK64(TCP_WILD_SRCH_LIMIT_WIDTH))
		 << TCP_WILD_SRCH_LIMIT_LBN);

	val |= ((uint64_t)tcp_wild + RX_FILTER_CTL_SRCH_FUDGE_WILD)
	    << TCP_WILD_SRCH_LIMIT_LBN;

	val &= ~((__FALCON_MASK64(UDP_FULL_SRCH_LIMIT_WIDTH))
		 << UDP_FULL_SRCH_LIMIT_LBN);

	val |= ((uint64_t)udp_full + RX_FILTER_CTL_SRCH_FUDGE_FULL)
	    << UDP_FULL_SRCH_LIMIT_LBN;

	val &= ~((__FALCON_MASK64(UDP_WILD_SRCH_LIMIT_WIDTH))
		 << UDP_WILD_SRCH_LIMIT_LBN);

	val |= ((uint64_t)udp_wild + RX_FILTER_CTL_SRCH_FUDGE_WILD)
	    << UDP_WILD_SRCH_LIMIT_LBN;

	falcon_write_qq(efhw_kva + RX_FILTER_CTL_REG_OFST, val, val2);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}
EXPORT_SYMBOL(falcon_nic_rx_filter_ctl_set);

/*--------------------------------------------------------------------
 *
 * TXDP - low level interface
 *
 *--------------------------------------------------------------------*/

_DEBUG_SYM_ void falcon_nic_tx_cfg(struct efhw_nic *nic, int unlocked)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	uint64_t val1, val2;

	EFHW_BUILD_ASSERT(TX_CFG_REG_OFST == TX_CFG_REG_KER_OFST);
	__DWCHCK(TX_OWNERR_CTL_LBN, TX_OWNERR_CTL_WIDTH);
	__DWCHCK(TX_NON_IP_DROP_DIS_LBN, TX_NON_IP_DROP_DIS_WIDTH);

	FALCON_LOCK_LOCK(nic);
	falcon_read_qq(efhw_kva + TX_CFG_REG_OFST, &val1, &val2);

	/* Will flag fatal interrupts on owner id errors. This should not be
	   on for production code because there is otherwise a denial of
	   serivce attack possible */
	val1 |= (1 << TX_OWNERR_CTL_LBN);

	/* Setup user queue TCP/UDP only packet security */
	if (unlocked)
		val1 |= (1 << TX_NON_IP_DROP_DIS_LBN);
	else
		val1 &= ~(1 << TX_NON_IP_DROP_DIS_LBN);

	falcon_write_qq(efhw_kva + TX_CFG_REG_OFST, val1, val2);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

/*--------------------------------------------------------------------
 *
 * Random thresholds - Low level interface (Would like these to be op
 * defaults wherever possible)
 *
 *--------------------------------------------------------------------*/

static void falcon_nic_pace_cfg(struct efhw_nic *nic)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	unsigned offset = 0;
	uint64_t val;

	val = 0xa81682;		/* !!!! */

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	switch (nic->devtype.variant) {
	case 'A':  offset = TX_PACE_REG_A1_OFST;  break;
	case 'B':  offset = TX_PACE_REG_B0_OFST;  break;
	default:   EFHW_ASSERT(0);                break;
	}
	falcon_write_qq(efhw_kva + offset, val, 0);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

/**********************************************************************
 * Supporting modules. ************************************************
 **********************************************************************/

/*--------------------------------------------------------------------
 *
 * Filter support
 *
 *--------------------------------------------------------------------*/

/*! \TODO this table should be per nic */
struct falcon_cached_ipfilter {
#if FALCON_FULL_FILTER_CACHE
	unsigned dmaq;
	unsigned saddr_le32;
	unsigned daddr_le32;
	unsigned sport_le16;
	unsigned dport_le16;
	unsigned tcp:1;
	unsigned full:1;
	unsigned rss_b0:1;
	unsigned scat_b0:1;
#endif
	unsigned addr_valid:1;

};


/* TODO: Dynamically allocate this and store in struct efhw_nic. */
static struct falcon_cached_ipfilter
    host_ipfilter_cache[EFHW_MAX_NR_DEVS][FALCON_FILTER_TBL_NUM];


static inline void host_ipfilter_cache_init(struct efhw_nic *nic)
{
	memset(host_ipfilter_cache[nic->index], 0,
	       sizeof(host_ipfilter_cache[0][0]) * nic->filter_tbl_size);
}

static inline int host_ipfilter_cache_active(struct efhw_nic *nic, uint idx)
{
	EFHW_ASSERT(nic->index < EFHW_MAX_NR_DEVS);
	EFHW_ASSERT(idx < nic->filter_tbl_size);

	return (host_ipfilter_cache[nic->index][idx].addr_valid);

}

static inline void host_ipfilter_cache_flush(struct efhw_nic *nic, uint idx)
{
	EFHW_ASSERT(nic->index < EFHW_MAX_NR_DEVS);
	EFHW_ASSERT(idx < nic->filter_tbl_size);

	memset(&host_ipfilter_cache[nic->index][idx], 0,
	       sizeof(struct falcon_cached_ipfilter));
	mmiowb();
}

static inline void
host_ipfilter_cache_set_addr(struct efhw_nic *nic, uint idx, uint dmaq,
			     unsigned tcp, unsigned full,
			     unsigned rss_b0, unsigned scat_b0,
			     unsigned saddr_le32, unsigned sport_le16,
			     unsigned daddr_le32, unsigned dport_le16)
{
	unsigned nic_i = nic->index;

	EFHW_ASSERT(nic_i < EFHW_MAX_NR_DEVS);
	EFHW_ASSERT(idx < nic->filter_tbl_size);
	EFHW_ASSERT(!host_ipfilter_cache[nic_i][idx].addr_valid);

	__RANGECHCK(sport_le16, SRC_TCP_DEST_UDP_1_WIDTH);
	__RANGECHCK(dport_le16, SRC_TCP_DEST_UDP_1_WIDTH);

#if FALCON_FULL_FILTER_CACHE
	host_ipfilter_cache[nic_i][idx].dmaq = dmaq;
	host_ipfilter_cache[nic_i][idx].saddr_le32 = saddr_le32;
	host_ipfilter_cache[nic_i][idx].daddr_le32 = daddr_le32;
	host_ipfilter_cache[nic_i][idx].sport_le16 = sport_le16;
	host_ipfilter_cache[nic_i][idx].dport_le16 = dport_le16;
	host_ipfilter_cache[nic_i][idx].tcp = tcp;
	host_ipfilter_cache[nic_i][idx].full = full;
	host_ipfilter_cache[nic_i][idx].rss_b0 = rss_b0;
	host_ipfilter_cache[nic_i][idx].scat_b0 = scat_b0;
#endif
	host_ipfilter_cache[nic_i][idx].addr_valid = 1;
	mmiowb();
}

#if FALCON_VERIFY_FILTERS
/* Check that all active filters still exist by reading from H/W */
static void _falcon_nic_ipfilter_sanity(struct efhw_nic *nic)
{
	unsigned i;
	struct falcon_cached_ipfilter *f;
	uint64_t q0_expect, q1_expect, q0_got, q1_got;

	for (i = 0; i < nic->filter_tbl_size; i++) {
		f = host_ipfilter_cache[nic->index] + i;
		if (!f->addr_valid)
			continue;

		_falcon_nic_ipfilter_build(nic, f->tcp, f->full,
					   f->rss_b0, f->scat_b0, i, f->dmaq,
					   f->saddr_le32, f->sport_le16,
					   f->daddr_le32, f->dport_le16,
					   &q0_expect, &q1_expect);

		falcon_read_qq(EFHW_KVA(nic) + RX_FILTER_TBL0_OFST +
			       i * 2 * FALCON_REGISTER128,
			       &q0_got, &q1_got);

		if ((q0_got != q0_expect) || (q1_got != q1_expect)) {
			falcon_write_qq(EFHW_KVA(nic) + 0x300,
					q0_got, q1_got);
			EFHW_ERR("ERROR: RX-filter[%d][%d] was "
				 "%"PRIx64":%" PRIx64" expected "
				 "%"PRIx64":%"PRIx64,
				 nic->index, i, q0_got, q1_got,
				 q0_expect, q1_expect);
		}
	}
}
#endif /* FALCON_VERIFY_FILTERS */

#if FALCON_FULL_FILTER_CACHE
static inline int
host_ipfilter_cache_check_not(uint nic, uint idx, int tcp, int full,
			      unsigned saddr_le32, unsigned sport_le16,
			      unsigned daddr_le32, unsigned dport_le16)
{
	return ((host_ipfilter_cache[nic][idx].saddr_le32 != saddr_le32) ||
		(host_ipfilter_cache[nic][idx].daddr_le32 != daddr_le32) ||
		(host_ipfilter_cache[nic][idx].sport_le16 != sport_le16) ||
		(host_ipfilter_cache[nic][idx].dport_le16 != dport_le16) ||
		(host_ipfilter_cache[nic][idx].tcp != tcp) ||
		(host_ipfilter_cache[nic][idx].full != full));
}
#endif

#define host_ipfilter_cache_saddr_le32(nic, idx) \
		host_ipfilter_cache[nic][idx].saddr_le32
#define host_ipfilter_cache_daddr_le32(nic, idx) \
		host_ipfilter_cache[nic][idx].daddr_le32
#define host_ipfilter_cache_sport_le16(nic, idx) \
		host_ipfilter_cache[nic][idx].sport_le16
#define host_ipfilter_cache_dport_le16(nic, idx) \
		host_ipfilter_cache[nic][idx].dport_le16
#define host_ipfilter_cache_tcp(nic, idx) \
		host_ipfilter_cache[nic][idx].tcp
#define host_ipfilter_cache_full(nic, idx) \
		host_ipfilter_cache[nic][idx].full

/**********************************************************************
 * Implementation of the HAL. ********************************************
 **********************************************************************/

/*----------------------------------------------------------------------------
 *
 * Initialisation and configuration discovery
 *
 *---------------------------------------------------------------------------*/

#ifdef __ci_ul_driver__

static int falcon_nic_init_irq_channel(struct efhw_nic *nic, int enable)
{
	EFHW_ERR("%s: not implemented for ul driver", __FUNCTION__);
	return -EOPNOTSUPP;
}

#else

static int falcon_nic_init_irq_channel(struct efhw_nic *nic, int enable)
{
	/* create a buffer for the irq channel */
	int rc;

	if (enable) {
		rc = efhw_iopage_alloc(nic, &nic->irq_iobuff);
		if (rc < 0)
			return rc;

		falcon_nic_irq_addr_set(nic,
				efhw_iopage_dma_addr(&nic->irq_iobuff));
	} else {
		if (efhw_iopage_is_valid(&nic->irq_iobuff))
			efhw_iopage_free(nic, &nic->irq_iobuff);

		efhw_iopage_mark_invalid(&nic->irq_iobuff);
		falcon_nic_irq_addr_set(nic, 0);
	}

	EFHW_TRACE("%s: " ci_dma_addr_fmt " %sable", __FUNCTION__,
		   efhw_iopage_dma_addr(&nic->irq_iobuff), enable ?
			"en" : "dis");

	return 0;
}

#endif

static void falcon_nic_close_hardware(struct efhw_nic *nic)
{
	/* check we are in possession of some hardware */
	if (!efhw_nic_have_hw(nic))
		return;

	falcon_nic_init_irq_channel(nic, 0);

	EFHW_NOTICE("%s:", __FUNCTION__);
}

#ifdef __ci_ul_driver__
extern
#else
static
#endif
int falcon_nic_get_mac_config(struct efhw_nic *nic)
{
	efhw_ioaddr_t efhw_kva = nic->bar_ioaddr;
	int is_mac_type_1g;
	uint32_t strap, altera;
	uint64_t rx_cfg, r;

	altera = readl(efhw_kva + ALTERA_BUILD_REG_OFST);
	strap = readl(efhw_kva + STRAP_REG_KER_OFST) & 0x7;

	switch (nic->devtype.variant) {
	case 'A':
		if ((altera & 0x0fff0000) == 0x1130000) {
			strap = 2;	/* FPGA - PCI-X 2G */
		} else if ((altera & 0x00ff0000) == 0x140000) {
			/* should be 114 */
			strap = 4;	/* FPGA - PCI-X 4G */
		} else if (strap < 2 || strap > 5) {
			EFHW_ERR("Invalid strap option %d altera_buid_ver=%x",
				 strap, altera);
			return -EINVAL;
		}
		is_mac_type_1g = (0 != (strap & 2));
		break;
	case 'B':
		/* Runtime check that the hardware and software agree about
		 * the size of the RXFIFO. Write binary 11 across the left
		 * most bit, and assert we get 1 back.
		 */
		r = 1LL << RX_TOEP_TCP_SUPPRESS_B0_LBN;
		r |= (r << 1);

		/* Save the original value */
		falcon_read_q(efhw_kva + RX_CFG_REG_OFST, &rx_cfg);

		/* Write and ready the dummy value */
		falcon_write_qq(efhw_kva + RX_CFG_REG_OFST, r, 0);
		falcon_read_q(efhw_kva + RX_CFG_REG_OFST, &r);

		/* Restore the original value */
		falcon_write_qq(efhw_kva + RX_CFG_REG_OFST, rx_cfg, 0);

		if (r != (1LL << RX_TOEP_TCP_SUPPRESS_B0_LBN)) {
			EFHW_ERR("The FPGA build (%x) RXFIFO size does not "
				 "match the software", altera);
			return -EINVAL;
		}
		is_mac_type_1g = (0 != (strap & 2));
#if FALCON_MAC_SET_TYPE_BY_SPEED
		/* Check the selected strap pins against the MAC speed -
		 * and adjust if necessary.
		 */
		{
			int speed;
			speed = readl(efhw_kva + MAC0_CTRL_REG_OFST) & 0x3;
			is_mac_type_1g = (speed <= 2);
		}
#endif
		break;
	default:
		EFHW_ASSERT(0);
		is_mac_type_1g = 0;
		break;
	}

	nic->fpga_version = altera;

	/* We can now set the MAC type correctly based on the strap pins. */
	if (is_mac_type_1g) {
		nic->flags &= ~NIC_FLAG_10G;
	} else {
		/* strap & 4 must be set according to checks above */
		nic->flags |= NIC_FLAG_10G;
	}
	EFHW_NOTICE("Board has %s MAC: strap=%d",
		    0 != (nic->flags & NIC_FLAG_10G) ? "10G" : "1G", strap);
	return 0;
}

static int
falcon_nic_init_hardware(struct efhw_nic *nic,
			 struct efhw_ev_handler *ev_handlers,
			 const uint8_t *mac_addr)
{
	int rc;

	/* header sanity checks */
	FALCON_ASSERT_VALID();

	rc = falcon_nic_get_mac_config(nic);
	if (rc < 0)
		return rc;

	/* Initialise supporting modules */
	falcon_nic_ipfilter_ctor(nic);

#if FALCON_USE_SHADOW_BUFFER_TABLE
	CI_ZERO_ARRAY(_falcon_buffer_table, FALCON_BUFFER_TBL_NUM);
#endif

	/* Initialise the top level hardware blocks */
	memcpy(nic->mac_addr, mac_addr, ETH_ALEN);

	EFHW_TRACE("%s:", __FUNCTION__);

	/* nic.c:efhw_nic_init marks all the interrupt units as unused.

	   ?? TODO we should be able to request the non-interrupting event
	   queue and the net driver's (for a net driver that is using libefhw)
	   additional RSS queues here.

	   Result would be that that net driver could call
	   nic.c:efhw_nic_allocate_common_hardware_resources() and that the
	   IFDEF FALCON's can be removed from
	   nic.c:efhw_nic_allocate_common_hardware_resources()
	 */
	nic->irq_unit[0] = INT_EN_REG_CHAR_OFST;

	/*****************************************************************
	 * The rest of this function deals with initialization of the NICs
	 * hardware (as opposed to the initialization of the
	 * struct efhw_nic data structure */

	/* char driver grabs SRM events onto the non interrupting
	 * event queue */
	falcon_nic_srm_upd_evq(nic, FALCON_EVQ_NONIRQ);

	/* RXDP tweaks */

	/* ?? bug2396 rx_cfg should be ok so long as the net driver
	 * always pushes buffers big enough for the link MTU */

	/* set the RX buffer cutoff size to be the same as PAGE_SIZE.
	 * Use this value when we think that there will be a lot of
	 * jumbo frames.
	 *
	 * The default value 1600 is useful when packets are small,
	 * but would means that jumbo frame RX queues would need more
	 * descriptors pushing */
	falcon_nic_set_rx_usr_buf_size(nic, FALCON_RX_USR_BUF_SIZE);

	/* TXDP tweaks */
	/* ?? bug2396 looks ok */
	falcon_nic_tx_cfg(nic, /*unlocked(for non-UDP/TCP)= */ 0);
	falcon_nic_pace_cfg(nic);

	/* ?? bug2396
	 * netdriver must load first or else must RMW this register */
	falcon_nic_rx_filter_ctl_set(nic, RX_FILTER_CTL_SRCH_LIMIT_TCP_FULL,
				     RX_FILTER_CTL_SRCH_LIMIT_TCP_WILD,
				     RX_FILTER_CTL_SRCH_LIMIT_UDP_FULL,
				     RX_FILTER_CTL_SRCH_LIMIT_UDP_WILD);

	if (!(nic->flags & NIC_FLAG_NO_INTERRUPT)) {
		rc = efhw_keventq_ctor(nic, FALCON_EVQ_CHAR, &nic->evq[0],
				       ev_handlers);
		if (rc < 0) {
			EFHW_ERR("%s: efhw_keventq_ctor() failed (%d) evq=%d",
				 __FUNCTION__, rc, FALCON_EVQ_CHAR);
			return rc;
		}
	}
	rc = efhw_keventq_ctor(nic, FALCON_EVQ_NONIRQ,
			       &nic->evq[FALCON_EVQ_NONIRQ], NULL);
	if (rc < 0) {
		EFHW_ERR("%s: efhw_keventq_ctor() failed (%d) evq=%d",
			 __FUNCTION__, rc, FALCON_EVQ_NONIRQ);
		return rc;
	}

	/* allocate IRQ channel */
	rc = falcon_nic_init_irq_channel(nic, 1);
	/* ignore failure at user-level for eftest */
	if ((rc < 0) && !(nic->options & NIC_OPT_EFTEST))
		return rc;

	return 0;
}

/*--------------------------------------------------------------------
 *
 * Interrupt
 *
 *--------------------------------------------------------------------*/

static void
falcon_nic_interrupt_enable(struct efhw_nic *nic, unsigned idx)
{
	int evq;

	if (idx || (nic->flags & NIC_FLAG_NO_INTERRUPT))
		return;

	/* Enable driver interrupts */
	EFHW_NOTICE("%s: enable master interrupt", __FUNCTION__);
	falcon_nic_interrupt_hw_enable(nic);

	/* An interrupting eventq must start of day ack its read pointer */
	evq = falcon_idx_to_evq(nic, idx);

	if (falcon_evq_is_interrupting(nic, evq)) {
		struct efhw_keventq *q = &nic->evq[idx];
		unsigned rdptr =
		    EFHW_EVENT_OFFSET(q, q, 1) / sizeof(efhw_event_t);
		falcon_nic_evq_ack(nic, evq, rdptr, false);
		EFHW_NOTICE("%s: ACK evq[%d]:%x", __FUNCTION__, evq, rdptr);
	}
}

static void falcon_nic_interrupt_disable(struct efhw_nic *nic, uint idx)
{
	/* NB. No need to check for NIC_FLAG_NO_INTERRUPT, as
	 ** falcon_nic_interrupt_hw_disable() will do it. */
	if (idx)
		return;
	falcon_nic_interrupt_hw_disable(nic);
}

static void
falcon_nic_set_interrupt_moderation(struct efhw_nic *nic, uint idx,
				    uint32_t val)
{
	falcon_timer_cmd(nic, falcon_idx_to_evq(nic, idx),
			 TIMER_MODE_INT_HLDOFF, val / 5);
}

static inline void legacy_irq_ack(struct efhw_nic *nic)
{
	EFHW_ASSERT(!(nic->flags & NIC_FLAG_NO_INTERRUPT));

	if (!(nic->flags & NIC_FLAG_MSI)) {
		writel(1, EFHW_KVA(nic) + INT_ACK_REG_CHAR_A1_OFST);
		mmiowb();
		/* ?? FIXME: We should be doing a read here to ensure IRQ is
		 * thoroughly acked before we return from ISR. */
	}
}

static int falcon_nic_interrupt(struct efhw_nic *nic)
{
	volatile uint32_t *syserr_ptr =
	    (uint32_t *) efhw_iopage_ptr(&nic->irq_iobuff);
	int handled = 0;
	int done_ack = 0;

	EFHW_ASSERT(!(nic->flags & NIC_FLAG_NO_INTERRUPT));
	EFHW_ASSERT(syserr_ptr);

	/* FIFO fill level interrupt - just log it. */
	if (unlikely(*(syserr_ptr + (DW0_OFST / 4)))) {
		EFHW_WARN("%s: *** FIFO *** %x", __FUNCTION__,
			  *(syserr_ptr + (DW0_OFST / 4)));
		*(syserr_ptr + (DW0_OFST / 4)) = 0;
		handled++;
	}

	/* Fatal interrupts. */
	if (unlikely(*(syserr_ptr + (DW2_OFST / 4)))) {
		*(syserr_ptr + (DW2_OFST / 4)) = 0;
		falcon_nic_handle_fatal_int(nic);
		handled++;
	}

	/* Event queue interrupt.  For legacy interrupts we have to check
	 * that the interrupt is for us, because it could be shared. */
	if (*(syserr_ptr + (DW1_OFST / 4))) {
		*(syserr_ptr + (DW1_OFST / 4)) = 0;
		/* ACK must come before callback to handler fn. */
		legacy_irq_ack(nic);
		done_ack = 1;
		handled++;
		if (nic->irq_handler)
			nic->irq_handler(nic, 0);
	}

	if (unlikely(!done_ack)) {
		if (!handled)
			/* Shared interrupt line (hopefully). */
			return 0;
		legacy_irq_ack(nic);
	}

	EFHW_TRACE("%s: handled %d", __FUNCTION__, handled);
	return 1;
}

/*--------------------------------------------------------------------
 *
 * Event Management - and SW event posting
 *
 *--------------------------------------------------------------------*/

static void
falcon_nic_event_queue_enable(struct efhw_nic *nic, uint evq, uint evq_size,
			      dma_addr_t q_base_addr,	/* not used */
			      uint buf_base_id)
{
	EFHW_ASSERT(nic);

	/*!\ TODO we can be more efficient if we know whether or not there
	 * is a timer attached */
	falcon_timer_cmd(nic, evq, 0 /* disable */ , 0);

	falcon_nic_evq_ptr_tbl(nic, evq, 1, buf_base_id, evq_size);
	EFHW_TRACE("%s: enable evq %u size %u", __FUNCTION__, evq, evq_size);
}

static void
falcon_nic_event_queue_disable(struct efhw_nic *nic, uint evq, int timer_only)
{
	EFHW_ASSERT(nic);

	/*!\ TODO we can be more efficient if we know whether or not there
	 * is a timer attached */
	falcon_timer_cmd(nic, evq, 0 /* disable */ , 0);

	if (!timer_only)
		falcon_nic_evq_ptr_tbl(nic, evq, 0, 0, 0);
	EFHW_TRACE("%s: disenable evq %u", __FUNCTION__, evq);
}

static void
falcon_nic_wakeup_request(struct efhw_nic *nic, dma_addr_t q_base_addr,
			  int next_i, int evq)
{
	EFHW_ASSERT(evq > FALCON_EVQ_CHAR);
	falcon_nic_evq_ack(nic, evq, next_i, true);
	EFHW_TRACE("%s: evq %d next_i %d", __FUNCTION__, evq, next_i);
}

static void falcon_nic_sw_event(struct efhw_nic *nic, int data, int evq)
{
	uint64_t ev_data = data;

	ev_data &= ~FALCON_EVENT_CODE_MASK;
	ev_data |= FALCON_EVENT_CODE_SW;

	falcon_drv_ev(nic, ev_data, evq);
	EFHW_NOTICE("%s: evq[%d]->%x", __FUNCTION__, evq, data);
}

/*--------------------------------------------------------------------
 *
 * Filter support - TODO vary the depth of the search
 *
 *--------------------------------------------------------------------*/

void
falcon_nic_ipfilter_ctor(struct efhw_nic *nic)
{
	if (nic->devtype.variant == 'B' && nic->fpga_version)
		nic->filter_tbl_size = 8 * 1024;
	else
		nic->filter_tbl_size = 16 * 1024;

	host_ipfilter_cache_init(nic);
}


static int
falcon_nic_ipfilter_set(struct efhw_nic *nic, int type, int *_filter_idx,
			int dmaq,
			unsigned saddr_be32, unsigned sport_be16,
			unsigned daddr_be32, unsigned dport_be16)
{
	FALCON_LOCK_DECL;
	int tcp;
	int full;
	int rss_b0;
	int scat_b0;
	int key, hash1, hash2, idx = -1;
	int k;
	int rc = 0;
	unsigned max_srch = -1;

	/* oh joy of joys .. maybe one day we'll optimise  */
	unsigned int saddr = ntohl(saddr_be32);
	unsigned int daddr = ntohl(daddr_be32);
	unsigned int sport = ntohs(sport_be16);
	unsigned int dport = ntohs(dport_be16);

	__RANGECHCK(sport, SRC_TCP_DEST_UDP_1_WIDTH);
	__RANGECHCK(dport, SRC_TCP_DEST_UDP_1_WIDTH);

	tcp = ((type & EFHW_IP_FILTER_TYPE_TCP_MASK) != 0) ? 1 : 0;
	full = ((type & EFHW_IP_FILTER_TYPE_FULL_MASK) != 0) ? 1 : 0;
	rss_b0 = ((type & EFHW_IP_FILTER_TYPE_RSS_B0_MASK) != 0) ? 1 : 0;
	scat_b0 = ((type & EFHW_IP_FILTER_TYPE_NOSCAT_B0_MASK) != 0) ? 0 : 1;
	if (tcp && full)
		max_srch = tcp_full_srch_limit;
	else if (tcp && !full)
		max_srch = tcp_wild_srch_limit;
	else if (!tcp && full)
		max_srch = udp_full_srch_limit;
	else if (!tcp && !full)
		max_srch = udp_wild_srch_limit;

	EFHW_TRACE("%s: %x tcp %d full %d max_srch=%d",
		   __FUNCTION__, type, tcp, full, max_srch);

	/* The second hash function is simply
	 * h2(key) = 13 LSB of (key * 2 -  1)
	 * And the index(k), or the filter table address for kth search is
	 * index(k) = 13 LSB of (h1(key) + k * h2(key))
	 */
	key = falcon_hash_get_key(saddr, sport, daddr, dport, tcp, full);
	hash1 = falcon_hash_function1(key, nic->filter_tbl_size);
	hash2 = falcon_hash_function2(key, nic->filter_tbl_size);

	/* Avoid race to claim a filter entry */
	FALCON_LOCK_LOCK(nic);

	for (k = 0; (unsigned)k < max_srch; k++) {
		idx = falcon_hash_iterator(hash1, hash2, k,
					   nic->filter_tbl_size);

		EFHW_TRACE("ipfilter_set[%d:%d:%d]: src=%x:%d dest=%x:%d %s",
			   *_filter_idx, idx, k,
			   saddr, sport, daddr, dport,
			   host_ipfilter_cache_active(nic, idx) ?
			   "Active" : "Clear");

		if (!host_ipfilter_cache_active(nic, idx))
			break;

#if FALCON_FULL_FILTER_CACHE
		/* Check that we are not duplicating the filter */
		if (!host_ipfilter_cache_check_not(nic->index, idx, tcp, full,
						   saddr, sport, daddr,
						   dport)) {
			EFHW_WARN("%s: ERROR: duplicate filter (disabling "
				  "interrupts)", __FUNCTION__);
			FALCON_LOCK_UNLOCK(nic);
			falcon_nic_interrupt_hw_disable(nic);
			return -EINVAL;
		}
#endif

	}
	if (k == max_srch) {
		rc = -EADDRINUSE;
		idx = -1;
		goto fail1;
	}

	EFHW_ASSERT(idx < (int)nic->filter_tbl_size);

	host_ipfilter_cache_set_addr(nic, idx, dmaq, tcp, full, rss_b0,
				     scat_b0, saddr, sport, daddr, dport);

	_falcon_nic_ipfilter_set(nic, tcp, full, rss_b0,
				 scat_b0, idx, dmaq,
				 saddr, sport, daddr, dport);

	*_filter_idx = idx;

	EFHW_TRACE("%s: filter %x rxq %d src " NIPQUAD_FMT
		   ":%d dest " NIPQUAD_FMT ":%d set in %d",
		   __FUNCTION__, idx, dmaq,
		   NIPQUAD(&saddr), sport, NIPQUAD(&daddr), dport, k);

fail1:
	FALCON_LOCK_UNLOCK(nic);
	return rc;
}

static void
falcon_nic_ipfilter_attach(struct efhw_nic *nic, int filter_idx, int dmaq_idx)
{
	/* Intentionally empty - Falcon attaches and sets the filter
	 * in filter_set */
	EFHW_TRACE("%s: attach filter %x with rxq %d - ignored",
		   __FUNCTION__, filter_idx, dmaq_idx);
}

static void falcon_nic_ipfilter_detach(struct efhw_nic *nic, int filter_idx)
{
	/* Intentionally empty - Falcon attaches and sets the filter
	 * in filter_clear */
	EFHW_TRACE("%s: detach filter %x from rxq - ignored",
		   __FUNCTION__, filter_idx);
}

static void falcon_nic_ipfilter_clear(struct efhw_nic *nic, int filter_idx)
{
	FALCON_LOCK_DECL;

	EFHW_TRACE("%s: filter %x", __FUNCTION__, filter_idx);

	/* In case the filter has already been freed */
	if (filter_idx == -1)
		return;

	FALCON_LOCK_LOCK(nic);

	/* if we flush a chained hash then all we need to do is zero it out */
	host_ipfilter_cache_flush(nic, filter_idx);
	_falcon_nic_ipfilter_clear(nic, filter_idx);

	FALCON_LOCK_UNLOCK(nic);
	return;
}

/*--------------------------------------------------------------------
 *
 * Buffer table - helpers
 *
 *--------------------------------------------------------------------*/

#define FALCON_LAZY_COMMIT_HWM (FALCON_BUFFER_UPD_MAX - 16)

/* Note re.:
 *  falcon_nic_buffer_table_lazy_commit(struct efhw_nic *nic)
 *  falcon_nic_buffer_table_update_poll(struct efhw_nic *nic)
 *  falcon_nic_buffer_table_confirm(struct efhw_nic *nic)
 * -- these are no-ops in the user-level driver because it would need to
 * coordinate with the real driver on the number of outstanding commits.
 *
 * An exception is made for eftest apps, which manage the hardware without
 * using the char driver.
 */

static inline void falcon_nic_buffer_table_lazy_commit(struct efhw_nic *nic)
{
#if defined(__ci_ul_driver__)
	if (!(nic->options & NIC_OPT_EFTEST))
		return;
#endif

	/* Do nothing if operating in synchronous mode. */
	if (!nic->irq_handler)
		return;
}

static inline void falcon_nic_buffer_table_update_poll(struct efhw_nic *nic)
{
	FALCON_LOCK_DECL;
	int count = 0, rc = 0;

#if defined(__ci_ul_driver__)
	if (!(nic->options & NIC_OPT_EFTEST))
		return;
#endif

	/* We can be called here early days */
	if (!nic->irq_handler)
		return;

	/* If we need to gather buffer update events then poll the
	   non-interrupting event queue */

	/* For each _buffer_table_commit there will be an update done
	   event. We don't keep track of how many buffers each commit has
	   committed, just make sure that all the expected events have been
	   gathered */
	FALCON_LOCK_LOCK(nic);

	EFHW_TRACE("%s: %d", __FUNCTION__, nic->buf_commit_outstanding);

	while (nic->buf_commit_outstanding > 0) {
		/* we're not expecting to handle any events that require
		 * upcalls into the core driver */
		struct efhw_ev_handler handler;
		memset(&handler, 0, sizeof(handler));
		nic->evq[FALCON_EVQ_NONIRQ].ev_handlers = &handler;
		rc = efhw_keventq_poll(nic, &nic->evq[FALCON_EVQ_NONIRQ]);
		nic->evq[FALCON_EVQ_NONIRQ].ev_handlers = NULL;

		if (rc < 0) {
			EFHW_ERR("%s: poll ERROR (%d:%d) ***** ",
				 __FUNCTION__, rc,
				 nic->buf_commit_outstanding);
			goto out;
		}

		FALCON_LOCK_UNLOCK(nic);

		if (count++)
			udelay(1);

		if (count > 1000) {
			EFHW_WARN("%s: poll Timeout ***** (%d)", __FUNCTION__,
				  nic->buf_commit_outstanding);
			nic->buf_commit_outstanding = 0;
			return;
		}
		FALCON_LOCK_LOCK(nic);
	}

out:
	FALCON_LOCK_UNLOCK(nic);
	return;
}

void falcon_nic_buffer_table_confirm(struct efhw_nic *nic)
{
	/* confirm buffer table updates - should be used for items where
	   loss of data would be unacceptable. E.g for the buffers that back
	   an event or DMA queue */
	FALCON_LOCK_DECL;

#if defined(__ci_ul_driver__)
	if (!(nic->options & NIC_OPT_EFTEST))
		return;
#endif

	/* Do nothing if operating in synchronous mode. */
	if (!nic->irq_handler)
		return;

	FALCON_LOCK_LOCK(nic);

	_falcon_nic_buffer_table_commit(nic);

	FALCON_LOCK_UNLOCK(nic);

	falcon_nic_buffer_table_update_poll(nic);
}

/*--------------------------------------------------------------------
 *
 * Buffer table - API
 *
 *--------------------------------------------------------------------*/

static void
falcon_nic_buffer_table_clear(struct efhw_nic *nic, int buffer_id, int num)
{
	FALCON_LOCK_DECL;
	FALCON_LOCK_LOCK(nic);
	_falcon_nic_buffer_table_clear(nic, buffer_id, num);
	FALCON_LOCK_UNLOCK(nic);
}

static void
falcon_nic_buffer_table_set(struct efhw_nic *nic, dma_addr_t dma_addr,
			    uint bufsz, uint region,
			    int own_id, int buffer_id)
{
	FALCON_LOCK_DECL;

	EFHW_ASSERT(region < FALCON_REGION_NUM);

	EFHW_ASSERT((bufsz == EFHW_4K) ||
		    (bufsz == EFHW_8K && FALCON_BUFFER_TABLE_FULL_MODE));

	falcon_nic_buffer_table_update_poll(nic);

	FALCON_LOCK_LOCK(nic);

	_falcon_nic_buffer_table_set(nic, dma_addr, bufsz, region, own_id,
				     buffer_id);

	falcon_nic_buffer_table_lazy_commit(nic);

	FALCON_LOCK_UNLOCK(nic);
}

void
falcon_nic_buffer_table_set_n(struct efhw_nic *nic, int buffer_id,
			      dma_addr_t dma_addr, uint bufsz, uint region,
			      int n_pages, int own_id)
{
	/* used to set up a contiguous range of buffers */
	FALCON_LOCK_DECL;

	EFHW_ASSERT(region < FALCON_REGION_NUM);

	EFHW_ASSERT((bufsz == EFHW_4K) ||
		    (bufsz == EFHW_8K && FALCON_BUFFER_TABLE_FULL_MODE));

	while (n_pages--) {

		falcon_nic_buffer_table_update_poll(nic);

		FALCON_LOCK_LOCK(nic);

		_falcon_nic_buffer_table_set(nic, dma_addr, bufsz, region,
					     own_id, buffer_id++);

		falcon_nic_buffer_table_lazy_commit(nic);

		FALCON_LOCK_UNLOCK(nic);

		dma_addr += bufsz;
	}
}

/*--------------------------------------------------------------------
 *
 * DMA Queues - mid level API
 *
 *--------------------------------------------------------------------*/

#if BUG5302_WORKAROUND

/* Tx queues can get stuck if the software write pointer is set to an index
 * beyond the configured size of the queue, such that they will not flush.
 * This code can be run before attempting a flush; it will detect the bogus
 * value and reset it.  This fixes most instances of this problem, although
 * sometimes it does not work, or we may not detect it in the first place,
 * if the out-of-range value was replaced by an in-range value earlier.
 * (In those cases we have to apply a bigger hammer later, if we see that
 * the queue is still not flushing.)
 */
static void
falcon_check_for_bogus_tx_dma_wptr(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	uint64_t val_low64, val_high64;
	uint64_t size, hwptr, swptr, val;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	ulong offset = falcon_dma_tx_q_offset(nic, dmaq);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_read_qq(efhw_kva + offset, &val_low64, &val_high64);
	FALCON_LOCK_UNLOCK(nic);

	size = (val_low64 >> TX_DESCQ_SIZE_LBN)
	    & __FALCON_MASK64(TX_DESCQ_SIZE_WIDTH);
	size = (1 << size) * 512;
	hwptr = (val_high64 >> __DW3(TX_DESCQ_HW_RPTR_LBN))
	    & __FALCON_MASK64(TX_DESCQ_HW_RPTR_WIDTH);
	swptr = (val_low64 >> TX_DESCQ_SW_WPTR_LBN)
	    & __FALCON_MASK64(__LW2(TX_DESCQ_SW_WPTR_LBN));
	val = (val_high64)
	    &
	    __FALCON_MASK64(__DW3
			    (TX_DESCQ_SW_WPTR_LBN + TX_DESCQ_SW_WPTR_WIDTH));
	val = val << __LW2(TX_DESCQ_SW_WPTR_LBN);
	swptr = swptr | val;

	if (swptr >= size) {
		EFHW_WARN("Resetting bad write pointer for TXQ[%d]", dmaq);
		writel((uint32_t) ((hwptr + 0) & (size - 1)),
		       efhw_kva + falcon_tx_dma_page_addr(dmaq) + 12);
		mmiowb();
	}
}

/* Here's that "bigger hammer": we reset all the pointers (hardware read,
 * hardware descriptor cache read, software write) to zero.
 */
void falcon_clobber_tx_dma_ptrs(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	uint64_t val_low64, val_high64;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	ulong offset = falcon_dma_tx_q_offset(nic, dmaq);

	EFHW_WARN("Recovering stuck TXQ[%d]", dmaq);
	FALCON_LOCK_LOCK(nic);
	falcon_read_qq(efhw_kva + offset, &val_low64, &val_high64);
	val_high64 &= ~(__FALCON_MASK64(TX_DESCQ_HW_RPTR_WIDTH)
			<< __DW3(TX_DESCQ_HW_RPTR_LBN));
	val_high64 &= ~(__FALCON_MASK64(TX_DC_HW_RPTR_WIDTH)
			<< __DW3(TX_DC_HW_RPTR_LBN));
	falcon_write_qq(efhw_kva + offset, val_low64, val_high64);
	mmiowb();
	writel(0, efhw_kva + falcon_tx_dma_page_addr(dmaq) + 12);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
}

#endif

static inline int
__falcon_really_flush_tx_dma_channel(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	uint val;

	EFHW_BUILD_ASSERT(TX_FLUSH_DESCQ_REG_KER_OFST ==
			  TX_FLUSH_DESCQ_REG_OFST);

	__DWCHCK(TX_FLUSH_DESCQ_CMD_LBN, TX_FLUSH_DESCQ_CMD_WIDTH);
	__DWCHCK(TX_FLUSH_DESCQ_LBN, TX_FLUSH_DESCQ_WIDTH);
	__RANGECHCK(dmaq, TX_FLUSH_DESCQ_WIDTH);

	val = ((1 << TX_FLUSH_DESCQ_CMD_LBN) | (dmaq << TX_FLUSH_DESCQ_LBN));

	EFHW_TRACE("TX DMA flush[%d]", dmaq);

#if BUG5302_WORKAROUND
	falcon_check_for_bogus_tx_dma_wptr(nic, dmaq);
#endif

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + TX_FLUSH_DESCQ_REG_OFST,
			val, FALCON_ATOMIC_TX_FLUSH_DESCQ);

	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
	return 0;
}

static inline int
__falcon_is_tx_dma_channel_flushed(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	uint64_t val_low64, val_high64;
	uint64_t enable, flush_pending;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	ulong offset = falcon_dma_tx_q_offset(nic, dmaq);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_read_qq(efhw_kva + offset, &val_low64, &val_high64);
	FALCON_LOCK_UNLOCK(nic);

	/* should see one of three values for these 2 bits
	 *   1, queue enabled no flush pending
	 *	- i.e. first flush request
	 *   2, queue enabled, flush pending
	 *	- i.e. request to reflush before flush finished
	 *   3, queue disabled (no flush pending)
	 *	- flush complete
	 */
	__DWCHCK(TX_DESCQ_FLUSH_LBN, TX_DESCQ_FLUSH_WIDTH);
	__DW3CHCK(TX_DESCQ_EN_LBN, TX_DESCQ_EN_WIDTH);
	enable = val_high64 & (1 << __DW3(TX_DESCQ_EN_LBN));
	flush_pending = val_low64 & (1 << TX_DESCQ_FLUSH_LBN);

	if (enable && !flush_pending)
		return 0;

	EFHW_TRACE("%d, %s: %s, %sflush pending", dmaq, __FUNCTION__,
		   enable ? "enabled" : "disabled",
		   flush_pending ? "" : "NO ");
	/* still in progress */
	if (enable && flush_pending)
		return -EALREADY;

	return -EAGAIN;
}

static int falcon_flush_tx_dma_channel(struct efhw_nic *nic, uint dmaq)
{
	int rc;
	rc = __falcon_is_tx_dma_channel_flushed(nic, dmaq);
	if (rc < 0) {
		EFHW_WARN("%s: failed %d", __FUNCTION__, rc);
		return rc;
	}
	return __falcon_really_flush_tx_dma_channel(nic, dmaq);
}

static int
__falcon_really_flush_rx_dma_channel(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	uint val;

	EFHW_BUILD_ASSERT(RX_FLUSH_DESCQ_REG_KER_OFST ==
			  RX_FLUSH_DESCQ_REG_OFST);

	__DWCHCK(RX_FLUSH_DESCQ_CMD_LBN, RX_FLUSH_DESCQ_CMD_WIDTH);
	__DWCHCK(RX_FLUSH_DESCQ_LBN, RX_FLUSH_DESCQ_WIDTH);
	__RANGECHCK(dmaq, RX_FLUSH_DESCQ_WIDTH);

	val = ((1 << RX_FLUSH_DESCQ_CMD_LBN) | (dmaq << RX_FLUSH_DESCQ_LBN));

	EFHW_TRACE("RX DMA flush[%d]", dmaq);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_write_qq(efhw_kva + RX_FLUSH_DESCQ_REG_OFST, val,
			FALCON_ATOMIC_RX_FLUSH_DESCQ);
	mmiowb();
	FALCON_LOCK_UNLOCK(nic);
	return 0;
}

static inline int
__falcon_is_rx_dma_channel_flushed(struct efhw_nic *nic, uint dmaq)
{
	FALCON_LOCK_DECL;
	uint64_t val;
	efhw_ioaddr_t efhw_kva = EFHW_KVA(nic);
	ulong offset = falcon_dma_rx_q_offset(nic, dmaq);

	/* Falcon requires 128 bit atomic access for this register */
	FALCON_LOCK_LOCK(nic);
	falcon_read_q(efhw_kva + offset, &val);
	FALCON_LOCK_UNLOCK(nic);

	__DWCHCK(RX_DESCQ_EN_LBN, RX_DESCQ_EN_WIDTH);

	/* is it enabled? */
	return (val & (1 << RX_DESCQ_EN_LBN))
	    ? 0 : -EAGAIN;
}

static int falcon_flush_rx_dma_channel(struct efhw_nic *nic, uint dmaq)
{
	int rc;
	rc = __falcon_is_rx_dma_channel_flushed(nic, dmaq);
	if (rc < 0) {
		EFHW_ERR("%s: failed %d", __FUNCTION__, rc);
		return rc;
	}
	return __falcon_really_flush_rx_dma_channel(nic, dmaq);
}

/*--------------------------------------------------------------------
 *
 * Falcon specific event callbacks
 *
 *--------------------------------------------------------------------*/

int
falcon_handle_char_event(struct efhw_nic *nic, struct efhw_ev_handler *h,
			 efhw_event_t *ev)
{
	EFHW_TRACE("DRIVER EVENT: "FALCON_EVENT_FMT,
		   FALCON_EVENT_PRI_ARG(*ev));

	switch (FALCON_EVENT_DRIVER_SUBCODE(ev)) {

	case TX_DESCQ_FLS_DONE_EV_DECODE:
		EFHW_TRACE("TX[%d] flushed",
			   (int)FALCON_EVENT_TX_FLUSH_Q_ID(ev));
#if !defined(__ci_ul_driver__)
		efhw_handle_txdmaq_flushed(nic, h, ev);
#endif
		break;

	case RX_DESCQ_FLS_DONE_EV_DECODE:
		EFHW_TRACE("RX[%d] flushed",
			   (int)FALCON_EVENT_TX_FLUSH_Q_ID(ev));
#if !defined(__ci_ul_driver__)
		efhw_handle_rxdmaq_flushed(nic, h, ev);
#endif
		break;

	case SRM_UPD_DONE_EV_DECODE:
		nic->buf_commit_outstanding =
		    max(0, nic->buf_commit_outstanding - 1);
		EFHW_TRACE("COMMIT DONE %d", nic->buf_commit_outstanding);
		break;

	case EVQ_INIT_DONE_EV_DECODE:
		EFHW_TRACE("EVQ INIT");
		break;

	case WAKE_UP_EV_DECODE:
		EFHW_TRACE("WAKE UP");
		efhw_handle_wakeup_event(nic, h, ev);
		break;

	case TIMER_EV_DECODE:
		EFHW_TRACE("TIMER");
		efhw_handle_timeout_event(nic, h, ev);
		break;

	case RX_DESCQ_FLSFF_OVFL_EV_DECODE:
		/* This shouldn't happen. */
		EFHW_ERR("%s: RX flush fifo overflowed", __FUNCTION__);
		return -EINVAL;

	default:
		EFHW_TRACE("UNKOWN DRIVER EVENT: " FALCON_EVENT_FMT,
			   FALCON_EVENT_PRI_ARG(*ev));
		break;
	}
	return 0;
}

/*--------------------------------------------------------------------
 *
 * Abstraction Layer Hooks
 *
 *--------------------------------------------------------------------*/

struct efhw_func_ops falcon_char_functional_units = {
	falcon_nic_close_hardware,
	falcon_nic_init_hardware,
	falcon_nic_interrupt,
	falcon_nic_interrupt_enable,
	falcon_nic_interrupt_disable,
	falcon_nic_set_interrupt_moderation,
	falcon_nic_event_queue_enable,
	falcon_nic_event_queue_disable,
	falcon_nic_wakeup_request,
	falcon_nic_sw_event,
	falcon_nic_ipfilter_set,
	falcon_nic_ipfilter_attach,
	falcon_nic_ipfilter_detach,
	falcon_nic_ipfilter_clear,
	falcon_dmaq_tx_q_init,
	falcon_dmaq_rx_q_init,
	falcon_dmaq_tx_q_disable,
	falcon_dmaq_rx_q_disable,
	falcon_flush_tx_dma_channel,
	falcon_flush_rx_dma_channel,
	falcon_nic_buffer_table_set,
	falcon_nic_buffer_table_set_n,
	falcon_nic_buffer_table_clear,
	falcon_nic_buffer_table_commit,
};
