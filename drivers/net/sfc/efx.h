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

#ifndef EFX_EFX_H
#define EFX_EFX_H

#include "net_driver.h"

/* PCI IDs */
#define EFX_VENDID_SFC	        0x1924
#define FALCON_A_P_DEVID	0x0703
#define FALCON_A_S_DEVID        0x6703
#define FALCON_B_P_DEVID        0x0710

/* TX */
extern int efx_xmit(struct efx_nic *efx,
		    struct efx_tx_queue *tx_queue, struct sk_buff *skb);
extern void efx_stop_queue(struct efx_nic *efx);
extern void efx_wake_queue(struct efx_nic *efx);

/* RX */
#if defined(EFX_USE_FASTCALL)
extern void fastcall efx_xmit_done(struct efx_tx_queue *tx_queue,
				   unsigned int index);
#else
extern void efx_xmit_done(struct efx_tx_queue *tx_queue, unsigned int index);
#endif
#if defined(EFX_USE_FASTCALL)
extern void fastcall efx_rx_packet(struct efx_rx_queue *rx_queue,
				   unsigned int index, unsigned int len,
				   int checksummed, int discard);
#else
extern void efx_rx_packet(struct efx_rx_queue *rx_queue, unsigned int index,
			  unsigned int len, int checksummed, int discard);
#endif
extern void efx_fini_rx_buffer(struct efx_rx_queue *rx_queue,
			       struct efx_rx_buffer *rx_buf);

/* Channels */
extern void efx_process_channel_now(struct efx_channel *channel);
extern int efx_flush_queues(struct efx_nic *efx);

/* Ports */
extern void efx_reconfigure_port(struct efx_nic *efx,
				 int on_disabled);

/* Global */
extern void efx_schedule_reset(struct efx_nic *efx, enum reset_type type);
extern void efx_suspend(struct efx_nic *efx);
extern void efx_resume(struct efx_nic *efx);
extern void efx_init_irq_moderation(struct efx_nic *efx, int tx_usecs,
				    int rx_usecs);
extern int efx_request_power(struct efx_nic *efx, int mw, const char *name);
extern void efx_hex_dump(const u8 *, unsigned int, const char *);

/* Dummy PHY ops for PHY drivers */
extern int efx_port_dummy_op_int(struct efx_nic *efx);
extern void efx_port_dummy_op_void(struct efx_nic *efx);
extern void efx_port_dummy_op_blink(struct efx_nic *efx, int blink);


extern unsigned int efx_monitor_interval;

static inline void efx_schedule_channel(struct efx_channel *channel)
{
	EFX_TRACE(channel->efx, "channel %d scheduling NAPI poll on CPU%d\n",
		  channel->channel, raw_smp_processor_id());
	channel->work_pending = 1;

#if defined(EFX_HAVE_OLD_NAPI)
	if (!test_and_set_bit(__LINK_STATE_RX_SCHED, &channel->napi_dev->state))
		__netif_rx_schedule(channel->napi_dev);
#else
	netif_rx_schedule(channel->napi_dev, &channel->napi_str);
#endif
}


#endif /* EFX_EFX_H */
