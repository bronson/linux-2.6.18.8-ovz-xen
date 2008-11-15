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

#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel_stat.h>
#include <linux/pci.h>
#include <linux/ethtool.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/udp.h>
#include <linux/rtnetlink.h>
#include <asm/io.h>
#include "net_driver.h"
#include "ethtool.h"
#include "efx.h"
#include "falcon.h"
#include "selftest.h"
#include "boards.h"
#include "workarounds.h"

/* Self tests */

/*
 * Loopback test packet structure
 *
 * The self-test should stress every RSS vector, and unfortunately
 * Falcon only performs RSS on TCP/UDP packets.
 */
struct efx_loopback_payload {
	struct ethhdr header;
	struct iphdr ip;
	struct udphdr udp;
	__be16 iteration;
	const char msg[64];
} __attribute__ ((packed));

/* Loopback test source MAC address */
static const unsigned char payload_source[ETH_ALEN] = {
	0x00, 0x0f, 0x53, 0x1b, 0x1b, 0x1b,
};

static const char *payload_msg =
	"Hello world! This is an Efx loopback test in progress!";

struct efx_selftest_state {
	/* Drop all packets in  efx_loopback_rx_packet */
	int flush;

	/* Number of packets being used in this test */
	int packet_count;

	/* RX good packet count */
	atomic_t rx_good;

	/* RX bad packet count */
	atomic_t rx_bad;

	/* Payload used in tests */
	struct efx_loopback_payload payload;
};

/**************************************************************************
 *
 * Configurable values
 *
 **************************************************************************/

/* Level of loopback testing
 *
 * The maximum packet burst length is 16**(n-1), i.e.
 *
 * - Level 0 : no packets
 * - Level 1 : 1 packet
 * - Level 2 : 17 packets (1 * 1 packet, 1 * 16 packets)
 * - Level 3 : 273 packets (1 * 1 packet, 1 * 16 packet, 1 * 256 packets)
 *
 */
static unsigned int loopback_test_level = 3;

/**************************************************************************
 *
 * Interrupt and event queue testing
 *
 **************************************************************************/

/*
 * Test interrupts
 *
 * This generates a test interrupt and waits for it to be received by
 * a CPU.  This routine must be called from process context and will
 * sleep.
 */
static int efx_test_interrupts(struct efx_nic *efx,
			       struct efx_self_tests *tests)
{
	struct efx_channel *channel;
	unsigned long j_start;

	EFX_LOG(efx, "testing interrupts\n");
	tests->interrupt = -1;

	/* Reset interrupt flag */
	efx->last_irq_cpu = -1;
	smp_wmb();

	/* To guarantee that an interrupt is received make sure that the
	 * channels are ack'd at least once, reenabling interrupts.
	 * We will then receive an interrupt, either by explicitly
	 * requesting one, or receiving traffic.
	 */
	efx_for_each_channel_with_interrupt(channel, efx) {
		/* Process the eventq synchronously */
		if (channel->work_pending)
			efx_process_channel_now(channel);
		/* Check if we haven't already received an interrupt */
		if (efx->last_irq_cpu >= 0)
			goto success;
	}

	j_start = jiffies;

	/* Generate test interrupt. */
	falcon_generate_interrupt(efx);

	/* Wait for arrival of test interrupt. */
	EFX_LOG(efx, "waiting for test interrupt\n");
	schedule_timeout_uninterruptible(HZ / 10);
	if (efx->last_irq_cpu >= 0)
		goto success;

	EFX_ERR(efx, "timed out in %ld jiffies waiting for interrupt\n",
		jiffies - j_start);
	return -ETIMEDOUT;

 success:
	EFX_LOG(efx, "%s test interrupt seen on CPU%d\n", INT_MODE(efx),
		efx->last_irq_cpu);
	tests->interrupt = 1;
	return 0;
}

/*
 * Test capability of events to receive events
 * This generates a test event and waits for it to be processed
 * This routine must be called from process context and will sleep.
 */
static int efx_test_eventq(struct efx_channel *channel,
			   struct efx_self_tests *tests)
{
	unsigned int magic;

	/* Channel specific code, limited to 20 bits */
	magic = (0x00010150 + channel->channel);
	EFX_LOG(channel->efx, "channel %d testing event queue with code %x\n",
		channel->channel, magic);

	tests->eventq_dma[channel->channel] = -1;
	tests->eventq_int[channel->channel] = 1;	/* fake pass */
	tests->eventq_poll[channel->channel] = 1;	/* fake pass */

	/* Reset flag and zero magic word */
	channel->efx->last_irq_cpu = -1;
	channel->eventq_magic = 0;
	smp_wmb();

	/* Generate test event */
	falcon_generate_test_event(channel, magic);
	udelay(1);

	efx_process_channel_now(channel);
	if (channel->eventq_magic != magic) {
		EFX_ERR(channel->efx, "channel %d  failed to see test event\n",
			channel->channel);
		return -ETIMEDOUT;
	} else {
		tests->eventq_dma[channel->channel] = 1;
	}

	return 0;
}


/*
 * Test capability of events to generate interrupts
 * This generates a test event and waits for it to be processed by an
 * ISR.  This routine must be called from process context and will
 * sleep.
 */
static int efx_test_eventq_irq(struct efx_channel *channel,
			       struct efx_self_tests *tests)
{
	unsigned int magic, count;
	unsigned long j_start = jiffies;

	/* Channel specific code, limited to 20 bits */
	magic = (0x00010150 + channel->channel);
	EFX_LOG(channel->efx, "channel %d testing event queue with code %x\n",
		channel->channel, magic);

	tests->eventq_dma[channel->channel] = -1;
	tests->eventq_int[channel->channel] = -1;
	tests->eventq_poll[channel->channel] = -1;

	/* Reset flag and zero magic word */
	channel->efx->last_irq_cpu = -1;
	channel->eventq_magic = 0;
	smp_wmb();

	/* Generate test event */
	falcon_generate_test_event(channel, magic);

	/* Wait for arrival of interrupt */
	count = 0;
	do {
		schedule_timeout_uninterruptible(HZ / 100);

		if (channel->work_pending)
			efx_process_channel_now(channel);

		if (channel->eventq_magic == magic)
			goto eventq_ok;
	}
	while (++count < 2);

	EFX_ERR(channel->efx, "channel %d timed out in %ld jiffies waiting for"
		" event queue\n", channel->channel, jiffies - j_start);

	/* See if interrupt arrived */
	if (channel->efx->last_irq_cpu >= 0) {
		EFX_ERR(channel->efx, "channel %d saw interrupt on CPU%d "
			"during event queue test\n", channel->channel,
			raw_smp_processor_id());
		tests->eventq_int[channel->channel] = 1;
	}

	/* Check to see if event was received even if interrupt wasn't */
	efx_process_channel_now(channel);
	if (channel->eventq_magic == magic) {
		EFX_ERR(channel->efx, "channel %d event was generated, but "
			"failed to trigger an interrupt\n", channel->channel);
		tests->eventq_dma[channel->channel] = 1;
	}

	return -ETIMEDOUT;
 eventq_ok:
	EFX_LOG(channel->efx, "channel %d event queue passed\n",
		channel->channel);
	tests->eventq_dma[channel->channel] = 1;
	tests->eventq_int[channel->channel] = 1;
	tests->eventq_poll[channel->channel] = 1;
	return 0;
}

/**************************************************************************
 *
 * PHY testing
 *
 **************************************************************************/

/*
 * Check PHY presence
 * This reads the PHY ID registers via GMII and checks that neither
 * are all-zeroes or all-ones (indicating a nonexistent or
 * uncommunicative PHY).
 */
static int efx_test_phy(struct efx_nic *efx,
			struct efx_self_tests *tests)
{
	u16 physid1, physid2;
	struct mii_if_info *mii = &efx->mii;
	struct net_device *net_dev = efx->net_dev;

	if (efx->phy_type == PHY_TYPE_NONE)
		return 0;

	EFX_LOG(efx, "testing PHY presence\n");
	tests->phy_ok = -1;

	physid1 = mii->mdio_read(net_dev, mii->phy_id, MII_PHYSID1);
	physid2 = mii->mdio_read(net_dev, mii->phy_id, MII_PHYSID2);

	if ((physid1 != 0x0000) && (physid1 != 0xffff) &&
	    (physid2 != 0x0000) && (physid2 != 0xffff)) {
		EFX_LOG(efx, "found MII PHY %d ID 0x%x:%x\n",
			mii->phy_id, physid1, physid2);
		tests->phy_ok = 1;
		return 0;
	}

	EFX_ERR(efx, "no MII PHY present with ID %d\n", mii->phy_id);
	return -ENODEV;
}

/**************************************************************************
 *
 * Loopback testing
 * NB Only one loopback test can be executing concurrently.
 *
 **************************************************************************/

/* Loopback test RX callback
 * This is called for each received packet during loopback testing.
 */
void efx_loopback_rx_packet(struct efx_nic *efx,
			    const char *buf_ptr, int pkt_len)
{
	struct efx_selftest_state *state = efx->loopback_selftest;
	struct efx_loopback_payload *received;
	struct efx_loopback_payload *payload;

	BUG_ON(!buf_ptr);

	/* If we are just flushing, then drop the packet */
	if ((state == NULL) || state->flush)
		return;

	payload = &state->payload;

	/* The packet should have been passed up to us before any LRO/SSR, so
	 * we should be able to compare the data directly. Since the packet
	 * is going to be thrown away by the caller, modify it in place.
	 * efx_test_loopback guarantees to not touch state->payload during
	 * the test */
	received = (struct efx_loopback_payload *)(char *)buf_ptr;
	received->ip.saddr = payload->ip.saddr;
	received->ip.check = payload->ip.check;

	/* Check that header exists */
	if (pkt_len < sizeof(received->header)) {
		EFX_ERR(efx, "saw runt RX packet (length %d) "
			"in %s loopback test\n", pkt_len,
			LOOPBACK_MODE(efx));
		goto err;
	}

	/* Check that header matches */
	if (memcmp(&received->header, &payload->header, ETH_HLEN) != 0) {
		EFX_ERR(efx, "saw non-loopback RX packet in"
			" %s loopback test\n",
			LOOPBACK_MODE(efx));
		goto err;
	}

	/* Check packet length */
	if (pkt_len != sizeof(*payload)) {
		EFX_ERR(efx, "saw incorrect RX packet length"
			" %d (wanted %d) in %s loopback test\n",
			pkt_len, (int)sizeof(*payload),
			LOOPBACK_MODE(efx));
		goto err;
	}

	/* Check that IP header matches */
	if (memcmp(&received->ip, &payload->ip, sizeof(payload->ip)) != 0) {
		EFX_ERR(efx, "saw corrupted IP header in %s "
			"loopback test\n",
			LOOPBACK_MODE(efx));
		goto err;
	}

	/* Check that msg and padding matches */
	if (memcmp(&received->msg, &payload->msg, sizeof(received->msg)) != 0) {
		EFX_ERR(efx, "saw corrupted RX packet in %s "
			"loopback test\n",
			LOOPBACK_MODE(efx));
		goto err;
	}

	/* Check that iteration matches */
	if (received->iteration != payload->iteration) {
		EFX_ERR(efx, "saw RX packet from iteration %d"
			" (wanted %d) in %s loopback test\n",
			ntohs(received->iteration), ntohs(payload->iteration),
			LOOPBACK_MODE(efx));
		goto err;
	}

	/* Increase correct RX count */
	EFX_TRACE(efx, "got loopback RX in %s loopback test\n",
		  LOOPBACK_MODE(efx));

	atomic_inc(&state->rx_good);
	return;

 err:
#ifdef EFX_ENABLE_DEBUG
	if (atomic_read(&state->rx_bad) == 0) {
		EFX_ERR(efx, "received packet:\n");
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 0x10, 1,
			       buf_ptr, pkt_len, 0);
		EFX_ERR(efx, "expected packet:\n");
		print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 0x10, 1,
			       &state->payload, sizeof(state->payload), 0);
	}
#endif
	atomic_inc(&state->rx_bad);
}

/* Initialise an efx_selftest_state for a new iteration */
static void efx_iterate_state(struct efx_nic *efx)
{
	struct efx_selftest_state *state = efx->loopback_selftest;
	struct net_device *net_dev = efx->net_dev;
	struct efx_loopback_payload *payload = &state->payload;

	/* Initialise the layerII header */
	memcpy(&payload->header.h_dest, net_dev->dev_addr, ETH_ALEN);
	memcpy(&payload->header.h_source, &payload_source, ETH_ALEN);
	payload->header.h_proto = htons(ETH_P_IP);

	/* saddr set later and used as incrementing count */
	payload->ip.daddr = htonl(INADDR_LOOPBACK);
	payload->ip.ihl = 5;
	payload->ip.check = 0;		/* offloaded */
	payload->ip.tot_len = htons(sizeof(*payload) - sizeof(struct ethhdr));
	payload->ip.version = IPVERSION;
	payload->ip.protocol = IPPROTO_UDP;

	/* Initialise udp header */
	payload->udp.source = 0;
	payload->udp.len = htons(sizeof(*payload) - sizeof(struct ethhdr) -
				 sizeof(struct iphdr));
	payload->udp.check = 0;	/* checksum ignored */

	/* Fill out payload */
	payload->iteration = htons(ntohs(payload->iteration) + 1);
	memcpy(&payload->msg, payload_msg, sizeof(payload_msg));

	/* Fill out remaining state members */
	atomic_set(&state->rx_good, 0);
	atomic_set(&state->rx_bad, 0);
	smp_wmb();
}

/*
 * Perform loopback test with N packets
 *
 * This will transmit "num_packets" copies of a test packet, and check
 * that they were both transmitted (i.e. a TX completion event was
 * received) and received (i.e. the data arrived intact via loopback).
 * The port must have already been placed into the desired loopback
 * mode.
 */
static int efx_test_loopback(struct efx_nic *efx,
			     struct efx_tx_queue *tx_queue,
			     struct efx_loopback_self_tests *lb_tests)
{
#if !defined(EFX_HAVE_OLD_NAPI)
	struct efx_channel *channel;
#endif
	struct efx_selftest_state *state = efx->loopback_selftest;
	struct efx_loopback_payload *payload;
	struct sk_buff *skb;
	int rc = 0, i, tx_done, rx_good, rx_bad;

	/* Fill out the packet contents */
	efx_iterate_state(efx);

	/* Create and fill skb */
	skb = alloc_skb(sizeof(state->payload), GFP_KERNEL);
	if (!skb) {
		rc = -ENOMEM;
		goto out1;
	}
	payload = ((struct efx_loopback_payload *)
		   skb_put(skb, sizeof(state->payload)));
	memcpy(payload, &state->payload, sizeof(state->payload));

	/* Transmit N copies of buffer */
	for (i = 0; i < state->packet_count; i++) {
		/* Set the source address in the copy of the packet.
		 * Incrementing the source address on a per-packet basis
		 * should ensure that we stress all RSS vectors */
		payload->ip.saddr = htonl(INADDR_LOOPBACK | (i << 2));
		skb_get(skb);

		if (efx_xmit(efx, tx_queue, skb) != NETDEV_TX_OK) {
			EFX_ERR(efx, "TX queue %d could not transmit "
				"packet %d of %d in %s loopback test\n",
				tx_queue->queue, i + 1,
				state->packet_count, LOOPBACK_MODE(efx));
			rc = -EPIPE;
			goto out2;
		}

		/* Avoid hogging the PCI bus */
		udelay(10);
	}

#if !defined(EFX_HAVE_OLD_NAPI)
	/* NAPI polling is not enabled, so process channels synchronously */
	schedule_timeout_uninterruptible(HZ / 50);
	efx_for_each_channel_with_interrupt(channel, efx) {
		if (channel->work_pending)
			efx_process_channel_now(channel);
	}
#else
	/* Allow time for processing */
	schedule_timeout_uninterruptible(HZ / 10);
#endif

	if (state->flush)
		goto out3;

	/* Check TX completion and received packet counts */
	tx_done = state->packet_count - (atomic_read(&skb->users) - 1);
	rx_good = atomic_read(&state->rx_good);
	rx_bad = atomic_read(&state->rx_bad);
	if (tx_done != state->packet_count) {
		/* Don't free the skbs; they will be picked up on TX
		 * overflow or channel teardown.
		 */
		EFX_ERR(efx, "TX queue %d saw only %d out of an "
			"expected %d TX completion events in %s loopback "
			"test\n", tx_queue->queue, tx_done,
			state->packet_count, LOOPBACK_MODE(efx));
		rc = -ETIMEDOUT;
		/* Allow to fall through so we see the RX errors as well */
	}

	/* We may always be up to a flush away from our desired packet total */
	if (rx_good != state->packet_count) {
		EFX_LOG(efx, "TX queue %d saw only %d out of an "
			"expected %d received packets in %s loopback "
			"test\n", tx_queue->queue, rx_good,
			state->packet_count, LOOPBACK_MODE(efx));
		rc = -ETIMEDOUT;
		/* Fall through */
	}

	/* Update loopback test structure */
	lb_tests->tx_sent[tx_queue->queue] += state->packet_count;
	lb_tests->tx_done[tx_queue->queue] += tx_done;
	lb_tests->rx_good += rx_good;
	lb_tests->rx_bad += rx_bad;

 out3:
 out2:
	/* Free skb */
	if (skb) {
		/* If the selftest failed then the skb needs its reference
		 * count decreasing */
		while (skb_shared(skb))
			kfree_skb(skb);
		dev_kfree_skb_any(skb);
	}

 out1:
	return rc;
}

/* Perform loopback test safely
 *
 * This performs a safe loopback test by starting with a single packet
 * and only increasing the number of packets while the tests are
 * passing.  This avoids flooding a network with garbage packets if
 * e.g. setting the loopback mode fails.
 */
static int
efx_test_loopback_safely(struct efx_nic *efx,
			 struct efx_tx_queue *tx_queue,
			 struct efx_loopback_self_tests *lb_tests)
{
	struct efx_selftest_state *state = efx->loopback_selftest;
	int i, rc = 0;

	for (i = 0; i < loopback_test_level; i++) {
		/* Determine how many packets to send */
		state->packet_count = (efx->type->txd_ring_mask + 1) / 3;
		state->packet_count = min(1 << (i << 2), state->packet_count);
		state->flush = 0;

		EFX_LOG(efx, "TX queue %d testing %s loopback"
			" with %d packets\n", tx_queue->queue,
			LOOPBACK_MODE(efx), state->packet_count);

		rc = efx_test_loopback(efx, tx_queue, lb_tests);
		if (rc) {
			/* Wait a while to ensure there are no packets
			 * floating around after a failure.
			 */
			schedule_timeout_uninterruptible(HZ / 5);
			return rc;
		}
	}

	EFX_LOG(efx, "TX queue %d passed %s loopback test "
		"with a burst length of %d packets\n",
		tx_queue->queue, LOOPBACK_MODE(efx), state->packet_count);

	return rc;
}

static int efx_test_loopbacks(struct efx_nic *efx,
			      struct efx_self_tests *tests,
			      unsigned int loopback_modes)
{
	struct efx_selftest_state *state = efx->loopback_selftest;
	struct ethtool_cmd ecmd, ecmd_loopback;
	struct efx_tx_queue *tx_queue;
	enum efx_loopback_mode old_mode, mode;
	int old_powered, count, rc = 0;
	int retry = EFX_WORKAROUND_8909(efx);

	/* Get current PHY settings */
	rc = efx_ethtool_get_settings(efx->net_dev, &ecmd);
	if (rc) {
		EFX_ERR(efx, "could not get GMII settings\n");
		return rc;
	}
	old_mode = efx->loopback_mode;
	old_powered = efx->phy_powered;

	/* Disable autonegotiation for the purposes of loopback */
	memcpy(&ecmd_loopback, &ecmd, sizeof(ecmd_loopback));
	if (ecmd_loopback.autoneg == AUTONEG_ENABLE) {
		ecmd_loopback.autoneg = AUTONEG_DISABLE;
		ecmd_loopback.duplex = DUPLEX_FULL;
		ecmd_loopback.speed = EFX_IS10G(efx) ?
			SPEED_10000 : SPEED_1000;
	}

	rc = efx_ethtool_set_settings(efx->net_dev, &ecmd_loopback);
	if (rc) {
		EFX_ERR(efx, "could not disable autonegotiation\n");
		goto out;
	}
	tests->loopback_speed = ecmd_loopback.speed;
	tests->loopback_full_duplex = ecmd_loopback.duplex;

	/* Test all supported loopback modes */
	for (mode = LOOPBACK_NONE; mode < LOOPBACK_TEST_MAX; mode++) {
		if (!(loopback_modes & (1 << mode)))
			continue;

		/* Move the port into the specified loopback mode. */
		state->flush = 1;
		efx->phy_powered = 1;
		efx->loopback_mode = mode;
		efx_reconfigure_port(efx, 0);

		/* Wait for the PHY to signal the link is up */
		count = 0;
		do {
			struct efx_channel *channel = &efx->channel[0];

			(void) efx->mac_op->check_hw(efx);
			schedule_timeout_uninterruptible(HZ / 10);
			if (channel->work_pending)
				efx_process_channel_now(channel);
			/* Wait for PHY events to be processed */
			flush_workqueue(efx->workqueue);
			rmb();
		} while ((++count < 20) && !efx->link_up);

		/* The link should now be up. If it isn't, there is no point
		 * in attempting a loopback test */
		if (!efx->link_up) {
			EFX_ERR(efx, "loopback %s never came up\n",
				LOOPBACK_MODE(efx));
			rc = -EIO;
			goto out;
		}

		EFX_LOG(efx, "link came up in %s loopback in %d iterations\n",
			LOOPBACK_MODE(efx), count);

		/* Test every TX queue */
		efx_for_each_tx_queue(tx_queue, efx) {
			rc |= efx_test_loopback_safely(efx, tx_queue,
						       &tests->loopback[mode]);
			if (rc)
				goto fail;
		}

		continue;

fail:
		if (retry) {
			/* Give the PHY a kick by pretending to move into
			 * a Falcon internal loopback mode, then back out */
			int first = ffs(efx->loopback_modes) - 1;

			EFX_INFO(efx, "retrying %s loopback\n",
				 LOOPBACK_MODE(efx));

			state->flush = 1;
			efx->loopback_mode = first;
			efx_reconfigure_port(efx, 0);

			retry = rc = 0;
			--mode;
			continue;
		}
		break;
	}

 out:
	/* Take out of loopback and restore PHY settings */
	state->flush = 1;
	efx->loopback_mode = old_mode;
	efx->phy_powered = old_powered;
	/* Push the loopback change, and restore any other
	 * settings we may have trodden on */
	(void) efx_ethtool_set_settings(efx->net_dev, &ecmd);

	return rc;
}

/**************************************************************************
 *
 * Entry points
 *
 *************************************************************************/

/* Online (i.e. non-disruptive) testing
 *
 * This checks interrupt generation, event delivery and PHY presence.
 * The caller should hold the suspend lock
 *
 */
int efx_online_test(struct efx_nic *efx, struct efx_self_tests *tests)
{
	struct efx_channel *channel;
	int rc = 0;

	ASSERT_RTNL();

	EFX_LOG(efx, "performing online self-tests\n");

	rc |= efx_test_interrupts(efx, tests);
	efx_for_each_channel(channel, efx) {
		if (channel->has_interrupt)
			rc |= efx_test_eventq_irq(channel, tests);
		else
			rc |= efx_test_eventq(channel, tests);
	}
	rc |= efx_test_phy(efx, tests);

	if (rc)
		EFX_ERR(efx, "failed online self-tests\n");

	return rc;
}

/* Offline (i.e. disruptive) testing
 * This checks MAC and PHY loopback on the specified port. The caller
 * should hold the rtnl lock
 */
int efx_offline_test(struct efx_nic *efx,
		     struct efx_self_tests *tests, unsigned int loopback_modes)
{
	struct efx_selftest_state *state;
	int rc = 0;

	ASSERT_RTNL();

	EFX_LOG(efx, "performing offline self-tests\n");

	/* Create a selftest_state structure to hold state for the test */
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state == NULL) {
		rc = -ENOMEM;
		goto out;
	}

	/* Set the port loopback_selftest member. From this point on
	 * all received packets will be dropped. Mark the state as
	 * "flushing" so all inflight packets are dropped */
	BUG_ON(efx->loopback_selftest);
	state->flush = 1;
	efx->loopback_selftest = (void *)state;
	wmb();

	/* Test all loopback  modes */
	rc = efx_test_loopbacks(efx, tests, loopback_modes);

	/* Tidy up the port test state */
	efx->loopback_selftest = NULL;
	wmb();
	kfree(state);

 out:
	if (rc)
		EFX_ERR(efx, "failed offline self-tests\n");

	return rc;
}

