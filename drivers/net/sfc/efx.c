/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005-2006: Fen Systems Ltd.
 * Copyright 2005-2008: Solarflare Communications Inc,
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

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/in.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>
#include <asm/uaccess.h>
#include "net_driver.h"
#include "gmii.h"
#include "driverlink.h"
#include "selftest.h"
#include "debugfs.h"
#include "ethtool.h"
#include "tx.h"
#include "rx.h"
#include "efx.h"
#include "mdio_10g.h"
#include "falcon.h"
#include "workarounds.h"

/**************************************************************************
 *
 * Type name strings
 *
 **************************************************************************
 */

/* Loopback mode names (see LOOPBACK_MODE()) */
const unsigned int efx_loopback_mode_max = LOOPBACK_MAX;
const char *efx_loopback_mode_names[] = {
	[LOOPBACK_NONE]	   = "NONE",
	[LOOPBACK_MAC]	   = "MAC",
	[LOOPBACK_XGMII]   = "XGMII",
	[LOOPBACK_XGXS]	   = "XGXS",
	[LOOPBACK_XAUI]    = "XAUI",
	[LOOPBACK_PHY]	   = "PHY",
	[LOOPBACK_PHYXS]   = "PHY(XS)",
	[LOOPBACK_PCS]     = "PHY(PCS)",
	[LOOPBACK_PMAPMD]  = "PHY(PMAPMD)",
	[LOOPBACK_NETWORK] = "NETWORK",
};

/* Interrupt mode names (see INT_MODE())) */
const unsigned int efx_interrupt_mode_max = EFX_INT_MODE_MAX;
const char *efx_interrupt_mode_names[] = {
	[EFX_INT_MODE_MSIX]   = "MSI-X",
	[EFX_INT_MODE_MSI]    = "MSI",
	[EFX_INT_MODE_LEGACY] = "legacy",
};

/* PHY type names (see PHY_TYPE())) */
const unsigned int efx_phy_type_max = PHY_TYPE_MAX;
const char *efx_phy_type_names[] = {
	[PHY_TYPE_NONE]        = "none",
	[PHY_TYPE_CX4_RTMR]    = "Mysticom CX4",
	[PHY_TYPE_1G_ALASKA]   = "1G Alaska",
	[PHY_TYPE_10XPRESS]    = "SFC 10Xpress",
	[PHY_TYPE_XFP]         = "Quake XFP",
	[PHY_TYPE_PM8358]      = "PM8358 XAUI",
};

const unsigned int efx_reset_type_max = RESET_TYPE_MAX;
const char *efx_reset_type_names[] = {
	[RESET_TYPE_INVISIBLE]    = "INVISIBLE",
	[RESET_TYPE_ALL]          = "ALL",
	[RESET_TYPE_WORLD]        = "WORLD",
	[RESET_TYPE_DISABLE]      = "DISABLE",
	[RESET_TYPE_MONITOR]      = "MONITOR",
	[RESET_TYPE_INT_ERROR]    = "INT_ERROR",
	[RESET_TYPE_RX_RECOVERY]  = "RX_RECOVERY",
};

const unsigned int efx_nic_state_max = STATE_MAX;
const char *efx_nic_state_names[] = {
	[STATE_INIT]          = "INIT",
	[STATE_RUNNING]       = "RUNNING",
	[STATE_FINI]          = "FINI",
	[STATE_RESETTING]     = "RESETTING",
	[STATE_DISABLED]      = "DISABLED",
};

#define EFX_MAX_MTU (9 * 1024)


/**************************************************************************
 *
 * Configurable values
 *
 *************************************************************************/

/*
 * Use separate channels for TX and RX events
 *
 * Set this to 1 to use separate channels for TX and RX. It allows us to
 * apply a higher level of interrupt moderation to TX events.
 *
 * This is forced to 0 for MSI interrupt mode as the interrupt vector
 * is not written
 */
static unsigned int separate_tx_and_rx_channels = 1;

/* This is the weight assigned to each of the (per-channel) virtual
 * NAPI devices.
 */
static int napi_weight = 64;

/* This is the time (in jiffies) between invocations of the hardware
 * monitor, which checks for known hardware bugs and resets the
 * hardware and driver as necessary.
 */
unsigned int efx_monitor_interval = 1 * HZ;

/* This controls whether or not the hardware monitor will trigger a
 * reset when it detects an error condition.
 */
static unsigned int monitor_reset = 1;

/* This controls whether or not the driver will initialise devices
 * with invalid MAC addresses stored in the EEPROM or flash.  If true,
 * such devices will be initialised with a random locally-generated
 * MAC address.  This allows for loading the efx_mtd driver to
 * reprogram the flash, even if the flash contents (including the MAC
 * address) have previously been erased.
 */
static unsigned int allow_bad_hwaddr;

/* Initial interrupt moderation settings.  They can be modified after
 * module load with ethtool.
 *
 * The default for RX should strike a balance between increasing the
 * round-trip latency and reducing overhead.
 */
static unsigned int rx_irq_mod_usec = 60;

/* Initial interrupt moderation settings.  They can be modified after
 * module load with ethtool.
 *
 * This default is chosen to ensure that a 10G link does not go idle
 * while a TX queue is stopped after it has become full.  A queue is
 * restarted when it drops below half full.  The time this takes (assuming
 * worst case 3 descriptors per packet and 1024 descriptors) is
 *   512 / 3 * 1.2 = 205 usec.
 */
static unsigned int tx_irq_mod_usec = 150;

/* Ignore online self-test failures at load
 *
 * If set to 1, then the driver will not fail to load
 * if the online self-test fails. Useful only during testing
 */
static unsigned int allow_load_on_failure;

/* Set to 1 to enable the use of Message-Signalled Interrupts (MSI).
 * MSI will not work on some motherboards due to limitations of the
 * chipset, so the default is off.
 *
 * This is the highest capability interrupt mode to use
 * 0 => MSI-X
 * 1 => MSI
 * 2 => legacy
 */
static unsigned int interrupt_mode;

/* If set to 1, then the driver will perform an offline self test
 * when each interface first comes up. This will appear like the
 * interface bounces up and down
 */
static unsigned int onload_offline_selftest = 1;

/* This is the requested number of CPUs to use for Receive-Side Scaling (RSS),
 * i.e. the number of CPUs among which we may distribute simultaneous
 * interrupt handling.
 *
 * Cards without MSI-X will only target one CPU
 *
 * Default (0) means to use all CPUs in the system.  This parameter
 * can be set using "rss_cpus=xxx" when loading the module.
 */
static unsigned int rss_cpus;
module_param(rss_cpus, uint, 0444);
MODULE_PARM_DESC(rss_cpus, "Number of CPUs to use for Receive-Side Scaling");

/**************************************************************************
 *
 * Utility functions and prototypes
 *
 *************************************************************************/
static void efx_remove_channel(struct efx_channel *channel);
static void efx_remove_port(struct efx_nic *efx);
static void efx_fini_napi(struct efx_nic *efx);
static void efx_fini_channels(struct efx_nic *efx);

/**************************************************************************
 *
 * Event queue processing
 *
 *************************************************************************/

/* Process channel's event queue
 *
 * This function is responsible for processing the event queue of a
 * single channel.  The caller must guarantee that this function will
 * never be concurrently called more than once on the same channel,
 * though different channels may be being processed concurrently.
 */
static inline int efx_process_channel(struct efx_channel *channel, int rx_quota)
{
	int rxdmaqs;
	struct efx_rx_queue *rx_queue;

	if (unlikely(channel->efx->reset_pending != RESET_TYPE_NONE ||
		     !channel->enabled))
		return rx_quota;

	rxdmaqs = falcon_process_eventq(channel, &rx_quota);

	/* Deliver last RX packet. */
	if (channel->rx_pkt) {
		__efx_rx_packet(channel, channel->rx_pkt,
				channel->rx_pkt_csummed);
		channel->rx_pkt = NULL;
	}

	efx_rx_strategy(channel);

	/* Refill descriptor rings as necessary */
	rx_queue = &channel->efx->rx_queue[0];
	while (rxdmaqs) {
		if (rxdmaqs & 0x01)
			efx_fast_push_rx_descriptors(rx_queue);
		rx_queue++;
		rxdmaqs >>= 1;
	}

	return rx_quota;
}

/* Mark channel as finished processing
 *
 * Note that since we will not receive further interrupts for this
 * channel before we finish processing and call the eventq_read_ack()
 * method, there is no need to use the interrupt hold-off timers.
 */
static inline void efx_channel_processed(struct efx_channel *channel)
{
	/* Write to EVQ_RPTR_REG.  If a new event arrived in a race
	 * with finishing processing, a new interrupt will be raised.
	 */
	channel->work_pending = 0;
	smp_wmb(); /* Ensure channel updated before any new interrupt. */
	falcon_eventq_read_ack(channel);
}

/* NAPI poll handler
 *
 * NAPI guarantees serialisation of polls of the same device, which
 * provides the guarantee required by efx_process_channel().
 */
#if !defined(EFX_HAVE_OLD_NAPI)
static int efx_poll(struct napi_struct *napi, int budget)
{
	struct efx_channel *channel =
		container_of(napi, struct efx_channel, napi_str);
	struct net_device *napi_dev = channel->napi_dev;
#else
static int efx_poll(struct net_device *napi, int *budget_ret)
{
	struct net_device *napi_dev = napi;
	struct efx_channel *channel = napi_dev->priv;
	int budget = min(napi_dev->quota, *budget_ret);
#endif
	int unused;
	int rx_packets;

	EFX_TRACE(channel->efx, "channel %d NAPI poll executing on CPU %d\n",
		  channel->channel, raw_smp_processor_id());

	unused = efx_process_channel(channel, budget);
	rx_packets = (budget - unused);
#if defined(EFX_HAVE_OLD_NAPI)
	napi_dev->quota -= rx_packets;
	*budget_ret -= rx_packets;
#endif

	if (rx_packets < budget) {
		/* There is no race here; although napi_disable() will
		 * only wait for netif_rx_complete(), this isn't a problem
		 * since efx_channel_processed() will have no effect if
		 * interrupts have already been disabled.
		 */
		netif_rx_complete(napi_dev, napi);
		efx_channel_processed(channel);
	}

#if !defined(EFX_HAVE_OLD_NAPI)
	return rx_packets;
#else
	return (rx_packets >= budget);
#endif
}

/* Process the eventq of the specified channel immediately on this CPU
 *
 * Disable hardware generated interrupts, wait for any existing
 * processing to finish, then directly poll (and ack ) the eventq.
 * Finally reenable NAPI and interrupts.
 *
 * Since we are touching interrupts the caller should hold the suspend lock
 */
void efx_process_channel_now(struct efx_channel *channel)
{
	struct efx_nic *efx = channel->efx;

	BUG_ON(!channel->used_flags);
	BUG_ON(!channel->enabled);

	/* Disable interrupts and wait for ISRs to complete */
	falcon_disable_interrupts(efx);
	if (efx->legacy_irq)
		synchronize_irq(efx->legacy_irq);
	if (channel->has_interrupt && channel->irq)
		synchronize_irq(channel->irq);

	/* Wait for any NAPI processing to complete */
	napi_disable(&channel->napi_str);

	/* Poll the channel */
	(void) efx_process_channel(channel, efx->type->evq_size);

	/* Ack the eventq. This may cause an interrupt to be generated
	 * when they are reenabled */
	efx_channel_processed(channel);

	/* Reenable NAPI polling */
	napi_enable(&channel->napi_str);

	/* Reenable interrupts */
	falcon_enable_interrupts(efx);
}

/* Create event queue
 * Event queue memory allocations are done only once.  If the channel
 * is reset, the memory buffer will be reused; this guards against
 * errors during channel reset and also simplifies interrupt handling.
 */
static int efx_probe_eventq(struct efx_channel *channel)
{
	EFX_LOG(channel->efx, "chan %d create event queue\n", channel->channel);

	return falcon_probe_eventq(channel);
}

/* Prepare channel's event queue */
static int efx_init_eventq(struct efx_channel *channel)
{
	EFX_LOG(channel->efx, "chan %d init event queue\n", channel->channel);

	ASSERT_RTNL();

	/* Initialise fields */
	channel->eventq_read_ptr = 0;

	return falcon_init_eventq(channel);
}

static void efx_fini_eventq(struct efx_channel *channel)
{
	EFX_LOG(channel->efx, "chan %d fini event queue\n", channel->channel);

	ASSERT_RTNL();

	falcon_fini_eventq(channel);
}

static void efx_remove_eventq(struct efx_channel *channel)
{
	EFX_LOG(channel->efx, "chan %d remove event queue\n", channel->channel);

	falcon_remove_eventq(channel);
}

/**************************************************************************
 *
 * Channel handling
 *
 *************************************************************************/

/* Setup per-NIC RX buffer parameters.
 * Calculate the rx buffer allocation parameters required to support
 * the current MTU, including padding for header alignment and overruns.
 */
static void efx_calc_rx_buffer_params(struct efx_nic *efx)
{
	unsigned int order, len;

	len = (max(EFX_PAGE_IP_ALIGN, NET_IP_ALIGN) +
	       EFX_MAX_FRAME_LEN(efx->net_dev->mtu) +
	       efx->type->rx_buffer_padding);

	/* Page-based allocation page-order */
	for (order = 0; ((1u << order) * PAGE_SIZE) < len; ++order)
		;

	efx->rx_buffer_len = len;
	efx->rx_buffer_order = order;
}

static int efx_probe_channel(struct efx_channel *channel)
{
	struct efx_tx_queue *tx_queue;
	struct efx_rx_queue *rx_queue;
	int rc;

	EFX_LOG(channel->efx, "creating channel %d\n", channel->channel);

	rc = efx_probe_eventq(channel);
	if (rc)
		goto fail1;

	efx_for_each_channel_tx_queue(tx_queue, channel) {
		rc = efx_probe_tx_queue(tx_queue);
		if (rc)
			goto fail2;
	}

	efx_for_each_channel_rx_queue(rx_queue, channel) {
		rc = efx_probe_rx_queue(rx_queue);
		if (rc)
			goto fail3;
	}

	channel->n_rx_frm_trunc = 0;

	return 0;

 fail3:
	efx_for_each_channel_rx_queue(rx_queue, channel)
		efx_remove_rx_queue(rx_queue);
 fail2:
	efx_for_each_channel_tx_queue(tx_queue, channel)
		efx_remove_tx_queue(tx_queue);
 fail1:
	return rc;
}


/* Channels are shutdown and reinitialised whilst the NIC is running
 * to propagate configuration changes (mtu, checksum offload), or
 * to clear hardware error conditions
 */
static int efx_init_channels(struct efx_nic *efx)
{
	struct efx_tx_queue *tx_queue;
	struct efx_rx_queue *rx_queue;
	struct efx_channel *channel;
	int rc = 0;

	/* Recalculate the rx buffer parameters */
	efx_calc_rx_buffer_params(efx);

	/* Initialise the channels */
	efx_for_each_channel(channel, efx) {
		EFX_LOG(channel->efx, "init chan %d\n", channel->channel);

		rc = efx_init_eventq(channel);
		if (rc)
			goto err;

		efx_for_each_channel_tx_queue(tx_queue, channel) {
			rc = efx_init_tx_queue(tx_queue);
			if (rc)
				goto err;
		}

		/* The rx buffer allocation strategy is MTU dependent */
		efx_rx_strategy(channel);

		efx_for_each_channel_rx_queue(rx_queue, channel) {
			rc = efx_init_rx_queue(rx_queue);
			if (rc)
				goto err;
		}

		WARN_ON(channel->rx_pkt != NULL);
		efx_rx_strategy(channel);
	}

	return 0;

 err:
	EFX_ERR(efx, "failed to initialise channel %d\n",
		channel ? channel->channel : -1);
	efx_fini_channels(efx);
	return rc;
}

/* This enables event queue processing and packet transmission.
 *
 * Note that this function is not allowed to fail, since that would
 * introduce too much complexity into the suspend/resume path.
 */
static void efx_start_channel(struct efx_channel *channel)
{
	struct efx_rx_queue *rx_queue;

	EFX_LOG(channel->efx, "starting chan %d\n", channel->channel);

	if (!(channel->efx->net_dev->flags & IFF_UP))
		netif_napi_add(channel->napi_dev, &channel->napi_str,
			       efx_poll, napi_weight);

	/* Mark channel as enabled */
	channel->work_pending = 0;
	channel->enabled = 1;
	smp_wmb(); /* ensure channel updated before first interrupt */

	/* Enable NAPI poll handler */
	napi_enable(&channel->napi_str);

	/* Load up RX descriptors */
	efx_for_each_channel_rx_queue(rx_queue, channel)
		efx_fast_push_rx_descriptors(rx_queue);
}

/* This disables event queue processing and packet transmission.
 * This function does not guarantee that all queue processing
 * (e.g. RX refill) is complete.
 */
static void efx_stop_channel(struct efx_channel *channel)
{
	struct efx_rx_queue *rx_queue;

	if (!channel->enabled)
		return;

	EFX_LOG(channel->efx, "stop chan %d\n", channel->channel);

	/* Mark channel as disabled */
	channel->enabled = 0;

	/* Wait for any NAPI processing to complete */
	napi_disable(&channel->napi_str);

	/* Ensure that any worker threads have exited or will be
	 * no-ops.
	 */
	efx_for_each_channel_rx_queue(rx_queue, channel) {
		spin_lock_bh(&rx_queue->add_lock);
		spin_unlock_bh(&rx_queue->add_lock);
	}
}

static void efx_fini_channels(struct efx_nic *efx)
{
	struct efx_channel *channel;
	struct efx_tx_queue *tx_queue;
	struct efx_rx_queue *rx_queue;

	ASSERT_RTNL();

	efx_for_each_channel(channel, efx) {
		EFX_LOG(channel->efx, "shut down chan %d\n", channel->channel);

		efx_for_each_channel_rx_queue(rx_queue, channel)
			efx_fini_rx_queue(rx_queue);
		efx_for_each_channel_tx_queue(tx_queue, channel)
			efx_fini_tx_queue(tx_queue);
	}

	/* Do the event queues last so that we can handle flush events
	 * for all DMA queues. */
	efx_for_each_channel(channel, efx) {
		EFX_LOG(channel->efx, "shut down evq %d\n", channel->channel);

		efx_fini_eventq(channel);
	}
}

static void efx_remove_channel(struct efx_channel *channel)
{
	struct efx_tx_queue *tx_queue;
	struct efx_rx_queue *rx_queue;

	EFX_LOG(channel->efx, "destroy chan %d\n", channel->channel);

	efx_for_each_channel_rx_queue(rx_queue, channel)
		efx_remove_rx_queue(rx_queue);
	efx_for_each_channel_tx_queue(tx_queue, channel)
		efx_remove_tx_queue(tx_queue);
	efx_remove_eventq(channel);

	channel->used_flags = 0;
}

/**************************************************************************
 *
 * Port handling
 *
 **************************************************************************/

/* This ensures that the kernel is kept informed (via
 * netif_carrier_on/off) of the link status, and also maintains the
 * link status's stop on the port's TX queue.
 */
static void efx_link_status_changed(struct efx_nic *efx)
{
	unsigned long flags __attribute__ ((unused));
	int carrier_ok;

	/* Ensure no link status notifications get sent to the OS after the net
	 * device has been unregistered. */
	if (!efx->net_dev_registered)
		return;

	carrier_ok = netif_carrier_ok(efx->net_dev) ? 1 : 0;
	if (efx->link_up != carrier_ok) {
		efx->n_link_state_changes++;

		if (efx->link_up)
			netif_carrier_on(efx->net_dev);
		else
			netif_carrier_off(efx->net_dev);
	}

	/* Inform driverlink client */
	EFX_DL_CALLBACK(efx, link_change, efx->link_up);

	/* Status message for kernel log */
	if (efx->link_up) {
		struct mii_if_info *gmii = &efx->mii;
		unsigned adv, lpa;
		/* NONE here means direct XAUI from the controller, with no
		 * MDIO-attached device we can query. */
		if (efx->phy_type != PHY_TYPE_NONE) {
			adv = gmii_advertised(gmii);
			lpa = gmii_lpa(gmii);
		} else {
			lpa = GM_LPA_10000 | LPA_DUPLEX;
			adv = lpa;
		}
		EFX_INFO(efx, "link up at %dMbps %s-duplex "
			 "(adv %04x lpa %04x) (MTU %d)%s%s%s%s\n",
			 (efx->link_options & GM_LPA_10000 ? 10000 :
			  (efx->link_options & GM_LPA_1000 ? 1000 :
			   (efx->link_options & GM_LPA_100 ? 100 :
			    10))),
			 (efx->link_options & GM_LPA_DUPLEX ?
			  "full" : "half"),
			 adv, lpa,
			 efx->net_dev->mtu,
			 (efx->loopback_mode ? " [" : ""),
			 (efx->loopback_mode ? LOOPBACK_MODE(efx) : ""),
			 (efx->loopback_mode ? " LOOPBACK]" : ""),
			 (efx->promiscuous ? " [PROMISC]" : ""));
	} else {
		EFX_INFO(efx, "link down\n");
	}

}

/* This call reinitialises the MAC to pick up new PHY settings
 * To call from a context that cannot sleep use reconfigure_work work item
 * For on_disabled=1 the caller must be serialised against efx_reset,
 * ideally by holding the rtnl lock.
 */
void efx_reconfigure_port(struct efx_nic *efx, int on_disabled)
{
	mutex_lock(&efx->mac_lock);

	EFX_LOG(efx, "reconfiguring MAC from PHY settings\n");

	if (on_disabled)
		ASSERT_RTNL();
	else if (!efx->port_enabled)
		goto out;

	efx->mac_op->reconfigure(efx);

out:
	/* Inform kernel of loss/gain of carrier */
	efx_link_status_changed(efx);

	mutex_unlock(&efx->mac_lock);
}

static void efx_reconfigure_work(struct work_struct *data)
{
	struct efx_nic *efx = container_of(data, struct efx_nic,
					   reconfigure_work);

	EFX_LOG(efx, "MAC reconfigure executing on CPU %d\n",
		raw_smp_processor_id());

	/* Reinitialise MAC to activate new PHY parameters */
	efx_reconfigure_port(efx, 0);
}

static int efx_probe_port(struct efx_nic *efx)
{
	unsigned char *dev_addr;
	int rc;

	EFX_LOG(efx, "create port\n");

	/* Connect up MAC/PHY operations table and read MAC address */
	rc = falcon_probe_port(efx);
	if (rc)
		goto err;

	/* Sanity check MAC address */
	dev_addr = efx->mac_address;
	if (!is_valid_ether_addr(dev_addr)) {
		DECLARE_MAC_BUF(mac);

		EFX_ERR(efx, "invalid MAC address %s\n",
			print_mac(mac, dev_addr));
		if (!allow_bad_hwaddr) {
			rc = -EINVAL;
			goto err;
		}
		random_ether_addr(dev_addr);
		EFX_INFO(efx, "using locally-generated MAC %s\n",
			 print_mac(mac, dev_addr));
	}

	/* Register debugfs entries */
	rc = efx_init_debugfs_port(efx);
	if (rc)
		goto err;

	return 0;

 err:
	efx_remove_port(efx);
	return rc;
}

static int efx_init_port(struct efx_nic *efx)
{
	int rc;

	EFX_LOG(efx, "init port\n");

	/* The default power state is ON */
	efx->phy_powered = 1;

	/* Initialise the MAC and PHY */
	rc = efx->mac_op->init(efx);
	if (rc)
		return rc;

	efx->port_initialized = 1;

	/* Reconfigure port to program MAC registers */
	efx->mac_op->reconfigure(efx);

	return 0;
}

/* Allow efx_reconfigure_port() to run, and propagate delayed changes
 * to the promiscuous flag to the MAC if needed */
static void efx_start_port(struct efx_nic *efx)
{
	EFX_LOG(efx, "start port\n");
	ASSERT_RTNL();

	BUG_ON(efx->port_enabled);

	mutex_lock(&efx->mac_lock);
	efx->port_enabled = 1;
	mutex_unlock(&efx->mac_lock);

	if (efx->net_dev_registered) {
		int promiscuous;

		netif_tx_lock_bh(efx->net_dev);
		promiscuous = (efx->net_dev->flags & IFF_PROMISC) ? 1 : 0;
		if (efx->promiscuous != promiscuous) {
			efx->promiscuous = promiscuous;
			queue_work(efx->workqueue, &efx->reconfigure_work);
		}
		netif_tx_unlock_bh(efx->net_dev);
	}
}

/* Prevents efx_reconfigure_port() from executing, and prevents
 * efx_set_multicast_list() from scheduling efx_reconfigure_work.
 * efx_reconfigure_work can still be scheduled via NAPI processing
 * until efx_flush_all() is called */
static void efx_stop_port(struct efx_nic *efx)
{
	EFX_LOG(efx, "stop port\n");
	ASSERT_RTNL();

	mutex_lock(&efx->mac_lock);
	efx->port_enabled = 0;
	mutex_unlock(&efx->mac_lock);

	/* Serialise against efx_set_multicast_list() */
	if (efx->net_dev_registered) {
		netif_tx_lock_bh(efx->net_dev);
		netif_tx_unlock_bh(efx->net_dev);
	}
}

static void efx_fini_port(struct efx_nic *efx)
{
	EFX_LOG(efx, "shut down port\n");

	if (!efx->port_initialized)
		return;

	efx->mac_op->fini(efx);
	efx->port_initialized = 0;

	/* Mark the link down */
	efx->link_up = 0;
	efx_link_status_changed(efx);
}

static void efx_remove_port(struct efx_nic *efx)
{
	EFX_LOG(efx, "destroying port\n");

	efx_fini_debugfs_port(efx);
	falcon_remove_port(efx);
}

/**************************************************************************
 *
 * NIC handling
 *
 **************************************************************************/

/* This configures the PCI device to enable I/O and DMA. */
static int efx_init_io(struct efx_nic *efx)
{
	struct pci_dev *pci_dev = efx->pci_dev;
	int rc;

	EFX_LOG(efx, "initialising I/O\n");

	/* Generic device-enabling code */
	rc = pci_enable_device(pci_dev);
	if (rc) {
		EFX_ERR(efx, "failed to enable PCI device\n");
		goto fail1;
	}

	pci_set_master(pci_dev);

	/* Set the PCI DMA mask.  Try all possibilities from our
	 * genuine mask down to 32 bits, because some architectures
	 * (e.g. x86_64 with iommu_sac_force set) will allow 40 bit
	 * masks event though they reject 46 bit masks.
	 */
	efx->dma_mask = efx->type->max_dma_mask;
	while (efx->dma_mask > 0x7fffffffUL) {
		if (pci_dma_supported(pci_dev, efx->dma_mask) &&
		    ((rc = pci_set_dma_mask(pci_dev, efx->dma_mask)) == 0))
			break;
		efx->dma_mask >>= 1;
	}
	if (rc) {
		EFX_ERR(efx, "could not find a suitable DMA mask\n");
		goto fail2;
	}
	EFX_LOG(efx, "using DMA mask %llx\n",
		(unsigned long long)efx->dma_mask);
	rc = pci_set_consistent_dma_mask(pci_dev, efx->dma_mask);
	if (rc) {
		/* pci_set_consistent_dma_mask() is not *allowed* to
		 * fail with a mask that pci_set_dma_mask() accepted,
		 * but just in case...
		 */
		EFX_ERR(efx, "failed to set consistent DMA mask\n");
		goto fail2;
	}

	/* Get memory base address */
	efx->membase_phys = pci_resource_start(efx->pci_dev,
					       efx->type->mem_bar);
#if !defined(EFX_HAVE_MSIX_TABLE_RESERVED)
	rc = pci_request_region(pci_dev, efx->type->mem_bar, "sfc");
#else
	if (!request_mem_region(efx->membase_phys, efx->type->mem_map_size,
				"sfc"))
		rc = -EIO;
#endif
	if (rc) {
		EFX_ERR(efx, "request for memory BAR failed\n");
		rc = -EIO;
		goto fail3;
	}
	efx->membase = ioremap_nocache(efx->membase_phys,
				       efx->type->mem_map_size);
	if (!efx->membase) {
		EFX_ERR(efx, "could not map memory BAR %d at %lx+%x\n",
			efx->type->mem_bar, efx->membase_phys,
			efx->type->mem_map_size);
		rc = -ENOMEM;
		goto fail4;
	}
	EFX_LOG(efx, "memory BAR %u at %lx+%x (virtual %p)\n",
		efx->type->mem_bar, efx->membase_phys, efx->type->mem_map_size,
		efx->membase);

	return 0;

 fail4:
	release_mem_region(efx->membase_phys, efx->type->mem_map_size);
 fail3:
	efx->membase_phys = 0UL;
	/* fall-thru */
 fail2:
	pci_disable_device(efx->pci_dev);
 fail1:
	return rc;
}

static void efx_fini_io(struct efx_nic *efx)
{
	EFX_LOG(efx, "shutting down I/O\n");

	if (efx->membase) {
		iounmap(efx->membase);
		efx->membase = NULL;
	}

	if (efx->membase_phys) {
#if !defined(EFX_HAVE_MSIX_TABLE_RESERVED)
		pci_release_region(efx->pci_dev, efx->type->mem_bar);
#else
		release_mem_region(efx->membase_phys, efx->type->mem_map_size);
#endif
		efx->membase_phys = 0UL;
	}

	pci_disable_device(efx->pci_dev);
}

/* Probe the number and type of interrupts we are able to obtain. */
static int efx_probe_interrupts(struct efx_nic *efx)
{
	struct msix_entry xentries[EFX_MAX_CHANNELS];
	int rc, i;

	/* Select number of used RSS queues */
	/* TODO: Can we react to CPU hotplug? */
	if (rss_cpus == 0)
		rss_cpus = num_online_cpus();

	efx->rss_queues = 1;
	if (efx->interrupt_mode == EFX_INT_MODE_MSIX) {
		unsigned int max_channel = efx->type->phys_addr_channels - 1;

		BUG_ON(!pci_find_capability(efx->pci_dev, PCI_CAP_ID_MSIX));
		efx->rss_queues = min(max_channel + 1, rss_cpus);
		efx->rss_queues = min(efx->rss_queues, EFX_MAX_CHANNELS);
	}

	/* Determine how many RSS queues we can use, and mark channels
	 * with the appropriate interrupt state */
	if (efx->interrupt_mode == EFX_INT_MODE_MSIX) {
		/* Build MSI request structure */
		for (i = 0; i < efx->rss_queues; i++)
			xentries[i].entry = i;

		/* Request maximum number of MSI interrupts */
		rc = pci_enable_msix(efx->pci_dev, xentries, efx->rss_queues);
		if (rc > 0) {
			EFX_BUG_ON_PARANOID(rc >= efx->rss_queues);
			efx->rss_queues = rc;
			rc = pci_enable_msix(efx->pci_dev, xentries,
					     efx->rss_queues);
		}
		if (rc == 0) {
			for (i = 0; i < efx->rss_queues; i++) {
				efx->channel[i].has_interrupt = 1;
				efx->channel[i].irq = xentries[i].vector;
			}
		} else {
			/* Fall back to single channel MSI */
			efx->interrupt_mode = EFX_INT_MODE_MSI;
			EFX_ERR(efx, "could not enable MSI-X\n");
		}
	}

	/* Try single interrupt MSI */
	if (efx->interrupt_mode == EFX_INT_MODE_MSI) {
		efx->rss_queues = 1;
		rc = pci_enable_msi(efx->pci_dev);
		if (rc == 0) {
			efx->channel[0].irq = efx->pci_dev->irq;
			efx->channel[0].has_interrupt = 1;
		} else {
			EFX_ERR(efx, "could not enable MSI\n");
			efx->interrupt_mode = EFX_INT_MODE_LEGACY;
		}
	}

	/* Assume legacy interrupts */
	if (efx->interrupt_mode == EFX_INT_MODE_LEGACY) {
		/* Every channel is interruptible */
		for (i = 0; i < EFX_MAX_CHANNELS; i++)
			efx->channel[i].has_interrupt = 1;
		efx->legacy_irq = efx->pci_dev->irq;
	}

	return 0;
}

static void efx_remove_interrupts(struct efx_nic *efx)
{
	struct efx_channel *channel;

	/* Remove MSI/MSI-X interrupts */
	efx_for_each_channel_with_interrupt(channel, efx)
		channel->irq = 0;
	pci_disable_msi(efx->pci_dev);
	pci_disable_msix(efx->pci_dev);

	/* Remove legacy interrupt */
	efx->legacy_irq = 0;
}

/* Select number of used resources
 * Should be called after probe_interrupts()
 */
static int efx_select_used(struct efx_nic *efx)
{
	struct efx_tx_queue *tx_queue;
	struct efx_rx_queue *rx_queue;
	int i;

	/* TX queues.  One per port per channel with TX capability
	 * (more than one per port won't work on Linux, due to out
	 *  of order issues... but will be fine on Solaris)
	 */
	tx_queue = &efx->tx_queue[0];

	/* Perform this for each channel with TX capabilities.
	 * At the moment, we only support a single TX queue
	 */
	tx_queue->used = 1;
	if ((!EFX_INT_MODE_USE_MSI(efx)) && separate_tx_and_rx_channels)
		tx_queue->channel = &efx->channel[1];
	else
		tx_queue->channel = &efx->channel[0];
	tx_queue->channel->used_flags |= EFX_USED_BY_TX;
	tx_queue++;

	/* RX queues.  Each has a dedicated channel. */
	for (i = 0; i < EFX_MAX_RX_QUEUES; i++) {
		rx_queue = &efx->rx_queue[i];

		if (i < efx->rss_queues) {
			rx_queue->used = 1;
			/* If we allow multiple RX queues per channel
			 * we need to decide that here
			 */
			rx_queue->channel = &efx->channel[rx_queue->queue];
			rx_queue->channel->used_flags |= EFX_USED_BY_RX;
			rx_queue++;
		}
	}
	return 0;
}

static int efx_probe_nic(struct efx_nic *efx)
{
	int rc;

	EFX_LOG(efx, "creating NIC\n");

	/* Carry out hardware-type specific initialisation */
	rc = falcon_probe_nic(efx);
	if (rc)
		goto fail1;

	/* Determine the number of channels and RX queues by trying to hook
	 * in MSI-X interrupts. */
	rc = efx_probe_interrupts(efx);
	if (rc)
		goto fail2;

	/* Determine number of RX queues and TX queues */
	rc = efx_select_used(efx);
	if (rc)
		goto fail3;

	/* Register debugfs entries */
	rc = efx_init_debugfs_nic(efx);
	if (rc)
		goto fail4;
	/* Initialise the interrupt moderation settings */
	efx_init_irq_moderation(efx, tx_irq_mod_usec, rx_irq_mod_usec);

	return 0;

 fail4:
	/* fall-thru */
 fail3:
	efx_remove_interrupts(efx);
 fail2:
	falcon_remove_nic(efx);
 fail1:
	return rc;
}

static void efx_remove_nic(struct efx_nic *efx)
{
	EFX_LOG(efx, "destroying NIC\n");

	efx_remove_interrupts(efx);
	falcon_remove_nic(efx);

	efx_fini_debugfs_nic(efx);
}

/**************************************************************************
 *
 * NIC startup/shutdown
 *
 *************************************************************************/

static int efx_probe_all(struct efx_nic *efx)
{
	struct efx_channel *channel;
	int rc;

	/* Create NIC */
	rc = efx_probe_nic(efx);
	if (rc) {
		EFX_ERR(efx, "failed to create NIC\n");
		goto fail1;
	}

	/* Create port */
	rc = efx_probe_port(efx);
	if (rc) {
		EFX_ERR(efx, "failed to create port\n");
		goto fail2;
	}

	/* Create channels */
	efx_for_each_channel(channel, efx) {
		rc = efx_probe_channel(channel);
		if (rc) {
			EFX_ERR(efx, "failed to create channel %d\n",
				channel->channel);
			goto fail3;
		}
	}

	return 0;

 fail3:
	efx_for_each_channel(channel, efx)
		efx_remove_channel(channel);
 fail2:
	efx_remove_port(efx);
 fail1:
	return rc;
}

/* Called after previous invocation(s) of efx_stop_all, restarts the
 * port, kernel transmit queue, NAPI processing and hardware interrupts.
 * This function is safe to call multiple times when the NIC is in any
 * state. */
static void efx_start_all(struct efx_nic *efx)
{
	struct efx_channel *channel;

	ASSERT_RTNL();

	/* Check that it is appropriate to restart the interface. All
	 * of these flags are safe to read under just the rtnl lock */
	if (efx->port_enabled)
		return;
	if ((efx->state != STATE_RUNNING) && (efx->state != STATE_INIT))
		return;
	if (efx->net_dev_registered && !netif_running(efx->net_dev))
		return;

	/* Mark the port as enabled so port reconfigurations can start, then
	 * restart the transmit interface early so the watchdog timer stops */
	efx_start_port(efx);
	efx_wake_queue(efx);

	efx_for_each_channel(channel, efx)
		efx_start_channel(channel);

	falcon_enable_interrupts(efx);

	/* Start hardware monitor if we're in RUNNING */
	if (efx->state == STATE_RUNNING)
		queue_delayed_work(efx->workqueue, &efx->monitor_work,
				   efx_monitor_interval);
}

/* Flush all delayed work. Should only be called when no more delayed work
 * will be scheduled. This doesn't flush pending online resets (efx_reset),
 * since we're holding the rtnl_lock at this point. */
static void efx_flush_all(struct efx_nic *efx)
{
#if defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
	struct efx_rx_queue *rx_queue;

	/* Make sure the hardware monitor is stopped */
	cancel_delayed_work_sync(&efx->monitor_work);

	/* Ensure that all RX slow refills are complete. */
	efx_for_each_rx_queue(rx_queue, efx) {
		cancel_delayed_work_sync(&rx_queue->work);
	}
#endif

#if defined(EFX_USE_CANCEL_WORK_SYNC)
	/* Stop scheduled port reconfigurations */
	cancel_work_sync(&efx->reconfigure_work);
#endif

#if !defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
	/* Ensure that the hardware monitor and asynchronous port
	 * reconfigurations are complete, which are the only two consumers
	 * of efx->workqueue. Since the hardware monitor runs on a long period,
	 * we put in some effort to cancel the delayed work safely rather
	 * than just flushing the queue twice (which is guaranteed to flush
	 * all the work since both efx_monitor and efx_reconfigure_work disarm
	 * if !efx->port_enabled. */
	if (timer_pending(&efx->monitor_work.timer))
		cancel_delayed_work(&efx->monitor_work);
	flush_workqueue(efx->workqueue);
	if (timer_pending(&efx->monitor_work.timer))
		cancel_delayed_work(&efx->monitor_work);
	flush_workqueue(efx->workqueue);

	/* efx_rx_work will disarm if !channel->enabled, so we can just
	 * flush the refill workqueue twice as well. */
	flush_workqueue(efx->refill_workqueue);
	flush_workqueue(efx->refill_workqueue);
#endif
}

/* Quiesce hardware and software without bringing the link down.
 * Safe to call multiple times, when the nic and interface is in any
 * state. The caller is guaranteed to subsequently be in a position
 * to modify any hardware and software state they see fit without
 * taking locks. */
static void efx_stop_all(struct efx_nic *efx)
{
	struct efx_channel *channel;

	ASSERT_RTNL();

	/* port_enabled can be read safely under the rtnl lock */
	if (!efx->port_enabled)
		return;

	/* Disable interrupts and wait for ISR to complete */
	falcon_disable_interrupts(efx);
	if (efx->legacy_irq)
		synchronize_irq(efx->legacy_irq);
	efx_for_each_channel_with_interrupt(channel, efx)
		if (channel->irq)
			synchronize_irq(channel->irq);

	/* Stop all synchronous port reconfigurations. */
	efx_stop_port(efx);

	/* Stop all NAPI processing and synchronous rx refills */
	efx_for_each_channel(channel, efx)
		efx_stop_channel(channel);

	/* Flush reconfigure_work, refill_workqueue, monitor_work */
	efx_flush_all(efx);

	/* Stop the kernel transmit interface late, so the watchdog
	 * timer isn't ticking over the flush */
	efx_stop_queue(efx);
	if (efx->net_dev_registered) {
		netif_tx_lock_bh(efx->net_dev);
		netif_tx_unlock_bh(efx->net_dev);
	}
}

static void efx_remove_all(struct efx_nic *efx)
{
	struct efx_channel *channel;

	efx_for_each_channel(channel, efx)
		efx_remove_channel(channel);
	efx_remove_port(efx);
	efx_remove_nic(efx);
}

static int efx_run_selftests(struct efx_nic *efx)
{
	struct efx_self_tests tests;
	unsigned modes = efx->startup_loopbacks & efx->loopback_modes;
	int rc;

	rc = efx_online_test(efx, &tests);
	if (rc) {
		EFX_ERR(efx, "failed self-tests with interrupt_mode of %s\n",
			INT_MODE(efx));
		goto fail;
	}

	if (onload_offline_selftest && modes) {
		/* Run offline self test */
		EFX_LOG(efx, "performing on-load offline self-tests\n");
		rc = efx_offline_test(efx, &tests, modes);
		EFX_LOG(efx, "%s on-load offline self-tests\n",
			rc ? "FAILED" : "PASSED");
		if (rc)
			goto fail;
	}

	return 0;

 fail:
	EFX_ERR(efx, "self-tests failed. Given up!\n");
	if (allow_load_on_failure)
		rc = 0;

	return rc;
}

int efx_flush_queues(struct efx_nic *efx)
{
	int rc;

	ASSERT_RTNL();

	efx_stop_all(efx);

	/* We can't just flush the tx queues because the event queues
	 * may contain tx completions from that queue. Just flush everything */
	efx_fini_channels(efx);
	rc = efx_init_channels(efx);
	if (rc) {
		efx_schedule_reset(efx, RESET_TYPE_DISABLE);
		return rc;
	}

	efx_start_all(efx);

	return 0;
}

/**************************************************************************
 *
 * Interrupt moderation
 *
 **************************************************************************/

/* Set interrupt moderation parameters */
void efx_init_irq_moderation(struct efx_nic *efx, int tx_usecs, int rx_usecs)
{
	struct efx_tx_queue *tx_queue;
	struct efx_rx_queue *rx_queue;

	ASSERT_RTNL();

	efx_for_each_tx_queue(tx_queue, efx)
		tx_queue->channel->irq_moderation = tx_usecs;

	efx_for_each_rx_queue(rx_queue, efx)
		rx_queue->channel->irq_moderation = rx_usecs;
}

/**************************************************************************
 *
 * Hardware monitor
 *
 **************************************************************************/

/* Run periodically off the general workqueue. Serialised against
 * efx_reconfigure_port via the mac_lock */
static void efx_monitor(struct work_struct *data)
{
#if !defined(EFX_NEED_WORK_API_WRAPPERS)
	struct efx_nic *efx = container_of(data, struct efx_nic,
					   monitor_work.work);
#else
	struct efx_nic *efx = container_of(data, struct efx_nic,
					   monitor_work);
#endif
	int rc = 0;

	EFX_TRACE(efx, "hardware monitor executing on CPU %d\n",
		  raw_smp_processor_id());


#if !defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
	/* Without cancel_delayed_work_sync(), we have to make sure that
	 * we don't rearm when port_enabled == 0 */
	mutex_lock(&efx->mac_lock);
	if (!efx->port_enabled) {
		mutex_unlock(&efx->mac_lock);
		return;
	}

	rc = efx->mac_op->check_hw(efx);
#else
	/* If the mac_lock is already held then it is likely a port
	 * reconfiguration is already in place, which will likely do
	 * most of the work of check_hw() anyway. */
	if (!mutex_trylock(&efx->mac_lock)) {
		queue_delayed_work(efx->workqueue, &efx->monitor_work,
				   efx_monitor_interval);
		return;
	}

	if (efx->port_enabled)
		rc = efx->mac_op->check_hw(efx);
#endif
	mutex_unlock(&efx->mac_lock);

	if (rc) {
		if (monitor_reset) {
			EFX_ERR(efx, "hardware monitor detected a fault: "
				"triggering reset\n");
			efx_schedule_reset(efx, RESET_TYPE_MONITOR);
		} else {
			EFX_ERR(efx, "hardware monitor detected a fault, "
				"skipping reset\n");
		}
	}

	queue_delayed_work(efx->workqueue, &efx->monitor_work,
			   efx_monitor_interval);
}

/**************************************************************************
 *
 * ioctls
 *
 *************************************************************************/

/* Net device ioctl
 * Context: process, rtnl_lock() held.
 */
static int efx_ioctl(struct net_device *net_dev, struct ifreq *ifr, int cmd)
{
	struct efx_nic *efx = net_dev->priv;
	int rc;

	ASSERT_RTNL();

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
		rc = generic_mii_ioctl(&efx->mii, if_mii(ifr), cmd, NULL);
		break;
	case SIOCSMIIREG:
		rc = generic_mii_ioctl(&efx->mii, if_mii(ifr), cmd, NULL);
		efx_reconfigure_port(efx, 0);
		break;
	default:
		rc = -EOPNOTSUPP;
	}

	return rc;
}

/**************************************************************************
 *
 * NAPI interface
 *
 **************************************************************************/

/* Allocate the NAPI dev's.
 * Called after we know how many channels there are.
 */
static int efx_init_napi(struct efx_nic *efx)
{
	struct efx_channel *channel;
	int rc;

	ASSERT_RTNL();

	/* Allocate the NAPI dev for the port */
	efx->net_dev = alloc_etherdev(0);
	if (!efx->net_dev) {
		rc = -ENOMEM;
		goto err;
	}
	efx->net_dev->priv = efx;
	efx->mii.dev = efx->net_dev;

	/* Set features based on module parameters and DMA mask.
	 * Enable DMA to ZONE_HIGHMEM if the NIC can access all memory
	 * directly.  This only has an effect on 32-bit systems and
	 * PAE on x86 limits memory to 64GB so 40 bits is plenty to
	 * address everything.  If the device can't address 40 bits
	 * then it's safest to turn NETIF_F_HIGHDMA off because this
	 * might be a PAE system with more than 4G of RAM and a 32-bit
	 * NIC.  The use of EFX_DMA_MASK is to eliminate compiler
	 * warnings on platforms where dma_addr_t is 32-bit.  We
	 * assume that in those cases we can access all memory
	 * directly if our DMA mask is all ones. */
	efx->net_dev->features |= NETIF_F_IP_CSUM | NETIF_F_SG;
	if (efx->dma_mask >= EFX_DMA_MASK(DMA_40BIT_MASK))
		efx->net_dev->features |= NETIF_F_HIGHDMA;

	/* Copy MAC address */
	memcpy(&efx->net_dev->dev_addr, efx->mac_address, ETH_ALEN);

	/* Allocate the per channel devs */
	efx_for_each_channel(channel, efx) {
#if !defined(EFX_HAVE_OLD_NAPI)
		channel->napi_dev = efx->net_dev;
#else
		channel->napi_dev = alloc_etherdev(0);
		if (!channel->napi_dev) {
			rc = -ENOMEM;
			goto err;
		}
		channel->napi_dev->priv = channel;
		atomic_set(&channel->napi_dev->refcnt, 1);
#endif
	}

	return 0;
 err:
	efx_fini_napi(efx);
	return rc;
}

/* Free the NAPI state for the port and channels */
static void efx_fini_napi(struct efx_nic *efx)
{
	struct efx_channel *channel;

	ASSERT_RTNL();

	efx_for_each_channel(channel, efx) {
		/* Finish per channel NAPI */
#if defined(EFX_HAVE_OLD_NAPI)
		if (channel->napi_dev) {
			channel->napi_dev->priv = NULL;
			free_netdev(channel->napi_dev);
		}
#endif
		channel->napi_dev = NULL;
	}

	/* Finish port NAPI */
	if (efx->net_dev) {
		efx->net_dev->priv = NULL;
		free_netdev(efx->net_dev);
		efx->net_dev = NULL;
	}
}

/**************************************************************************
 *
 * Kernel netpoll interface
 *
 *************************************************************************/

#ifdef CONFIG_NET_POLL_CONTROLLER

/* Although in the common case interrupts will be disabled, this is not
 * guaranteed. However, all our work happens inside the NAPI callback,
 * so no locking is required.
 */
static void efx_netpoll(struct net_device *net_dev)
{
	struct efx_nic *efx = net_dev->priv;
	struct efx_channel *channel;

	efx_for_each_channel_with_interrupt(channel, efx)
		efx_schedule_channel(channel);
}

#endif

/**************************************************************************
 *
 * Kernel net device interface
 *
 *************************************************************************/

/* Context: process, rtnl_lock() held. */
static int efx_net_open(struct net_device *net_dev)
{
	struct efx_nic *efx = net_dev->priv;
	ASSERT_RTNL();

	EFX_LOG(efx, "opening device %s on CPU %d\n", net_dev->name,
		raw_smp_processor_id());
	efx_start_all(efx);
	return 0;
}

/* Context: process, rtnl_lock() held.
 * Note that the kernel will ignore our return code; this method
 * should really be a void.
 */
static int efx_net_stop(struct net_device *net_dev)
{
	struct efx_nic *efx = net_dev->priv;
	int rc;

	EFX_LOG(efx, "closing %s on CPU %d\n", net_dev->name,
		raw_smp_processor_id());

	/* Stop device and flush all the channels */
	efx_stop_all(efx);
	efx_fini_channels(efx);
	rc = efx_init_channels(efx);
	if (rc)
		efx_schedule_reset(efx, RESET_TYPE_DISABLE);

	return 0;
}

/* Context: process, dev_base_lock held, non-blocking.
 * Statistics are taken directly from the MAC.
 */
static struct net_device_stats *efx_net_stats(struct net_device *net_dev)
{
	struct efx_nic *efx = net_dev->priv;
	struct efx_mac_stats *mac_stats = &efx->mac_stats;
	struct net_device_stats *stats = &efx->stats;

	if (!spin_trylock(&efx->stats_lock))
		return stats;
	if (efx->state == STATE_RUNNING)
		efx->mac_op->update_stats(efx);
	spin_unlock(&efx->stats_lock);

	stats->rx_packets = mac_stats->rx_packets;
	stats->tx_packets = mac_stats->tx_packets;
	stats->rx_bytes = mac_stats->rx_bytes;
	stats->tx_bytes = mac_stats->tx_bytes;
	stats->tx_errors = mac_stats->tx_bad;
	stats->multicast = mac_stats->rx_multicast;
	stats->collisions = mac_stats->tx_collision;
	stats->rx_length_errors = mac_stats->rx_gtjumbo;
	stats->rx_over_errors = mac_stats->rx_overflow;
	stats->rx_crc_errors = mac_stats->rx_bad;
	stats->rx_frame_errors = mac_stats->rx_align_error;
	stats->rx_fifo_errors = 0;
	stats->rx_missed_errors = mac_stats->rx_missed;
	stats->rx_errors = (stats->rx_length_errors +
			    stats->rx_over_errors +
			    stats->rx_crc_errors +
			    stats->rx_frame_errors +
			    stats->rx_fifo_errors +
			    stats->rx_missed_errors +
			    mac_stats->rx_symbol_error);
	stats->tx_aborted_errors = 0;
	stats->tx_carrier_errors = 0;
	stats->tx_fifo_errors = 0;
	stats->tx_heartbeat_errors = 0;
	stats->tx_window_errors = 0;

	return stats;
}

/* Context: netif_tx_lock held, BHs disabled. */
static void efx_watchdog(struct net_device *net_dev)
{
	struct efx_nic *efx = net_dev->priv;

	EFX_ERR(efx, "TX stuck with stop_count=%d port_enabled=%d: %s\n",
		atomic_read(&efx->netif_stop_count), efx->port_enabled,
		monitor_reset ? "resetting channels" : "skipping reset");

	if (monitor_reset)
		efx_schedule_reset(efx, RESET_TYPE_MONITOR);
}


/* Context: process, rtnl_lock() held. */
static int efx_change_mtu(struct net_device *net_dev, int new_mtu)
{
	struct efx_nic *efx = net_dev->priv;
	int rc = 0;

	ASSERT_RTNL();

	if (new_mtu > EFX_MAX_MTU)
		return -EINVAL;

	efx_stop_all(efx);

	/* Ask driverlink client if we can change MTU */
	rc = EFX_DL_CALLBACK(efx, request_mtu, new_mtu);
	if (rc) {
		EFX_ERR(efx, "MTU change vetoed by driverlink %s driver\n",
			efx->dl_cb_dev.request_mtu->driver->name);
		goto out;
	}

	EFX_LOG(efx, "changing MTU to %d\n", new_mtu);

	efx_fini_channels(efx);
	net_dev->mtu = new_mtu;
	rc = efx_init_channels(efx);
	if (rc)
		goto fail;

	/* Reconfigure the MAC */
	efx_reconfigure_port(efx, 1);

	/* Notify driverlink client of new MTU */
	EFX_DL_CALLBACK(efx, mtu_changed, new_mtu);

	efx_start_all(efx);

 out:
	return rc;

 fail:
	efx_schedule_reset(efx, RESET_TYPE_DISABLE);
	return rc;
}

static int efx_set_mac_address(struct net_device *net_dev, void *data)
{
	struct efx_nic *efx = net_dev->priv;
	struct sockaddr *addr = data;
	char *new_addr = addr->sa_data;

	ASSERT_RTNL();

	if (!is_valid_ether_addr(new_addr)) {
		DECLARE_MAC_BUF(mac);
		EFX_ERR(efx, "invalid ethernet MAC address requested: %s\n",
			print_mac(mac, new_addr));
		return -EINVAL;
	}

	memcpy(net_dev->dev_addr, new_addr, net_dev->addr_len);

	/* Reconfigure the MAC */
	efx_reconfigure_port(efx, 1);

	return 0;
}

/* Context: netif_tx_lock held, BHs disabled. */
static void efx_set_multicast_list(struct net_device *net_dev)
{
	struct efx_nic *efx = net_dev->priv;
	struct dev_mc_list *mc_list = net_dev->mc_list;
	union efx_multicast_hash *mc_hash = &efx->multicast_hash;
	unsigned long flags __attribute__ ((unused));
	int promiscuous;
	u32 crc;
	int bit;
	int i;

	/* Set per-MAC promiscuity flag and reconfigure MAC if necessary */
	promiscuous = (net_dev->flags & IFF_PROMISC) ? 1 : 0;
	if (efx->promiscuous != promiscuous) {
		if (efx->port_enabled) {
			efx->promiscuous = promiscuous;
			queue_work(efx->workqueue, &efx->reconfigure_work);
		}
	}

	/* Build multicast hash table */
	if (promiscuous || (net_dev->flags & IFF_ALLMULTI)) {
		memset(mc_hash, 0xff, sizeof(*mc_hash));
	} else {
		memset(mc_hash, 0x00, sizeof(*mc_hash));
		for (i = 0; i < net_dev->mc_count; i++) {
			crc = ether_crc_le(ETH_ALEN, mc_list->dmi_addr);
			bit = (crc & ((1 << EFX_MCAST_HASH_BITS) - 1));
			set_bit_le(bit, (void *)mc_hash);
			mc_list = mc_list->next;
		}
	}

	/* Create and activate new global multicast hash table */
	falcon_set_multicast_hash(efx);
}

/* Handle net device notifier events */
static int efx_netdev_event(struct notifier_block *this,
			    unsigned long event, void *ptr)
{
	struct net_device *net_dev = (struct net_device *)ptr;

	if (net_dev->open == efx_net_open && event == NETDEV_CHANGENAME) {
		struct efx_nic *efx = net_dev->priv;

		strcpy(efx->name, net_dev->name);
		efx_fini_debugfs_netdev(net_dev);
		efx_init_debugfs_netdev(net_dev);
	}

	return NOTIFY_DONE;
}

static struct notifier_block efx_netdev_notifier = {
	.notifier_call = efx_netdev_event,
};

static int efx_register_netdev(struct efx_nic *efx)
{
	struct net_device *net_dev = efx->net_dev;
	int rc;

	net_dev->watchdog_timeo = 5 * HZ;
	net_dev->irq = efx->pci_dev->irq;
	net_dev->open = efx_net_open;
	net_dev->stop = efx_net_stop;
	net_dev->get_stats = efx_net_stats;
	net_dev->tx_timeout = &efx_watchdog;
	net_dev->hard_start_xmit = efx_hard_start_xmit;
	net_dev->do_ioctl = efx_ioctl;
	net_dev->change_mtu = efx_change_mtu;
	net_dev->set_mac_address = efx_set_mac_address;
	net_dev->set_multicast_list = efx_set_multicast_list;
#ifdef CONFIG_NET_POLL_CONTROLLER
	net_dev->poll_controller = efx_netpoll;
#endif
	SET_NETDEV_DEV(net_dev, &efx->pci_dev->dev);
	SET_ETHTOOL_OPS(net_dev, &efx_ethtool_ops);

	/* Always start with carrier off; PHY events will detect the link */
	netif_carrier_off(efx->net_dev);

	BUG_ON(efx->net_dev_registered);

	/* Clear MAC statistics */
	efx->mac_op->update_stats(efx);
	memset(&efx->mac_stats, 0, sizeof(efx->mac_stats));

	rc = register_netdev(net_dev);
	if (rc) {
		EFX_ERR(efx, "could not register net dev\n");
		return rc;
	}
	strcpy(efx->name, net_dev->name);

	/* Create debugfs symlinks */
	rc = efx_init_debugfs_netdev(net_dev);
	if (rc) {
		EFX_ERR(efx, "failed to init net dev debugfs\n");
		unregister_netdev(efx->net_dev);
		return rc;
	}

	/* Allow link change notifications to be sent to the operating
	 * system.  The must happen after register_netdev so that
	 * there are no outstanding link changes if that call fails.
	 * It must happen before efx_reconfigure_port so that the
	 * initial state of the link is reported. */
	mutex_lock(&efx->mac_lock);
	efx->net_dev_registered = 1;
	mutex_unlock(&efx->mac_lock);

	/* Safety net: in case we don't get a PHY event */
	rtnl_lock();
	efx_reconfigure_port(efx, 1);
	rtnl_unlock();

	EFX_LOG(efx, "registered\n");

	return 0;
}

static void efx_unregister_netdev(struct efx_nic *efx)
{
	int was_registered = efx->net_dev_registered;
	struct efx_tx_queue *tx_queue;

	if (!efx->net_dev)
		return;

	BUG_ON(efx->net_dev->priv != efx);

	/* SFC Bug 5356: Ensure that no more link status notifications get
	 * sent to the stack.  Bad things happen if there's an
	 * outstanding notification after the net device is freed, but
	 * they only get flushed out by unregister_netdev, not by
	 * free_netdev. */
	mutex_lock(&efx->mac_lock);
	efx->net_dev_registered = 0;
	mutex_unlock(&efx->mac_lock);

	/* Free up any skbs still remaining. This has to happen before
	 * we try to unregister the netdev as running their destructors
	 * may be needed to get the device ref. count to 0. */
	efx_for_each_tx_queue(tx_queue, efx)
		efx_release_tx_buffers(tx_queue);

	if (was_registered) {
		strlcpy(efx->name, pci_name(efx->pci_dev), sizeof(efx->name));
		efx_fini_debugfs_netdev(efx->net_dev);
		unregister_netdev(efx->net_dev);
	}
}

/**************************************************************************
 *
 * Device reset and suspend
 *
 **************************************************************************/

/* This suspends the device (and acquires the suspend lock) without
 * flushing the descriptor queues.  It is included for the convenience
 * of the driverlink layer.
 */
void efx_suspend(struct efx_nic *efx)
{
	EFX_LOG(efx, "suspending operations\n");

	down(&efx->suspend_lock);

	rtnl_lock();
	efx_stop_all(efx);
}

void efx_resume(struct efx_nic *efx)
{
	EFX_LOG(efx, "resuming operations\n");

	efx_start_all(efx);
	rtnl_unlock();

	up(&efx->suspend_lock);
}

/* The final hardware and software finalisation before reset.
 * This function does not handle serialisation with the kernel, it
 * assumes the caller has done this */
static int efx_reset_down(struct efx_nic *efx, struct ethtool_cmd *ecmd)
{
	int rc;

	ASSERT_RTNL();

	rc = efx->mac_op->get_settings(efx, ecmd);
	if (rc) {
		EFX_ERR(efx, "could not back up PHY settings\n");
		goto fail;
	}

	efx_fini_channels(efx);
	return 0;

 fail:
	return rc;
}

/* The first part of software initialisation after a hardware reset
 * This function does not handle serialisation with the kernel, it
 * assumes the caller has done this */
static int efx_reset_up(struct efx_nic *efx, struct ethtool_cmd *ecmd)
{
	int rc;

	rc = efx_init_channels(efx);
	if (rc)
		goto fail1;

	/* In an INVISIBLE_RESET there might not be a link state transition,
	 * so we push the multicast list here. */
	falcon_set_multicast_hash(efx);

	/* Restore MAC and PHY settings. */
	rc = efx->mac_op->set_settings(efx, ecmd);
	if (rc) {
		EFX_ERR(efx, "could not restore PHY settings\n");
		goto fail2;
	}

	return 0;

 fail2:
	efx_fini_channels(efx);
 fail1:
	return rc;
}

/* Reset the NIC as transparently as possible. Do not reset the PHY
 * Note that the reset may fail, in which case the card will be left
 * in a most-probably-unusable state.
 *
 * This function will sleep.  You cannot reset from within an atomic
 * state; use efx_schedule_reset() instead.
 */
static int efx_reset(struct efx_nic *efx)
{
	struct ethtool_cmd ecmd;
	unsigned long flags __attribute__ ((unused));
	enum reset_type method = efx->reset_pending;
	int rc;

	efx_dl_reset_lock();

	rc = down_interruptible(&efx->suspend_lock);
	if (rc) {
		EFX_ERR(efx, "reset aborted by signal\n");
		goto unlock_dl_lock;
	}

	/* We've got suspend_lock, which means we can only be in
	 * STATE_RUNNING or STATE_FINI. Don't clear
	 * efx->reset_pending, since this flag indicates that we
	 * should retry device initialisation.
	 */
	if (efx->state != STATE_RUNNING) {
		EFX_INFO(efx, "scheduled reset quenched. NIC not RUNNING\n");
		goto unlock_suspend_lock;
	}

	/* Notify driverlink clients of imminent reset. */
	efx_dl_reset_suspend(efx);
	rtnl_lock();

	efx->state = STATE_RESETTING;
	EFX_INFO(efx, "resetting (%s)\n", RESET_TYPE(method));

	/* The net_dev->get_stats handler is quite slow, and will fail
	 * if a fetch is pending over reset. Serialise against it. */
	spin_lock(&efx->stats_lock);
	spin_unlock(&efx->stats_lock);

	efx_stop_all(efx);
	mutex_lock(&efx->mac_lock);

	rc = efx_reset_down(efx, &ecmd);
	if (rc)
		goto fail1;
	falcon_fini_nic(efx);

	rc = falcon_reset_hw(efx, method);
	if (rc) {
		EFX_ERR(efx, "failed to reset hardware\n");
		goto fail2;
	}

	/* Allow resets to be rescheduled. */
	efx->reset_pending = RESET_TYPE_NONE;

	/* Reinitialise bus-mastering, which may have been turned off before
	 * the reset was scheduled. This is still appropriate, even in the
	 * RESET_TYPE_DISABLE since this driver generally assumes the hardware
	 * can respond to requests. */
	pci_set_master(efx->pci_dev);

	/* Reinitialise device. This is appropriate in the RESET_TYPE_DISABLE
	 * case so the driver can talk to external SRAM */
	rc = falcon_init_nic(efx);
	if (rc) {
		EFX_ERR(efx, "failed to initialise NIC\n");
		goto fail3;
	}

	/* Leave device stopped if necessary */
	if (method == RESET_TYPE_DISABLE) {
		/* Reinitialise the device anyway so the driver unload sequence
		 * can talk to the external SRAM */
		(void) falcon_init_nic(efx);
		rc = -EIO;
		goto fail4;
	}

	rc = efx_reset_up(efx, &ecmd);
	if (rc)
		goto fail5;

	mutex_unlock(&efx->mac_lock);
	efx_reconfigure_port(efx, 1);
	EFX_LOG(efx, "reset complete\n");

	efx->state = STATE_RUNNING;
	efx_start_all(efx);

	rtnl_unlock();

	goto notify;

 fail5:
 fail4:
 fail3:
 fail2:
 fail1:
	EFX_ERR(efx, "has been disabled\n");
	efx->state = STATE_DISABLED;

	/* Remove the net_dev */
	mutex_unlock(&efx->mac_lock);
	rtnl_unlock();
	efx_unregister_netdev(efx);
	efx_fini_port(efx);

 notify:
	/* Notify driverlink clients of completed reset */
	efx_dl_reset_resume(efx, (rc == 0));

 unlock_suspend_lock:
	up(&efx->suspend_lock);

 unlock_dl_lock:
	efx_dl_reset_unlock();

	return rc;
}

/* The worker thread exists so that code that cannot sleep can
 * schedule a reset for later.
 */
static void efx_reset_work(struct work_struct *data)
{
	struct efx_nic *nic = container_of(data, struct efx_nic, reset_work);

	efx_reset(nic);
}

void efx_schedule_reset(struct efx_nic *efx, enum reset_type type)
{
	enum reset_type method;

	if (efx->reset_pending != RESET_TYPE_NONE) {
		EFX_INFO(efx, "quenching already scheduled reset\n");
		return;
	}

	switch (type) {
	case RESET_TYPE_INVISIBLE:
	case RESET_TYPE_ALL:
	case RESET_TYPE_WORLD:
	case RESET_TYPE_DISABLE:
		method = type;
		break;
	case RESET_TYPE_RX_RECOVERY:
	case RESET_TYPE_RX_DESC_FETCH:
	case RESET_TYPE_TX_DESC_FETCH:
		method = RESET_TYPE_INVISIBLE;
		break;
	default:
		method = RESET_TYPE_ALL;
		break;
	}

	if (method != type)
		EFX_LOG(efx, "scheduling %s reset for %s\n",
			RESET_TYPE(method), RESET_TYPE(type));
	else
		EFX_LOG(efx, "scheduling %s reset\n", RESET_TYPE(method));

	efx->reset_pending = method;

#if !defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
	queue_work(efx->reset_workqueue, &efx->reset_work);
#else
	queue_work(efx->workqueue, &efx->reset_work);
#endif
}

/**************************************************************************
 *
 * List of NICs we support
 *
 **************************************************************************/

enum efx_type_index {
	EFX_TYPE_FALCON_A = 0,
	EFX_TYPE_FALCON_B = 1,
};

static struct efx_nic_type *efx_nic_types[] = {
	[EFX_TYPE_FALCON_A] = &falcon_a_nic_type,
	[EFX_TYPE_FALCON_B] = &falcon_b_nic_type,
};


/* PCI device ID table */
static struct pci_device_id efx_pci_table[] __devinitdata = {
	{EFX_VENDID_SFC, FALCON_A_P_DEVID, PCI_ANY_ID, PCI_ANY_ID,
	 0, 0, EFX_TYPE_FALCON_A},
	{EFX_VENDID_SFC, FALCON_B_P_DEVID, PCI_ANY_ID, PCI_ANY_ID,
	 0, 0, EFX_TYPE_FALCON_B},
	{0}			/* end of list */
};

/**************************************************************************
 *
 * Dummy PHY/MAC/Board operations
 *
 * Can be used where the MAC does not implement this operation
 * Needed so all function pointers are valid and do not have to be tested
 * before use
 *
 **************************************************************************/
int efx_port_dummy_op_int(struct efx_nic *efx)
{
	return 0;
}
void efx_port_dummy_op_void(struct efx_nic *efx) {}
void efx_port_dummy_op_blink(struct efx_nic *efx, int blink) {}

static struct efx_mac_operations efx_dummy_mac_operations = {
	.init		= efx_port_dummy_op_int,
	.reconfigure	= efx_port_dummy_op_void,
	.fini		= efx_port_dummy_op_void,
};

static struct efx_phy_operations efx_dummy_phy_operations = {
	.init		 = efx_port_dummy_op_int,
	.reconfigure	 = efx_port_dummy_op_void,
	.check_hw        = efx_port_dummy_op_int,
	.fini		 = efx_port_dummy_op_void,
	.clear_interrupt = efx_port_dummy_op_void,
	.reset_xaui      = efx_port_dummy_op_void,
};

/* Dummy board operations */
static int efx_nic_dummy_op_int(struct efx_nic *nic)
{
	return 0;
}

static void efx_nic_dummy_op_void(struct efx_nic *nic) {}

static struct efx_board efx_dummy_board_info = {
	.init    = efx_nic_dummy_op_int,
	.init_leds = efx_port_dummy_op_int,
	.set_fault_led = efx_port_dummy_op_blink,
	.monitor = efx_nic_dummy_op_int,
	.blink = efx_port_dummy_op_blink,
	.fini    = efx_nic_dummy_op_void,
};

/**************************************************************************
 *
 * Data housekeeping
 *
 **************************************************************************/

/* This zeroes out and then fills in the invariants in a struct
 * efx_nic (including all sub-structures).
 */
static int efx_init_struct(struct efx_nic *efx, enum efx_type_index type,
			   struct pci_dev *pci_dev)
{
	struct efx_channel *channel;
	struct efx_tx_queue *tx_queue;
	struct efx_rx_queue *rx_queue;
	int i, rc;

	/* Initialise common structures */
	memset(efx, 0, sizeof(*efx));
	spin_lock_init(&efx->biu_lock);
	spin_lock_init(&efx->phy_lock);
	mutex_init(&efx->spi_lock);
	sema_init(&efx->suspend_lock, 1);
	INIT_WORK(&efx->reset_work, efx_reset_work);
	INIT_DELAYED_WORK(&efx->monitor_work, efx_monitor);
	efx->pci_dev = pci_dev;
	efx->state = STATE_INIT;
	efx->reset_pending = RESET_TYPE_NONE;
	strlcpy(efx->name, pci_name(pci_dev), sizeof(efx->name));
	efx->board_info = efx_dummy_board_info;

	efx->rx_checksum_enabled = 1;
	spin_lock_init(&efx->netif_stop_lock);
	spin_lock_init(&efx->stats_lock);
	mutex_init(&efx->mac_lock);
	efx->mac_op = &efx_dummy_mac_operations;
	efx->phy_op = &efx_dummy_phy_operations;
	INIT_LIST_HEAD(&efx->dl_node);
	INIT_LIST_HEAD(&efx->dl_device_list);
	efx->dl_cb = efx_default_callbacks;
	INIT_WORK(&efx->reconfigure_work, efx_reconfigure_work);
	atomic_set(&efx->netif_stop_count, 1);

	for (i = 0; i < EFX_MAX_CHANNELS; i++) {
		channel = &efx->channel[i];
		channel->efx = efx;
		channel->channel = i;
		channel->evqnum = i;
		channel->work_pending = 0;
	}
	for (i = 0; i < EFX_MAX_TX_QUEUES; i++) {
		tx_queue = &efx->tx_queue[i];
		tx_queue->efx = efx;
		tx_queue->queue = i;
		tx_queue->buffer = NULL;
		tx_queue->channel = &efx->channel[0]; /* for safety */
	}
	for (i = 0; i < EFX_MAX_RX_QUEUES; i++) {
		rx_queue = &efx->rx_queue[i];
		rx_queue->efx = efx;
		rx_queue->queue = i;
		rx_queue->channel = &efx->channel[0]; /* for safety */
		rx_queue->buffer = NULL;
		spin_lock_init(&rx_queue->add_lock);
		INIT_DELAYED_WORK(&rx_queue->work, efx_rx_work);
	}

	efx->type = efx_nic_types[type];

	/* Sanity-check NIC type */
	EFX_BUG_ON_PARANOID(efx->type->txd_ring_mask &
			    (efx->type->txd_ring_mask + 1));
	EFX_BUG_ON_PARANOID(efx->type->rxd_ring_mask &
			    (efx->type->rxd_ring_mask + 1));
	EFX_BUG_ON_PARANOID(efx->type->evq_size &
			    (efx->type->evq_size - 1));
	/* As close as we can get to guaranteeing that we don't overflow */
	EFX_BUG_ON_PARANOID(efx->type->evq_size <
			    (efx->type->txd_ring_mask + 1 +
			     efx->type->rxd_ring_mask + 1));

	EFX_BUG_ON_PARANOID(efx->type->phys_addr_channels > EFX_MAX_CHANNELS);

	/* Higher numbered interrupt modes are less capable! */
	efx->interrupt_mode = max(efx->type->max_interrupt_mode,
				  interrupt_mode);
#if defined(EFX_NEED_DUMMY_MSIX)
	if (efx->interrupt_mode == EFX_INT_MODE_MSIX)
		efx->interrupt_mode = EFX_INT_MODE_MSI;
#endif

	/* Tasks that can fail are last */
	efx->refill_workqueue = create_workqueue("sfc_refill");
	if (!efx->refill_workqueue) {
		rc = -ENOMEM;
		goto fail1;
	}

	efx->workqueue = create_singlethread_workqueue("sfc_work");
	if (!efx->workqueue) {
		rc = -ENOMEM;
		goto fail2;
	}

#if !defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
	efx->reset_workqueue = create_singlethread_workqueue("sfc_reset");
	if (!efx->reset_workqueue) {
		rc = -ENOMEM;
		goto fail3;
	}
#endif

	return 0;

#if !defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
 fail3:
	destroy_workqueue(efx->workqueue);
	efx->workqueue = NULL;
#endif

 fail2:
	destroy_workqueue(efx->refill_workqueue);
	efx->refill_workqueue = NULL;
 fail1:
	return rc;
}

static void efx_fini_struct(struct efx_nic *efx)
{
#if !defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
	if (efx->reset_workqueue) {
		destroy_workqueue(efx->reset_workqueue);
		efx->reset_workqueue = NULL;
	}
#endif
	if (efx->workqueue) {
		destroy_workqueue(efx->workqueue);
		efx->workqueue = NULL;
	}
	if (efx->refill_workqueue) {
		destroy_workqueue(efx->refill_workqueue);
		efx->refill_workqueue = NULL;
	}
}

/**************************************************************************
 *
 * PCI interface
 *
 **************************************************************************/

/* Main body of final NIC shutdown code
 * This is called only at module unload (or hotplug removal).
 */
static void efx_pci_remove_main(struct efx_nic *efx)
{
	ASSERT_RTNL();

	/* Skip everything if we never obtained a valid membase */
	if (!efx->membase)
		return;

	efx_fini_channels(efx);
	efx_fini_port(efx);

	/* Shutdown the board, then the NIC and board state */
	efx->board_info.fini(efx);
	falcon_fini_nic(efx);
	falcon_fini_interrupt(efx);
	efx->board_info.fini(efx);

	/* Tear down NAPI and LRO */
	efx_fini_napi(efx);
	efx_remove_all(efx);
}

/* Final NIC shutdown
 * This is called only at module unload (or hotplug removal).
 */
static void efx_pci_remove(struct pci_dev *pci_dev)
{
	struct efx_nic *efx;

	efx = pci_get_drvdata(pci_dev);
	if (!efx)
		return;

	/* Unregister driver from driverlink layer */
	efx_dl_unregister_nic(efx);

	/* Mark the NIC as fini under both suspend_lock and
	 * rtnl_lock */
	down(&efx->suspend_lock);
	rtnl_lock();
	efx->state = STATE_FINI;
	up(&efx->suspend_lock);

	if (efx->membase) {
		/* Stop the NIC. Since we're in STATE_FINI, this
		 * won't be reversed. */
		if (efx->net_dev_registered)
			dev_close(efx->net_dev);

		/* Release the rtnl lock. Any queued efx_resets()
		 * can now return early [we're in STATE_FINI]. */
		rtnl_unlock();

		efx_unregister_netdev(efx);
		efx_fini_debugfs_channels(efx);

		/* Wait for any scheduled resets to complete. No more will be
		 * scheduled from this point because efx_stop_all() has been
		 * called, we are no longer registered with driverlink, and
		 * the net_device's have been removed. */
#if !defined(EFX_USE_CANCEL_DELAYED_WORK_SYNC)
		flush_workqueue(efx->reset_workqueue);
#else
		flush_workqueue(efx->workqueue);
#endif

		/* Fini and remove all the software state */
		rtnl_lock();
		efx_pci_remove_main(efx);
	}

	rtnl_unlock();

	efx_fini_io(efx);
	EFX_LOG(efx, "shutdown successful\n");

	pci_set_drvdata(pci_dev, NULL);
	efx_fini_struct(efx);
	kfree(efx);
};

/* Main body of NIC initialisation
 * This is called at module load (or hotplug insertion, theoretically).
 */
static int efx_pci_probe_main(struct efx_nic *efx)
{
	int rc;

	/* Do start-of-day initialisation */
	rc = efx_probe_all(efx);
	if (rc)
		goto fail1;

	/* Initialise port/channel net_dev's  */
	rc = efx_init_napi(efx);
	if (rc)
		goto fail2;

	/* Initialise the board */
	rc = efx->board_info.init(efx);
	if (rc) {
		EFX_ERR(efx, "failed to initialise board\n");
		goto fail3;
	}

	/* Initialise device */
	rc = falcon_init_nic(efx);
	if (rc) {
		EFX_ERR(efx, "failed to initialise NIC\n");
		goto fail4;
	}

	/* Initialise port */
	rc = efx_init_port(efx);
	if (rc) {
		EFX_ERR(efx, "failed to initialise port\n");
		goto fail5;
	}

	/* Initialise channels */
	rc = efx_init_channels(efx);
	if (rc)
		goto fail6;

	rc = falcon_init_interrupt(efx);
	if (rc)
		goto fail7;

	/* Start up device - interrupts can occur from this point */
	efx_start_all(efx);

	/* Check basic functionality and set interrupt mode */
	rc = efx_run_selftests(efx);
	if (rc)
		goto fail8;

	/* Stop the NIC */
	efx_stop_all(efx);

	return 0;

 fail8:
	efx_stop_all(efx);
	falcon_fini_interrupt(efx);
 fail7:
	efx_fini_channels(efx);
 fail6:
	efx_fini_port(efx);
 fail5:
	falcon_fini_nic(efx);
 fail4:
	efx->board_info.fini(efx);
 fail3:
	efx_fini_napi(efx);
 fail2:
	efx_remove_all(efx);
 fail1:
	return rc;
}

/* NIC initialisation
 *
 * This is called at module load (or hotplug insertion,
 * theoretically).  It sets up PCI mappings, tests and resets the NIC,
 * sets up and registers the network devices with the kernel and hooks
 * the interrupt service routine.  It does not prepare the device for
 * transmission; this is left to the first time one of the network
 * interfaces is brought up (i.e. efx_net_open).
 */
static int __devinit efx_pci_probe(struct pci_dev *pci_dev,
				   const struct pci_device_id *entry)
{
	struct efx_nic *efx;
	enum efx_type_index type = entry->driver_data;
	int i, rc;

	/* Allocate and initialise a struct efx_nic */
	efx = kmalloc(sizeof(*efx), GFP_KERNEL);
	if (!efx) {
		rc = -ENOMEM;
		goto fail1;
	}
	pci_set_drvdata(pci_dev, efx);
	rc = efx_init_struct(efx, type, pci_dev);
	if (rc)
		goto fail2;

	EFX_INFO(efx, "Solarflare Communications NIC detected\n");

	/* Set up basic I/O (BAR mappings etc) */
	rc = efx_init_io(efx);
	if (rc)
		goto fail3;

	/* From this point on we begin to expose the driver to the OS
	 * to varying degrees, so lets grab the suspend_lock and
	 * rtnl_lock to serialise against efx_reset() and
	 * friends. efx->state is not STATE_RUNNING yet, but we don't
	 * want these tasks to fail, just to block until we drop the
	 * lock
	 */
	rc = down_interruptible(&efx->suspend_lock);
	if (rc) {
		EFX_ERR(efx, "suspend interrupted - aborting\n");
		goto fail4;
	}

	rtnl_lock();

	/* Probe, initialise and start everything. Run self-test */
	for (i = 0; i < 5; i++) {
		rc = efx_pci_probe_main(efx);
		if (rc == 0)
			break;

		/* Retry if a recoverably reset event has been scheduled */
		if ((efx->reset_pending != RESET_TYPE_INVISIBLE) &&
		    (efx->reset_pending != RESET_TYPE_ALL))
			goto fail5;

		/* Serialise against efx_reset(). No more resets will be
		 * scheduled since efx_stop_all() has been called, and we
		 * have not and never have been registered with either
		 * the rtnetlink or driverlink layers. */
		rtnl_unlock();
		up(&efx->suspend_lock);

#if defined(EFX_USE_CANCEL_WORK_SYNC)
		cancel_work_sync(&efx->reset_work);
#else
		flush_workqueue(efx->reset_workqueue);
#endif

		down(&efx->suspend_lock);
		rtnl_lock();

		efx->reset_pending = RESET_TYPE_NONE;
	};
	if (rc) {
		EFX_ERR(efx, "Could not reset NIC\n");
		goto fail5;
	}

	/* Self-tests have all passed */
	rc = efx_init_debugfs_channels(efx);
	if (rc)
		goto fail6;

	/* Switch to the running state before we expose the device to
	 * the OS.  This is to ensure that the initial gathering of
	 * MAC stats succeeds. */
	efx->state = STATE_RUNNING;

	rtnl_unlock();

	rc = efx_register_netdev(efx);
	if (rc)
		goto fail7;

	up(&efx->suspend_lock);

	EFX_LOG(efx, "initialisation successful\n");

	/* Register with driverlink layer */
	rc = efx_dl_register_nic(efx);
	if (rc)
		goto fail8;

	return 0;

 fail8:
	down(&efx->suspend_lock);
	efx_unregister_netdev(efx);
 fail7:
	/* Re-acquire the rtnl lock around pci_remove_main() */
	rtnl_lock();
	efx_fini_debugfs_channels(efx);
 fail6:
	efx_pci_remove_main(efx);
 fail5:
	/* Drop the locks before fini */
	rtnl_unlock();
	up(&efx->suspend_lock);
 fail4:
	efx_fini_io(efx);
 fail3:
	efx_fini_struct(efx);
 fail2:
	kfree(efx);
 fail1:
	EFX_LOG(efx, "initialisation failed. rc=%d\n", rc);
	return rc;
}

/* PCI driver definition */
static struct pci_driver efx_pci_driver = {
	.name		= EFX_DRIVER_NAME,
	.id_table	= efx_pci_table,
	.probe		= efx_pci_probe,
	.remove		= efx_pci_remove,
};

/**************************************************************************
 *
 * Kernel module interface
 *
 *************************************************************************/

module_param(interrupt_mode, uint, 0444);
MODULE_PARM_DESC(interrupt_mode,
		 "Interrupt mode (0=>MSIX 1=>MSI 2=>legacy)");

module_param(onload_offline_selftest, uint, 0444);
MODULE_PARM_DESC(onload_offline_selftest, "Perform offline selftest on load");

static int __init efx_init_module(void)
{
	int rc;

	printk(KERN_INFO "Solarflare NET driver v" EFX_DRIVER_VERSION "\n");

	rc = efx_init_debugfs();
	if (rc)
		goto err_debugfs;

	rc = register_netdevice_notifier(&efx_netdev_notifier);
	if (rc)
		goto err_notifier;

	rc = pci_register_driver(&efx_pci_driver);
	if (rc < 0)
		goto err_pci;

	return 0;

 err_pci:
	unregister_netdevice_notifier(&efx_netdev_notifier);
 err_notifier:
	efx_fini_debugfs();
 err_debugfs:
	return rc;
}

static void __exit efx_exit_module(void)
{
	printk(KERN_INFO "Solarflare NET driver unloading\n");

	pci_unregister_driver(&efx_pci_driver);
	unregister_netdevice_notifier(&efx_netdev_notifier);
	efx_fini_debugfs();

}

module_init(efx_init_module);
module_exit(efx_exit_module);

MODULE_AUTHOR("Michael Brown <mbrown@fensystems.co.uk> and "
	      "Solarflare Communications");
MODULE_DESCRIPTION("Solarflare Communications network driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, efx_pci_table);
