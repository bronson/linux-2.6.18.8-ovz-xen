/****************************************************************************
 * Driver for Solarflare network controllers
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

#ifndef EFX_WORKAROUNDS_H
#define EFX_WORKAROUNDS_H

/*
 * Hardware workarounds.
 * Bug numbers are from Solarflare's Bugzilla.
 */

#define EFX_WORKAROUND_ALWAYS(efx) 1
#define EFX_WORKAROUND_FALCON_A(efx) (FALCON_REV(efx) <= FALCON_REV_A1)
#define EFX_WORKAROUND_FALCON_B0FPGA(efx) \
	(FALCON_REV(efx) == FALCON_REV_B0 && !(efx)->is_asic)

/* XAUI resets if link not detected */
#define EFX_WORKAROUND_5147 EFX_WORKAROUND_ALWAYS
/* SNAP frames have TOBE_DISC set */
#define EFX_WORKAROUND_5475 EFX_WORKAROUND_ALWAYS
/* PHY interrupts can go to the wrong port */
#define EFX_WORKAROUND_6263 EFX_WORKAROUND_ALWAYS
/* Reprog PCIe ACK timer to workaround issue in PCIe IP block */
#define EFX_WORKAROUND_6943 EFX_WORKAROUND_ALWAYS
/* RX PCIe double split performance issue */
#define EFX_WORKAROUND_7575 EFX_WORKAROUND_ALWAYS
/* Bit-bashed I2C reads cause performance drop */
#define EFX_WORKAROUND_7884 EFX_WORKAROUND_ALWAYS
/* Selftests need to be retried */
#define EFX_WORKAROUND_8909 EFX_WORKAROUND_ALWAYS
/* Queued ACKs aren't flushed before L1 entry */
#define EFX_WORKAROUND_9096 EFX_WORKAROUND_ALWAYS
/* TX pkt parser problem with <= 16 byte TXes */
#define EFX_WORKAROUND_9141 EFX_WORKAROUND_ALWAYS
/* XGXS and XAUI reset sequencing in SW */
#define EFX_WORKAROUND_9388 EFX_WORKAROUND_ALWAYS
/* Low rate CRC errors require XAUI reset */
#define EFX_WORKAROUND_10750 EFX_WORKAROUND_ALWAYS
/* TX_EV_PKT_ERR can be caused by a dangling TX descriptor
 * or a PCIe error (bug 11028) */
#define EFX_WORKAROUND_10727 EFX_WORKAROUND_ALWAYS
/* CX4 retimer fails to bring link up after reset */
#define EFX_WORKAROUND_10934 EFX_WORKAROUND_ALWAYS
/* Transmit flow control may get disabled */
#define EFX_WORKAROUND_11482 EFX_WORKAROUND_ALWAYS
/* Flush events can take a very long time to appear */
#define EFX_WORKAROUND_11557 EFX_WORKAROUND_ALWAYS

/* Spurious parity errors in TSORT buffers */
#define EFX_WORKAROUND_5129 EFX_WORKAROUND_FALCON_A
/* No unaligned TX over 512 byte boundaries */
#define EFX_WORKAROUND_5391 EFX_WORKAROUND_FALCON_A
/* iSCSI parsing errors */
#define EFX_WORKAROUND_5583 EFX_WORKAROUND_FALCON_A
/* RX events go missing */
#define EFX_WORKAROUND_5676 EFX_WORKAROUND_FALCON_A
/* RX_RESET on A1 */
#define EFX_WORKAROUND_6555 EFX_WORKAROUND_FALCON_A
/* Spurious duplicate RX events */
#define EFX_WORKAROUND_7062 EFX_WORKAROUND_FALCON_A
/* Increase filter depth to avoid RX_RESET */
#define EFX_WORKAROUND_7244 EFX_WORKAROUND_FALCON_A
/* Flushes may never complete */
#define EFX_WORKAROUND_7803 EFX_WORKAROUND_FALCON_A
/* Leak overlength packets rather than free */
#define EFX_WORKAROUND_8071 EFX_WORKAROUND_FALCON_A

/* Memory needs clearing at start-of-day */
#define EFX_WORKAROUND_8202 EFX_WORKAROUND_FALCON_B0FPGA
/* MAC statistics are transient */
#define EFX_WORKAROUND_8419 EFX_WORKAROUND_FALCON_B0FPGA
/* Prefetch watchdog timer may trigger erroneously on busy systems */
#define EFX_WORKAROUND_9008 EFX_WORKAROUND_FALCON_B0FPGA

#endif /* EFX_WORKAROUNDS_H */
