/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005-2006: Fen Systems Ltd.
 * Copyright 2006:      Solarflare Communications Inc,
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

#define EFX_IN_KCOMPAT_C 1

#include "net_driver.h"
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/random.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/rtnetlink.h>
#include <linux/bootmem.h>
#include <asm/uaccess.h>
#include "gmii.h"
#include "ethtool.h"

/*
 * Kernel backwards compatibility
 *
 * This file provides functionality missing from earlier kernels.
 */

/**************************************************************************
 *
 * GMII-friendly versions of mii_ethtool_[gs]set
 *
 **************************************************************************
 *
 * Kernels prior to 2.6.12 don't support GMII PHYs via
 * mii_ethtool_gset and mii_ethtool_sset.  These are those functions
 * taken from a 2.6.12 kernel tree, with the tests for
 * mii->supports_gmii removed (since that field doesn't exist in older
 * kernels).
 *
 */

#ifdef EFX_NEED_MII_ETHTOOL_FIX
int efx_mii_ethtool_gset(struct mii_if_info *mii, struct ethtool_cmd *ecmd)
{
	struct net_device *dev = mii->dev;
	u32 advert, bmcr, lpa, nego;
	u32 advert2 = 0, bmcr2 = 0, lpa2 = 0;

	ecmd->supported =
	    (SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full |
	     SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full |
	     SUPPORTED_Autoneg | SUPPORTED_TP | SUPPORTED_MII);
	ecmd->supported |= SUPPORTED_1000baseT_Half | SUPPORTED_1000baseT_Full;

	/* only supports twisted-pair */
	ecmd->port = PORT_MII;

	/* only supports internal transceiver */
	ecmd->transceiver = XCVR_INTERNAL;

	/* this isn't fully supported at higher layers */
	ecmd->phy_address = mii->phy_id;

	ecmd->advertising = ADVERTISED_TP | ADVERTISED_MII;
	advert = mii->mdio_read(dev, mii->phy_id, MII_ADVERTISE);
	advert2 = mii->mdio_read(dev, mii->phy_id, MII_CTRL1000);

	if (advert & ADVERTISE_10HALF)
		ecmd->advertising |= ADVERTISED_10baseT_Half;
	if (advert & ADVERTISE_10FULL)
		ecmd->advertising |= ADVERTISED_10baseT_Full;
	if (advert & ADVERTISE_100HALF)
		ecmd->advertising |= ADVERTISED_100baseT_Half;
	if (advert & ADVERTISE_100FULL)
		ecmd->advertising |= ADVERTISED_100baseT_Full;
	if (advert2 & ADVERTISE_1000HALF)
		ecmd->advertising |= ADVERTISED_1000baseT_Half;
	if (advert2 & ADVERTISE_1000FULL)
		ecmd->advertising |= ADVERTISED_1000baseT_Full;

	bmcr = mii->mdio_read(dev, mii->phy_id, MII_BMCR);
	lpa = mii->mdio_read(dev, mii->phy_id, MII_LPA);
	bmcr2 = mii->mdio_read(dev, mii->phy_id, MII_CTRL1000);
	lpa2 = mii->mdio_read(dev, mii->phy_id, MII_STAT1000);
	if (bmcr & BMCR_ANENABLE) {
		ecmd->advertising |= ADVERTISED_Autoneg;
		ecmd->autoneg = AUTONEG_ENABLE;

		nego = mii_nway_result(advert & lpa);
		if ((bmcr2 & (ADVERTISE_1000HALF | ADVERTISE_1000FULL)) &
		    (lpa2 >> 2))
			ecmd->speed = SPEED_1000;
		else if (nego == LPA_100FULL || nego == LPA_100HALF)
			ecmd->speed = SPEED_100;
		else
			ecmd->speed = SPEED_10;
		if ((lpa2 & LPA_1000FULL) || nego == LPA_100FULL ||
		    nego == LPA_10FULL) {
			ecmd->duplex = DUPLEX_FULL;
			mii->full_duplex = 1;
		} else {
			ecmd->duplex = DUPLEX_HALF;
			mii->full_duplex = 0;
		}
	} else {
		ecmd->autoneg = AUTONEG_DISABLE;

		ecmd->speed = ((bmcr & BMCR_SPEED1000 &&
				(bmcr & BMCR_SPEED100) == 0) ? SPEED_1000 :
			       (bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10);
		ecmd->duplex =
			(bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}

	/* ignore maxtxpkt, maxrxpkt for now */

	return 0;
}

int efx_mii_ethtool_sset(struct mii_if_info *mii, struct ethtool_cmd *ecmd)
{
	struct net_device *dev = mii->dev;

	if (ecmd->speed != SPEED_10 &&
	    ecmd->speed != SPEED_100 &&
	    ecmd->speed != SPEED_1000)
		return -EINVAL;
	if (ecmd->duplex != DUPLEX_HALF && ecmd->duplex != DUPLEX_FULL)
		return -EINVAL;
	if (ecmd->port != PORT_MII)
		return -EINVAL;
	if (ecmd->transceiver != XCVR_INTERNAL)
		return -EINVAL;
	if (ecmd->phy_address != mii->phy_id)
		return -EINVAL;
	if (ecmd->autoneg != AUTONEG_DISABLE && ecmd->autoneg != AUTONEG_ENABLE)
		return -EINVAL;

	/* ignore supported, maxtxpkt, maxrxpkt */

	if (ecmd->autoneg == AUTONEG_ENABLE) {
		u32 bmcr, advert, tmp;
		u32 advert2 = 0, tmp2 = 0;

		if ((ecmd->advertising & (ADVERTISED_10baseT_Half |
					  ADVERTISED_10baseT_Full |
					  ADVERTISED_100baseT_Half |
					  ADVERTISED_100baseT_Full |
					  ADVERTISED_1000baseT_Half |
					  ADVERTISED_1000baseT_Full)) == 0)
			return -EINVAL;

		/* advertise only what has been requested */
		advert = mii->mdio_read(dev, mii->phy_id, MII_ADVERTISE);
		tmp = advert & ~(ADVERTISE_ALL | ADVERTISE_100BASE4);
		advert2 = mii->mdio_read(dev, mii->phy_id, MII_CTRL1000);
		tmp2 = advert2 & ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
		if (ecmd->advertising & ADVERTISED_10baseT_Half)
			tmp |= ADVERTISE_10HALF;
		if (ecmd->advertising & ADVERTISED_10baseT_Full)
			tmp |= ADVERTISE_10FULL;
		if (ecmd->advertising & ADVERTISED_100baseT_Half)
			tmp |= ADVERTISE_100HALF;
		if (ecmd->advertising & ADVERTISED_100baseT_Full)
			tmp |= ADVERTISE_100FULL;
		if (ecmd->advertising & ADVERTISED_1000baseT_Half)
			tmp2 |= ADVERTISE_1000HALF;
		if (ecmd->advertising & ADVERTISED_1000baseT_Full)
			tmp2 |= ADVERTISE_1000FULL;
		if (advert != tmp) {
			mii->mdio_write(dev, mii->phy_id, MII_ADVERTISE, tmp);
			mii->advertising = tmp;
		}
		if (advert2 != tmp2)
			mii->mdio_write(dev, mii->phy_id, MII_CTRL1000, tmp2);

		/* turn on autonegotiation, and force a renegotiate */
		bmcr = mii->mdio_read(dev, mii->phy_id, MII_BMCR);
		bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
		mii->mdio_write(dev, mii->phy_id, MII_BMCR, bmcr);

		mii->force_media = 0;
	} else {
		u32 bmcr, tmp;

		/* turn off auto negotiation, set speed and duplexity */
		bmcr = mii->mdio_read(dev, mii->phy_id, MII_BMCR);
		tmp = bmcr & ~(BMCR_ANENABLE | BMCR_SPEED100 |
			       BMCR_SPEED1000 | BMCR_FULLDPLX);
		if (ecmd->speed == SPEED_1000)
			tmp |= BMCR_SPEED1000;
		else if (ecmd->speed == SPEED_100)
			tmp |= BMCR_SPEED100;
		if (ecmd->duplex == DUPLEX_FULL) {
			tmp |= BMCR_FULLDPLX;
			mii->full_duplex = 1;
		} else {
			mii->full_duplex = 0;
		}
		if (bmcr != tmp)
			mii->mdio_write(dev, mii->phy_id, MII_BMCR, tmp);

		mii->force_media = 1;
	}
	return 0;
}
#endif /* NEED_EFX_MII_ETHTOOL_GSET */

/**************************************************************************
 *
 * unregister_netdevice_notifier : Has a race before 2.6.17
 *
 **************************************************************************
 *
 */

#ifdef EFX_NEED_UNREGISTER_NETDEVICE_NOTIFIER_FIX
/**
 * efx_unregister_netdevice_notifier - fixed unregister_netdevice_notifier
 * @nb:		notifier to unregister
 *
 * unregister_netdevice_notifier() does not wait for the notifier
 * to be unused before 2.6.17.  This wrapper fixes that.
 */
int efx_unregister_netdevice_notifier(struct notifier_block *nb)
{
	int res;

	res = unregister_netdevice_notifier(nb);
	/* Ensure any outstanding calls complete. */
	rtnl_lock();
	rtnl_unlock();
	return res;
}
#endif /* NEED_EFX_UNREGISTER_NETDEVICE_NOTIFIER */

/**************************************************************************
 *
 * IOMMU-locking versions of pci_[un]map_single and
 * pci_{alloc,free}_consistent.  See SFC bug 4560.
 *
 **************************************************************************
 *
 */
#ifdef EFX_NEED_IOMMU_LOCK

/*
 * efx_use_iommu_lock - IOMMU lock use control
 *
 * If set to 1, the driver will attempt to mitigate the race condition
 * bug around IOMMU accesses in some 2.6 kernels.  If set to 2, the
 * driver will use the lock even if it thinks it doesn't need to.
 * Note that this is only a best-effort attempt; in particular, we
 * cannot do anything about other drivers touching the IOMMU.
 */
static unsigned int efx_use_iommu_lock = 1;
EXPORT_SYMBOL(efx_use_iommu_lock);

/*
 * efx_iommu_lock - lock around IOMMU accesses
 *
 * This spinlock should be held while calling functions that access
 * the IOMMU if efx_use_iommu_lock is >= 2.  The efx_pci_*()
 * functions do this where possible.
 */
static spinlock_t efx_iommu_lock = SPIN_LOCK_UNLOCKED;
EXPORT_SYMBOL(efx_iommu_lock);

/* Don't use the IOMMU lock if the device can access the whole of memory */
#define EFX_DMA_CONSISTENT(_efx)			\
	(((_efx)->dma_mask >> PAGE_SHIFT) >= max_pfn)
/**
 * efx_pci_map_single - map buffer for DMA, under IOMMU lock
 * @pci:		PCI device
 * @ptr:		Buffer
 * @size:		Buffer length
 * @direction:		DMA direction
 *
 * Wrapper for pci_map_single that uses efx_iommu_lock if necessary.
 */
dma_addr_t efx_pci_map_single(struct pci_dev *pci, void *ptr, size_t size,
			      int direction)
{
	struct efx_nic *efx = pci_get_drvdata(pci);
	unsigned long flags __attribute__ ((unused));
	dma_addr_t dma_addr;

	if (unlikely((efx_use_iommu_lock &&
		      (!EFX_NO_IOMMU) &&
		      (!EFX_DMA_CONSISTENT(efx))) ||
		     efx_use_iommu_lock >= 2)) {
		spin_lock_irqsave(&efx_iommu_lock, flags);
		dma_addr = pci_map_single(pci, ptr, size, direction);
		spin_unlock_irqrestore(&efx_iommu_lock, flags);
	} else {
		dma_addr = pci_map_single(pci, ptr, size, direction);
	}
	return dma_addr;
}

/**
 * efx_pci_unmap_single - unmap buffer for DMA, under IOMMU lock
 * @pci:		PCI device
 * @dma_addr:		DMA address
 * @size:		Buffer length
 * @direction:		DMA direction
 *
 * Wrapper for pci_unmap_single that uses efx_iommu_lock if necessary.
 */
void efx_pci_unmap_single(struct pci_dev *pci, dma_addr_t dma_addr,
			  size_t size, int direction)
{
	struct efx_nic *efx = pci_get_drvdata(pci);
	unsigned long flags __attribute__ ((unused));

	if (unlikely((efx_use_iommu_lock &&
		      (!EFX_NO_IOMMU) &&
		      (!EFX_DMA_CONSISTENT(efx))) ||
		     efx_use_iommu_lock >= 2)) {
		spin_lock_irqsave(&efx_iommu_lock, flags);
		pci_unmap_single(pci, dma_addr, size, direction);
		spin_unlock_irqrestore(&efx_iommu_lock, flags);
	} else {
		pci_unmap_single(pci, dma_addr, size, direction);
	}
}

/**
 * efx_pci_alloc_consistent - allocate DMA-consistent buffer, under IOMMU lock
 * @pci:		PCI device
 * @size:		Buffer length
 * @dma_addr:		DMA address
 *
 * Wrapper for pci_alloc_consistent that uses efx_iommu_lock if necessary.
 *
 * Bugs: Currently this can't use the spinlock because
 *	pci_alloc_consistent may block.
 */
void *efx_pci_alloc_consistent(struct pci_dev *pci, size_t size,
			       dma_addr_t *dma_addr)
{
	return pci_alloc_consistent(pci, size, dma_addr);
}

/**
 * efx_pci_free_consistent - free DMA-consistent buffer, under IOMMU lock
 * @pci:		PCI device
 * @size:		Buffer length
 * @ptr:		Buffer
 * @dma_addr:		DMA address
 *
 * Wrapper for pci_free_consistent that uses efx_iommu_lock if necessary.
 */
void efx_pci_free_consistent(struct pci_dev *pci, size_t size, void *ptr,
			     dma_addr_t dma_addr)
{
	struct efx_nic *efx = pci_get_drvdata(pci);
	unsigned long flags __attribute__ ((unused));

	if (unlikely((efx_use_iommu_lock &&
		      (!EFX_NO_IOMMU) &&
		      (!EFX_DMA_CONSISTENT(efx))) ||
		     efx_use_iommu_lock >= 2)) {
		spin_lock_irqsave(&efx_iommu_lock, flags);
		pci_free_consistent(pci, size, ptr, dma_addr);
		spin_unlock_irqrestore(&efx_iommu_lock, flags);
	} else {
		pci_free_consistent(pci, size, ptr, dma_addr);
	}
}

module_param(efx_use_iommu_lock, uint, 0644);
MODULE_PARM_DESC(efx_use_iommu_lock, "Enable lock for bug in free_iommu");

#endif

#ifdef EFX_NEED_COMPOUND_PAGE_FIX

void efx_compound_page_destructor(struct page *page)
{
	/* Fake up page state to keep __free_pages happy */
	set_page_count(page, 1);
	page[1].mapping = NULL;

	__free_pages(page, (unsigned long)page[1].index);
}

#endif /* NEED_COMPOUND_PAGE_FIX */

/**************************************************************************
 *
 * print_hex_dump, taken from lib/hexdump.c.
 *
 **************************************************************************
 *
 */
#ifdef EFX_NEED_HEX_DUMP

#define hex_asc(x)	"0123456789abcdef"[x]
#define isascii(c) (((unsigned char)(c))<=0x7f)

static void hex_dump_to_buffer(const void *buf, size_t len, int rowsize,
			       int groupsize, char *linebuf, size_t linebuflen,
			       int ascii)
{
        const u8 *ptr = buf;
        u8 ch;
        int j, lx = 0;
        int ascii_column;

        if (rowsize != 16 && rowsize != 32)
                rowsize = 16;

        if (!len)
                goto nil;
        if (len > rowsize)              /* limit to one line at a time */
                len = rowsize;
        if ((len % groupsize) != 0)     /* no mixed size output */
                groupsize = 1;

        switch (groupsize) {
        case 8: {
                const u64 *ptr8 = buf;
                int ngroups = len / groupsize;

                for (j = 0; j < ngroups; j++)
                        lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%16.16llx ", (unsigned long long)*(ptr8 + j));
                ascii_column = 17 * ngroups + 2;
                break;
        }

        case 4: {
                const u32 *ptr4 = buf;
                int ngroups = len / groupsize;

                for (j = 0; j < ngroups; j++)
                        lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%8.8x ", *(ptr4 + j));
                ascii_column = 9 * ngroups + 2;
                break;
        }

        case 2: {
                const u16 *ptr2 = buf;
                int ngroups = len / groupsize;

                for (j = 0; j < ngroups; j++)
                        lx += scnprintf(linebuf + lx, linebuflen - lx,
				"%4.4x ", *(ptr2 + j));
                ascii_column = 5 * ngroups + 2;
                break;
        }

        default:
                for (j = 0; (j < rowsize) && (j < len) && (lx + 4) < linebuflen;
                     j++) {
                        ch = ptr[j];
                        linebuf[lx++] = hex_asc(ch >> 4);
                        linebuf[lx++] = hex_asc(ch & 0x0f);
                        linebuf[lx++] = ' ';
                }
                ascii_column = 3 * rowsize + 2;
                break;
        }
        if (!ascii)
                goto nil;

        while (lx < (linebuflen - 1) && lx < (ascii_column - 1))
                linebuf[lx++] = ' ';
	/* Removed is_print() check */
        for (j = 0; (j < rowsize) && (j < len) && (lx + 2) < linebuflen; j++)
                linebuf[lx++] = isascii(ptr[j]) ? ptr[j] : '.';
nil:
        linebuf[lx++] = '\0';
}

void print_hex_dump(const char *level, const char *prefix_str, int prefix_type,
		    int rowsize, int groupsize,
		    const void *buf, size_t len, int ascii)
{
        const u8 *ptr = buf;
        int i, linelen, remaining = len;
        char linebuf[200];

        if (rowsize != 16 && rowsize != 32)
                rowsize = 16;

        for (i = 0; i < len; i += rowsize) {
                linelen = min(remaining, rowsize);
                remaining -= rowsize;
                hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
				   linebuf, sizeof(linebuf), ascii);

                switch (prefix_type) {
                case DUMP_PREFIX_ADDRESS:
                        printk("%s%s%*p: %s\n", level, prefix_str,
			       (int)(2 * sizeof(void *)), ptr + i, linebuf);
                        break;
                case DUMP_PREFIX_OFFSET:
                        printk("%s%s%.8x: %s\n", level, prefix_str, i, linebuf);
                        break;
                default:
                        printk("%s%s%s\n", level, prefix_str, linebuf);
                        break;
                }
        }
}

#endif /* EFX_NEED_HEX_DUMP */

/**************************************************************************
 *
 * print_mac, from net/ethernet/eth.c in v2.6.24
 *
 **************************************************************************
 *
 */
#ifdef EFX_NEED_PRINT_MAC
char *print_mac(char *buf, const u8 *addr)
{
        sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        return buf;
}
#endif /* EFX_NEED_PRINT_MAC */

#ifdef EFX_NEED_CSUM_TCPUDP_NOFOLD
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
__wsum
csum_tcpudp_nofold (__be32 saddr, __be32 daddr, unsigned short len,
		    unsigned short proto, __wsum sum)
#else
__wsum
csum_tcpudp_nofold (unsigned long saddr, unsigned long daddr,
		    unsigned short len, unsigned short proto, __wsum sum)
#endif
{
	unsigned long result;

	result = (__force u64)saddr + (__force u64)daddr +
		(__force u64)sum + ((len + proto) << 8);

	/* Fold down to 32-bits so we don't lose in the typedef-less network stack.  */
	/* 64 to 33 */
	result = (result & 0xffffffff) + (result >> 32);
	/* 33 to 32 */
	result = (result & 0xffffffff) + (result >> 32);
	return (__force __wsum)result;

}
#endif /* EFX_NEED_CSUM_TCPUDP_NOFOLD */

#ifdef EFX_NEED_RANDOM_ETHER_ADDR
/* Generate random MAC address */
void efx_random_ether_addr(uint8_t *addr) {
        get_random_bytes (addr, ETH_ALEN);
	addr [0] &= 0xfe;       /* clear multicast bit */
	addr [0] |= 0x02;       /* set local assignment bit (IEEE802) */
}
#endif /* EFX_NEED_RANDOM_ETHER_ADDR */

#ifdef EFX_NEED_MSECS_TO_JIFFIES
/*
 * When we convert to jiffies then we interpret incoming values
 * the following way:
 *
 * - negative values mean 'infinite timeout' (MAX_JIFFY_OFFSET)
 *
 * - 'too large' values [that would result in larger than
 *   MAX_JIFFY_OFFSET values] mean 'infinite timeout' too.
 *
 * - all other values are converted to jiffies by either multiplying
 *   the input value by a factor or dividing it with a factor
 *
 * We must also be careful about 32-bit overflows.
 */
#ifndef MSEC_PER_SEC
#define MSEC_PER_SEC	1000L
#endif
unsigned long msecs_to_jiffies(const unsigned int m)
{
	/*
	 * Negative value, means infinite timeout:
	 */
	if ((int)m < 0)
		return MAX_JIFFY_OFFSET;

#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
	/*
	 * HZ is equal to or smaller than 1000, and 1000 is a nice
	 * round multiple of HZ, divide with the factor between them,
	 * but round upwards:
	 */
	return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
	/*
	 * HZ is larger than 1000, and HZ is a nice round multiple of
	 * 1000 - simply multiply with the factor between them.
	 *
	 * But first make sure the multiplication result cannot
	 * overflow:
	 */
	if (m > jiffies_to_msecs(MAX_JIFFY_OFFSET))
		return MAX_JIFFY_OFFSET;

	return m * (HZ / MSEC_PER_SEC);
#else
	/*
	 * Generic case - multiply, round and divide. But first
	 * check that if we are doing a net multiplication, that
	 * we wouldnt overflow:
	 */
	if (HZ > MSEC_PER_SEC && m > jiffies_to_msecs(MAX_JIFFY_OFFSET))
		return MAX_JIFFY_OFFSET;

	return (m * HZ + MSEC_PER_SEC - 1) / MSEC_PER_SEC;
#endif
}
#endif /* EFX_NEED_MSECS_TO_JIFFIES */

#ifdef EFX_NEED_MSLEEP
/**
 * msleep - sleep safely even with waitqueue interruptions
 * @msecs: Time in milliseconds to sleep for
 */
void msleep(unsigned int msecs)
{
	unsigned long timeout = msecs_to_jiffies(msecs) + 1;

	while (timeout)
		timeout = schedule_timeout_uninterruptible(timeout);
}
#endif
