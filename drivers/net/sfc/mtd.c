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

#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>

#define EFX_DRIVER_NAME "sfc_mtd"
#include "driverlink_api.h"
#include "net_driver.h"
#include "spi.h"

/*
 * Flash and EEPROM (MTD) device driver
 *
 * This file provides a separate kernel module (sfc_mtd) which
 * exposes the flash and EEPROM devices present on Solarflare NICs as
 * MTD devices, enabling you to reflash the boot ROM code (or use the
 * remaining space on the flash as a jffs2 filesystem, should you want
 * to do so).
 */

#define EFX_MTD_VERIFY_BUF_LEN 16
#define EFX_MAX_PARTITIONS 2
#define EFX_FLASH_BOOTROM_OFFSET 0x8000U

/* Write enable for EEPROM/flash configuration area
 *
 * Normally, writes to parts of non-volatile storage which contain
 * critical configuration are disabled to prevent accidents.  This
 * parameter allows enabling of such writes.
 */
static unsigned int efx_allow_nvconfig_writes;

struct efx_mtd {
	struct mtd_info mtd;
	struct mtd_partition part[EFX_MAX_PARTITIONS];
	char part_name[EFX_MAX_PARTITIONS][32];
	char name[32];
	struct efx_dl_device *efx_dev;
	struct efx_nic *efx;
	/* This must be held when using *spi; it guards against races
	 * with device reset and between sequences of dependent
	 * commands. */
	struct semaphore access_lock;
	struct efx_spi_device *spi;
};

/* SPI utilities */

static int efx_spi_fast_wait(struct efx_mtd *efx_mtd)
{
	struct efx_spi_device *spi = efx_mtd->spi;
	u8 status;
	int i, rc;

	/* Wait up to 1000us for flash/EEPROM to finish a fast operation. */
	for (i = 0; i < 50; i++) {
		udelay(20);

		rc = spi->read(spi, efx_mtd->efx, SPI_RDSR, -1,
			       &status, sizeof(status));
		if (rc)
			return rc;
		if (!(status & SPI_STATUS_NRDY))
			return 0;
	}
	EFX_ERR(efx_mtd->efx, "timed out waiting for %s last status=0x%02x\n",
		efx_mtd->name, status);
	return -ETIMEDOUT;
}

static int efx_spi_slow_wait(struct efx_mtd *efx_mtd, int uninterruptible)
{
	struct efx_spi_device *spi = efx_mtd->spi;
	u8 status;
	int rc, i;

	/* Wait up to 4s for flash/EEPROM to finish a slow operation. */
	for (i = 0; i < 40; i++) {
		__set_current_state(uninterruptible ?
				    TASK_UNINTERRUPTIBLE : TASK_INTERRUPTIBLE);
		schedule_timeout(HZ / 10);
		rc = spi->read(spi, efx_mtd->efx, SPI_RDSR, -1,
			       &status, sizeof(status));
		if (rc)
			return rc;
		if (!(status & SPI_STATUS_NRDY))
			return 0;
		if (signal_pending(current))
			return -EINTR;
	}
	EFX_ERR(efx_mtd->efx, "timed out waiting for %s\n", efx_mtd->name);
	return -ETIMEDOUT;
}

static int
efx_spi_write_enable(struct efx_mtd *efx_mtd)
{
	struct efx_spi_device *spi = efx_mtd->spi;

	return spi->write(spi, efx_mtd->efx, SPI_WREN, -1, NULL, 0);
}

static int efx_spi_unlock(struct efx_mtd *efx_mtd)
{
	struct efx_spi_device *spi = efx_mtd->spi;
	const u8 unlock_mask = (SPI_STATUS_BP2 | SPI_STATUS_BP1 |
				     SPI_STATUS_BP0);
	u8 status;
	int rc;

	rc = spi->read(spi, efx_mtd->efx, SPI_RDSR, -1, &status,
		       sizeof(status));
	if (rc)
		return rc;

	if (!(status & unlock_mask))
		return 0; /* already unlocked */

	rc = efx_spi_write_enable(efx_mtd);
	if (rc)
		return rc;
	rc = spi->write(spi, efx_mtd->efx, SPI_SST_EWSR, -1, NULL, 0);
	if (rc)
		return rc;

	status &= ~unlock_mask;
	rc = spi->write(spi, efx_mtd->efx, SPI_WRSR, -1, &status,
			sizeof(status));
	if (rc)
		return rc;
	rc = efx_spi_fast_wait(efx_mtd);
	if (rc)
		return rc;

	return 0;
}

/* Dummy device used in case of a failed reset */

static int efx_spi_dummy_read(const struct efx_spi_device *spi,
			      struct efx_nic *efx, unsigned int command,
			      int address, void *data, unsigned int len)
{
	return -EIO;
}

static int efx_spi_dummy_write(const struct efx_spi_device *spi,
			       struct efx_nic *efx, unsigned int command,
			       int address, const void *data, unsigned int len)
{
	return -EIO;
}

static struct efx_spi_device efx_spi_dummy_device = {
	.block_size	= 1,
	.erase_command	= 0xff,
	.read		= efx_spi_dummy_read,
	.write		= efx_spi_dummy_write,
};

/* MTD interface */

static int efx_mtd_read(struct mtd_info *mtd, loff_t start, size_t len,
			size_t *retlen, u8 *buffer)
{
	struct efx_mtd *efx_mtd = mtd->priv;
	struct efx_spi_device *spi;
	unsigned int command;
	unsigned int block_len;
	unsigned int pos = 0;
	int rc;

	rc = down_interruptible(&efx_mtd->access_lock);
	if (rc)
		goto out;
	spi = efx_mtd->spi;

	while (pos < len) {
		block_len = min((unsigned int)len - pos,
				efx_spi_read_limit(spi, start + pos));
		command = efx_spi_munge_command(spi, SPI_READ, start + pos);
		rc = spi->read(spi, efx_mtd->efx, command, start + pos,
			       buffer + pos, block_len);
		if (rc)
			break;
		pos += block_len;

		/* Avoid locking up the system */
		cond_resched();
		if (signal_pending(current)) {
			rc = -EINTR;
			break;
		}
	}

	up(&efx_mtd->access_lock);
out:
	*retlen = pos;
	return rc;
}

/* Check that device contents match buffer.  If repeat is true, buffer
 * contains a pattern of length EFX_MTD_VERIFY_BUF_LEN which the
 * device contents should match repeatedly.
 */
static int efx_mtd_verify(struct mtd_info *mtd, loff_t start,
			  size_t len, const u8 *buffer, int repeat)
{
	u8 verify_buffer[EFX_MTD_VERIFY_BUF_LEN];
	unsigned int block_len;
	size_t read_len;
	unsigned int pos = 0;
	int rc = 0;

	while (pos < len) {
		block_len = min(len - pos, sizeof(verify_buffer));
		rc = efx_mtd_read(mtd, start + pos, block_len, &read_len,
				  verify_buffer);
		if (rc)
			return rc;
		if (memcmp(repeat ? buffer : buffer + pos, verify_buffer,
			   block_len))
			return -EIO;
		pos += block_len;
	}

	return 0;
}

static int efx_mtd_erase(struct mtd_info *mtd, struct erase_info *erase)
{
	struct efx_mtd *efx_mtd = mtd->priv;
	struct efx_spi_device *spi;
	u8 empty[EFX_MTD_VERIFY_BUF_LEN];
	int rc;

	if (erase->len != mtd->erasesize) {
		rc = -EINVAL;
		goto out;
	}

	rc = down_interruptible(&efx_mtd->access_lock);
	if (rc)
		goto out;
	spi = efx_mtd->spi;
	if (spi->erase_command == 0) {
		rc = -EOPNOTSUPP;
		goto out_up;
	}

	rc = efx_spi_unlock(efx_mtd);
	if (rc)
		goto out_up;
	rc = efx_spi_write_enable(efx_mtd);
	if (rc)
		goto out_up;
	rc = spi->write(spi, efx_mtd->efx, spi->erase_command, erase->addr,
			NULL, 0);
	if (rc)
		goto out_up;
	rc = efx_spi_slow_wait(efx_mtd, 0);

out_up:
	up(&efx_mtd->access_lock);
	if (rc)
		goto out;

	memset(empty, 0xff, sizeof(empty));
	rc = efx_mtd_verify(mtd, erase->addr, erase->len, empty, 1);

out:
	if (rc == 0) {
		erase->state = MTD_ERASE_DONE;
	} else {
		erase->state = MTD_ERASE_FAILED;
#if defined(EFX_USE_MTD_ERASE_FAIL_ADDR)
		erase->fail_addr = 0xffffffff;
#endif
	}
	mtd_erase_callback(erase);
	return rc;
}

static int efx_mtd_write(struct mtd_info *mtd, loff_t start,
			 size_t len, size_t *retlen, const u8 *buffer)
{
	struct efx_mtd *efx_mtd = mtd->priv;
	struct efx_spi_device *spi;
	unsigned int command;
	unsigned int block_len;
	unsigned int pos = 0;
	int rc;

	rc = down_interruptible(&efx_mtd->access_lock);
	if (rc)
		goto out;
	spi = efx_mtd->spi;

	rc = efx_spi_unlock(efx_mtd);
	if (rc)
		goto out_up;

	while (pos < len) {
		rc = efx_spi_write_enable(efx_mtd);
		if (rc)
			break;

		block_len = min((unsigned int)len - pos,
				efx_spi_write_limit(spi, start + pos));
		command = efx_spi_munge_command(spi, SPI_WRITE, start + pos);
		rc = spi->write(spi, efx_mtd->efx, command, start + pos,
				buffer + pos, block_len);
		if (rc)
			break;
		pos += block_len;

		rc = efx_spi_fast_wait(efx_mtd);
		if (rc)
			break;

		/* Avoid locking up the system */
		cond_resched();
		if (signal_pending(current)) {
			rc = -EINTR;
			break;
		}
	}

out_up:
	up(&efx_mtd->access_lock);
	if (rc == 0)
		rc = efx_mtd_verify(mtd, start, len, buffer, 0);
out:
	*retlen = pos;
	return rc;
}

static void efx_mtd_sync(struct mtd_info *mtd)
{
	struct efx_mtd *efx_mtd = mtd->priv;
	int rc;

	down(&efx_mtd->access_lock);
	rc = efx_spi_slow_wait(efx_mtd, 1);
	if (rc)
		EFX_ERR(efx_mtd->efx, "%s sync failed (%d)\n",
			efx_mtd->name, rc);
	up(&efx_mtd->access_lock);
}

/* Driverlink interface */

static void efx_mtd_reset_suspend(struct efx_dl_device *efx_dev)
{
	struct efx_mtd *efx_mtd = efx_dev->priv;

	if (!efx_mtd)
		return;

	/* Acquire lock to ensure that any in-progress operations have
	 * completed, and no new ones can start.
	 */
	down(&efx_mtd->access_lock);
}

static void efx_mtd_reset_resume(struct efx_dl_device *efx_dev, int ok)
{
	struct efx_mtd *efx_mtd = efx_dev->priv;

	if (!efx_mtd)
		return;

	/* If device reset failed already, or SPI device doesn't
	 * become ready, disable device.
	 */
	if (!ok || efx_spi_slow_wait(efx_mtd, 1) != 0) {
		efx_mtd->spi = &efx_spi_dummy_device;
		EFX_ERR(efx_mtd->efx, "%s disabled after failed reset\n",
			efx_mtd->name);
	}

	up(&efx_mtd->access_lock);
}

static void efx_mtd_remove(struct efx_dl_device *efx_dev)
{
	struct efx_mtd *efx_mtd = efx_dev->priv;

	del_mtd_partitions(&efx_mtd->mtd);
	kfree(efx_mtd);
	efx_dev->priv = NULL;
}

static __devinit int efx_mtd_register(struct efx_mtd *efx_mtd,
				      struct efx_dl_device *efx_dev,
				      struct efx_nic *efx,
				      struct efx_spi_device *spi,
				      const char *type_name,
				      const char **part_type_name,
				      unsigned int num_parts)
{
	int i;

	efx_dev->priv = efx_mtd;

	efx_mtd->efx_dev = efx_dev;
	efx_mtd->efx = efx;
	efx_mtd->spi = spi;
	sema_init(&efx_mtd->access_lock, 1);

	efx_mtd->mtd.size = spi->size;
	efx_mtd->mtd.erasesize = spi->erase_size;
#if defined(EFX_USE_MTD_WRITESIZE)
	efx_mtd->mtd.writesize = 1;
#endif
	if (snprintf(efx_mtd->name, sizeof(efx_mtd->name),
		     "%s %s", efx->name, type_name) >=
	    sizeof(efx_mtd->name))
		return -ENAMETOOLONG;

	efx_mtd->mtd.priv = efx_mtd;
	efx_mtd->mtd.name = efx_mtd->name;
	efx_mtd->mtd.erase = efx_mtd_erase;
	efx_mtd->mtd.read = efx_mtd_read;
	efx_mtd->mtd.write = efx_mtd_write;
	efx_mtd->mtd.sync = efx_mtd_sync;

	for (i = 0; i < num_parts; i++) {
		efx_mtd->part[i].name = efx_mtd->part_name[i];
		if (snprintf(efx_mtd->part_name[i],
			     sizeof(efx_mtd->part_name[i]),
			     "%s %s", efx->name, part_type_name[i]) >=
		    sizeof(efx_mtd->part_name[i]))
			return -ENAMETOOLONG;

		if (efx_allow_nvconfig_writes)
			efx_mtd->part[i].mask_flags &= ~MTD_WRITEABLE;
	}

	return add_mtd_partitions(&efx_mtd->mtd, efx_mtd->part, num_parts);
}

static int __devinit
efx_flash_probe(struct efx_dl_device *efx_dev,
		const struct net_device *net_dev,
		const struct efx_dl_device_info *dev_info,
		const char *silicon_rev)
{
	struct efx_nic *efx = efx_dl_get_nic(efx_dev);
	struct efx_mtd *efx_mtd;
	const char *part_type_name[2];
	unsigned int num_parts;
	int rc;

	if (!efx->spi_flash)
		return -ENODEV;

	efx_mtd = kzalloc(sizeof(*efx_mtd), GFP_KERNEL);
	if (!efx_mtd)
		return -ENOMEM;

	efx_mtd->mtd.type = MTD_NORFLASH;
	efx_mtd->mtd.flags = MTD_CAP_NORFLASH;

	part_type_name[0] = "sfc_flash_config";
	efx_mtd->part[0].offset = 0;
	efx_mtd->part[0].size = min(efx->spi_flash->size,
				    EFX_FLASH_BOOTROM_OFFSET);
	efx_mtd->part[0].mask_flags = MTD_WRITEABLE;

	if (efx->spi_flash->size <= EFX_FLASH_BOOTROM_OFFSET) {
		num_parts = 1;
	} else {
		part_type_name[1] = "sfc_flash_bootrom";
		efx_mtd->part[1].offset = EFX_FLASH_BOOTROM_OFFSET;
		efx_mtd->part[1].size = (efx->spi_flash->size
					 - EFX_FLASH_BOOTROM_OFFSET);
		num_parts = 2;
	}

	rc = efx_mtd_register(efx_mtd, efx_dev, efx, efx->spi_flash,
			      "sfc_flash", part_type_name, num_parts);
	if (rc)
		kfree(efx_mtd);
	return rc;
}

static struct efx_dl_driver efx_flash_driver = {
	.name		= "sfc_flash",
	.probe		= efx_flash_probe,
	.remove		= efx_mtd_remove,
	.reset_suspend	= efx_mtd_reset_suspend,
	.reset_resume	= efx_mtd_reset_resume,
};

static int __devinit
efx_eeprom_probe(struct efx_dl_device *efx_dev,
		 const struct net_device *net_dev,
		 const struct efx_dl_device_info *dev_info,
		 const char *silicon_rev)
{
	struct efx_nic *efx = efx_dl_get_nic(efx_dev);
	struct efx_mtd *efx_mtd;
	const char *type_name;
	const char *part_type_name[1];
	int rc;

	if (!efx->spi_eeprom)
		return -ENODEV;

	efx_mtd = kzalloc(sizeof(*efx_mtd), GFP_KERNEL);
	if (!efx_mtd)
		return -ENOMEM;

	efx_mtd->mtd.type = MTD_RAM;
	efx_mtd->mtd.flags = MTD_CAP_RAM;

	efx_mtd->part[0].offset = 0;
	efx_mtd->part[0].size = efx->spi_eeprom->size;
	efx_mtd->part[0].mask_flags = MTD_WRITEABLE;

	if (efx->spi_eeprom->size <= 0x200) {
		type_name = "sfc_small_eeprom";
		part_type_name[0] = "sfc_small_config";
	} else {
		type_name = "sfc_large_eeprom";
		part_type_name[0] = "sfc_large_config";
	}

	rc = efx_mtd_register(efx_mtd, efx_dev, efx, efx->spi_eeprom,
			      type_name, part_type_name, 1);
	if (rc)
		kfree(efx_mtd);
	return rc;
}

static struct efx_dl_driver efx_eeprom_driver = {
	.name		= "sfc_eeprom",
	.probe		= efx_eeprom_probe,
	.remove		= efx_mtd_remove,
	.reset_suspend	= efx_mtd_reset_suspend,
	.reset_resume	= efx_mtd_reset_resume,
};

/* Kernel module interface */

static int __init efx_mtd_init_module(void)
{
	int rc;

	rc = efx_dl_register_driver(&efx_flash_driver);
	if (rc)
		return rc;
	rc = efx_dl_register_driver(&efx_eeprom_driver);
	if (rc) {
		efx_dl_unregister_driver(&efx_flash_driver);
		return rc;
	}

	return 0;
}

static void __exit efx_mtd_exit_module(void)
{
	efx_dl_unregister_driver(&efx_eeprom_driver);
	efx_dl_unregister_driver(&efx_flash_driver);
}

module_init(efx_mtd_init_module);
module_exit(efx_mtd_exit_module);

MODULE_AUTHOR("Michael Brown <mbrown@fensystems.co.uk> and "
	      "Solarflare Communications");
MODULE_DESCRIPTION("SFC MTD driver");
MODULE_LICENSE("GPL");
