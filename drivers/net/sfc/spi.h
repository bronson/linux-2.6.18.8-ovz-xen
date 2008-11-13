/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2005:      Fen Systems Ltd.
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

#ifndef EFX_SPI_H
#define EFX_SPI_H

#include "net_driver.h"

/**************************************************************************
 *
 * Basic SPI command set and bit definitions
 *
 *************************************************************************/

/*
 * Commands common to all known devices.
 *
 */

/* Write status register */
#define SPI_WRSR 0x01

/* Write data to memory array */
#define SPI_WRITE 0x02

/* Read data from memory array */
#define SPI_READ 0x03

/* Reset write enable latch */
#define SPI_WRDI 0x04

/* Read status register */
#define SPI_RDSR 0x05

/* Set write enable latch */
#define SPI_WREN 0x06

/* SST: Enable write to status register */
#define SPI_SST_EWSR 0x50

/*
 * Status register bits.  Not all bits are supported on all devices.
 *
 */

/* Write-protect pin enabled */
#define SPI_STATUS_WPEN 0x80

/* Block protection bit 2 */
#define SPI_STATUS_BP2 0x10

/* Block protection bit 1 */
#define SPI_STATUS_BP1 0x08

/* Block protection bit 0 */
#define SPI_STATUS_BP0 0x04

/* State of the write enable latch */
#define SPI_STATUS_WEN 0x02

/* Device busy flag */
#define SPI_STATUS_NRDY 0x01

/**************************************************************************
 *
 * Efx SPI devices
 *
 **************************************************************************
 */

/**
 * struct efx_spi_device - an Efx SPI (Serial Peripheral Interface) device
 * @device_id:		Controller's id for the device
 * @size:		Size (in bytes)
 * @addr_len:		Number of address bytes in read/write commands
 * @munge_address:	Flag whether addresses should be munged.
 *	Some devices with 9-bit addresses (e.g. AT25040A EEPROM)
 *	use bit 3 of the command byte as address bit A8, rather
 *	than having a two-byte address.  If this flag is set, then
 *	commands should be munged in this way.
 * @erase_command:	Erase command (or 0 if sector erase not needed).
 * @erase_size:		Erase sector size (in bytes)
 *	Erase commands affect sectors with this size and alignment.
 *	This must be a power of two.
 * @block_size:		Write block size (in bytes).
 *	Write commands are limited to blocks with this size and alignment.
 * @read:		Read function for the device
 * @write:		Write function for the device
 */
struct efx_spi_device {
	int device_id;
	unsigned int size;
	unsigned int addr_len;
	unsigned int munge_address:1;
	u8 erase_command;
	unsigned int erase_size;
	unsigned int block_size;
	int (*read) (const struct efx_spi_device *spi,
		     struct efx_nic *efx, unsigned int command,
		     int address, void *data, unsigned int len);
	int (*write) (const struct efx_spi_device *spi,
		      struct efx_nic *efx, unsigned int command,
		      int address, const void *data, unsigned int len);
};

/* Maximum length for SPI read or write through Falcon */
#define FALCON_SPI_MAX_LEN 16U

/**
 * efx_spi_write_limit - calculate maximum permitted length for write
 * @spi:		SPI device description
 * @start:		Starting address
 *
 * Return the maximum length for a write starting at the given address
 * in the device.
 *
 * SPI writes must not cross block boundaries.  Devices tend
 * to wrap addresses at block boundaries; e.g. trying to write 5 bytes
 * starting at offset 14 with a block size of 16 might write
 * {14,15,0,1,2} rather than {14,15,16,17,18}.
 */
static inline unsigned int
efx_spi_write_limit(const struct efx_spi_device *spi, unsigned int start)
{
	return min(FALCON_SPI_MAX_LEN,
		   (spi->block_size - (start & (spi->block_size - 1))));
}

/**
 * efx_spi_read_limit - calculate maximum permitted length for read
 * @spi:		SPI device description
 * @start:		Starting address
 *
 * Return the maximum length for a read starting at the given address
 * in the device.
 */
static inline unsigned int
efx_spi_read_limit(const struct efx_spi_device *spi __attribute__ ((unused)),
		   unsigned int start __attribute__ ((unused)))
{
	return FALCON_SPI_MAX_LEN;
}

/**
 * efx_spi_munge_command - adjust command as necessary for given address
 * @spi:		SPI device description
 * @command:		Normal SPI command
 * @address:		Address for command
 *
 * Some devices with 9-bit addresses (e.g. AT25040A EEPROM) use bit 3
 * of the command byte as address bit A8, rather than having a
 * two-byte address.  This function calculates the appropriate command
 * byte for the device, taking this munging into account.
 */
static inline u8 efx_spi_munge_command(const struct efx_spi_device *spi,
					    const u8 command,
					    const unsigned int address)
{
	return (command | (((address >> 8) & spi->munge_address) << 3));
}

#endif /* EFX_SPI_H */
