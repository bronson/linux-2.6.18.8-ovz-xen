/****************************************************************************
 * Driver for Solarflare network controllers
 *           (including support for SFE4001 10GBT NIC)
 *
 * Copyright 2007:      Solarflare Communications Inc,
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
 ****************************************************************************/

#include "net_driver.h"
#include "lm87_support.h"
#include "workarounds.h"

/* Setting this to 1 will cause efx_check_lm87 to dump the status when it
 * detects an alarm. This will result in the canonical name (i.e. that in
 * the LM87 data book) being printed for each set status bit, along with
 * the reading for that sensor value, if applicable. If set to 0 only the
 * raw status1 and status2 register values are printed. */
#define LM87_VERBOSE_ALARMS	1

/**************************************************************************
 *
 * Onboard LM87 temperature and voltage monitor
 *
 **************************************************************************
 */

/* LM87 channel mode: all current boards either do not use AIN1/FAN1 and 2
 * or use them as AIN. */
#define LM87_CHANNEL_MODE 0x16
#define LM87_CHANNEL_AIN1 1
#define LM87_CHANNEL_AIN2 2
#define LM87_CHANNEL_INIT (LM87_CHANNEL_AIN2 | LM87_CHANNEL_AIN1)

/* LM87 configuration register 1 */
#define LM87_CONFIG_1 0x40
#define LM87_START 0x01
#define LM87_INTEN 0x02
#define LM87_INITIALIZATION 0x80

/* LM87 interrupt status register 1 */
#define LM87_INT_STATUS_1 0x41

/* LM87 interrupt status register 2 */
#define LM87_INT_STATUS_2 0x42

/* LM87 interrupt mask register 1 */
#define LM87_INT_MASK_1 0x43

/* LM87 interrupt mask register 2 */
#define LM87_INT_MASK_2 0x44

/* LM87 monitoring limits */
#define LM87_LIMITS 0x2b


int efx_probe_lm87(struct efx_nic *efx, int addr,
		   const u8 *limits, int nlimits, const u16 irqmask)
{
	struct efx_i2c_interface *i2c = &efx->i2c;
	u8 byte;
	int rc;

	/* Check for onboard LM87 */
	rc = efx_i2c_check_presence_retry(i2c, addr);
	if (rc) {
		/* Not an error to lack an LM87, but failure to probe the
		 * bus is worrying. */
		if (rc == -EFAULT) {
			EFX_ERR(efx, "Failed to probe I2C bus for LM87!\n");
			return rc;
		} else {
			EFX_LOG(efx, "has no onboard LM87 chip\n");
			return 0;
		}
	}
	efx->board_info.lm87_addr = addr;
	EFX_LOG(efx, "detected onboard LM87 chip at 0x%2x\n", addr);

	/* Reset chip */
	byte = LM87_INITIALIZATION;
	rc = efx_i2c_write_retry(i2c, addr, LM87_CONFIG_1, &byte, 1);
	if (rc) {
		EFX_ERR(efx, "could not reset LM87\n");
		return rc;
	}

	/* Configure channel mode: currently hardwire to make pins 5 and 6
	 * AIN1 and AIN2 rather than FAN1, FAN2. */
	byte = LM87_CHANNEL_INIT;
	rc = efx_i2c_write_retry(i2c, addr, LM87_CHANNEL_MODE, &byte, 1);
	if (rc) {
		EFX_ERR(efx, "could not program LM87 chan. mode\n");
		return rc;
	}

	/* Configure limits */
	rc = efx_i2c_write_retry(i2c, addr, LM87_LIMITS, limits, nlimits);
	if (rc) {
		EFX_ERR(efx, "could not program LM87 limits\n");
		return rc;
	}

	/* Mask off unwanted interrupts */
	byte = (irqmask & 0xff);
	rc = efx_i2c_write_retry(i2c, addr, LM87_INT_MASK_1, &byte, 1);
	if (rc) {
		EFX_ERR(efx, "could not mask LM87 interrupts\n");
		return rc;
	}

	byte = (irqmask >> 8);
	rc = efx_i2c_write_retry(i2c, addr, LM87_INT_MASK_2, &byte, 1);
	if (rc) {
		EFX_ERR(efx, "could not mask LM87 interrupts\n");
		return rc;
	}

	/* Start monitoring */
	byte = LM87_START;
	if (irqmask != EFX_LM87_NO_INTS)
		byte |= LM87_INTEN;

	rc = efx_i2c_write_retry(i2c, addr, LM87_CONFIG_1, &byte, 1);
	if (rc) {
		EFX_ERR(efx, "could not start LM87\n");
		return rc;
	}

	return rc;
}

void efx_remove_lm87(struct efx_nic *efx)
{
	struct efx_i2c_interface *i2c = &efx->i2c;
	u8 byte;

	if (!efx->board_info.lm87_addr)
		return;

	/* Reset chip */
	byte = LM87_INITIALIZATION;
	if (efx_i2c_write_retry(i2c, efx->board_info.lm87_addr,
				LM87_CONFIG_1, &byte, 1) != 0)
		EFX_ERR(efx, "could not reset LM87 on exit\n");
}

#if LM87_VERBOSE_ALARMS
/* Bit number to name mapping for status1 */
static const char *lm_stat_names[] = {
/* Status 1 contents */
	"+2.5Vin",
	"Vccp1",
	"Vcc",
	"+5Vin",
	"Int. Temp.",
	"Ext. Temp.",
	"FAN1/AIN1",
	"FAN2/AIN2",
/* Status 2 contents */
	"+12Vin",
	"Vccp2",
	"Reserved",
	"Reserved",
	"CI",
	"THERM#",
	"D1 Fault",
	"D2 Fault"
};

/* Where to read the value corresponding to an alarm bit. */
static const int lm_stat_regs[] = {
	0x20, 0x21, 0x22, 0x23, 0x27, 0x26, 0x28, 0x29,
	0x24, 0x25, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/* The positions of the alarm bits do not correspond exactly to the
 * order of the limit values. Convert so the user only needs to maintain
 * one array */
static int lm_bit_to_lim[] = {
	0, /* 2.5V */
	1, /* Vccp1 */
	2, /* Vcc */
	3, /* 5V */
	7, /* Int temp. */
	6, /* Ext temp. */
	8, /* AIN1 */
	9, /* AIN2 */
	4, /* 12V */
	5  /* Vccp2 */
};

/* These are bit numbers. I feel justified in hardwiring the max. */
static const int lm_stat_max = 16;

static void lm87_show_alarm(struct efx_nic *efx, int bit)
{
	char valbuf[8];
	u8 val;

	if (lm_stat_regs[bit] != 0xff) {
		efx_i2c_read_retry(&efx->i2c, efx->board_info.lm87_addr,
				   lm_stat_regs[bit], &val, 1);
		sprintf(valbuf, "0x%02x  ", val);
	} else {
		strcpy(valbuf, "----  ");
	}
	/* If the board code knows what this sensor is wired to, let it tell
	 * us, else just print the LM87 datasheet name of the input, and the
	 * value. */
	if (efx->board_info.interpret_sensor == NULL ||
	    (bit < ARRAY_SIZE(lm_bit_to_lim) &&
	     efx->board_info.interpret_sensor(efx, lm_bit_to_lim[bit], val)
	     == 0))
		EFX_ERR(efx, ": %10s  %4s\n",
			STRING_TABLE_LOOKUP(bit, lm_stat), valbuf);
}

static void lm87_dump_alarms(struct efx_nic *efx, int stat1, int stat2)
{
	int i;
	EFX_ERR(efx, "   NAME    value\n");
	for (i = 0; i < 8; i++) {
		if (stat1 & (1 << i))
			lm87_show_alarm(efx, i);
		if (stat2 & (1 << i))
			lm87_show_alarm(efx, i + 8);
	}
}

#else
#define lm87_dump_alarms(_name, _stat1, _stat2) do {} while (0)
#endif

/* Read onboard LM87 (if present)
 * Return error code if lm87 could not be read (-EIO)
 * _or_ is raising an alarm (-ERANGE). 0 if AOK.
 */
int efx_check_lm87(struct efx_nic *efx, unsigned mask)
{
	struct efx_i2c_interface *i2c = &efx->i2c;
	u8 int_status_1, int_status_2;
	unsigned ints;
	int rc = 0;

	/* If link is up then do not monitor temperature */
	if (EFX_WORKAROUND_7884(efx) && efx->link_up)
		return 0;

	if (!efx->board_info.lm87_addr)
		return 0;

	/* Read interrupt status registers */
	rc = efx_i2c_read_retry(i2c, efx->board_info.lm87_addr,
				LM87_INT_STATUS_1, &int_status_1, 1);
	if (rc) {
		EFX_ERR(efx, "could not read LM87 INT status 1\n");
		return rc;
	}
	rc = efx_i2c_read_retry(i2c, efx->board_info.lm87_addr,
				LM87_INT_STATUS_2, &int_status_2, 1);
	if (rc) {
		EFX_ERR(efx, "could not read LM87 INT status 2\n");
		return rc;
	}

	int_status_1 &= mask;
	int_status_2 &= (mask >> 8);
	ints = ((int_status_2 << 8) | int_status_1);

	/* Check interrupt status */
	if (ints == 0)
		return 0;

	EFX_ERR(efx, "LM87 detected a hardware failure (status %02x:%02x)\n",
		int_status_1, int_status_2);
	lm87_dump_alarms(efx, int_status_1, int_status_2);

	return -ERANGE;
}
