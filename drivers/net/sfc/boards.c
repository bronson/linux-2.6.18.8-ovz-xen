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
 ****************************************************************************
 */

#include "net_driver.h"
#include "phy.h"
#include "lm87_support.h"
#include "boards.h"
#include "efx.h"

/* Macros for unpacking the board revision */
/* The revision info is in host byte order. */
#define BOARD_TYPE(_rev) (_rev >> 8)
#define BOARD_MAJOR(_rev) ((_rev >> 4) & 0xf)
#define BOARD_MINOR(_rev) (_rev & 0xf)

/* Blink support. If the PHY has no auto-blink mode so we hang it off a timer */
#define BLINK_INTERVAL (HZ/2)

static void blink_led_timer(unsigned long context)
{
	struct efx_nic *efx = (struct efx_nic *)context;
	struct efx_blinker *bl = &efx->board_info.blinker;
	efx->board_info.set_fault_led(efx, bl->state);
	bl->state = !bl->state;
	if (bl->resubmit) {
		bl->timer.expires = jiffies + BLINK_INTERVAL;
		add_timer(&bl->timer);
	}
}

static void board_blink(struct efx_nic *efx, int blink)
{
	struct efx_blinker *blinker = &efx->board_info.blinker;

	/* The rtnl mutex serialises all ethtool ioctls, so
	 * nothing special needs doing here. */
	if (blink) {
		blinker->resubmit = 1;
		blinker->state = 0;
		setup_timer(&blinker->timer, blink_led_timer,
			    (unsigned long)efx);
		blinker->timer.expires = jiffies + BLINK_INTERVAL;
		add_timer(&blinker->timer);
	} else {
		blinker->resubmit = 0;
		if (blinker->timer.function)
			del_timer_sync(&blinker->timer);
		efx->board_info.set_fault_led(efx, 0);
	}
}


struct sensor_conf {
	const char *name;
	const unsigned high;
	const unsigned low;
};

#define NO_LIMIT	((unsigned)-1)

#define LM87_SENSOR_BYTES	(18)

static int sensor_limits_to_bytes(const struct sensor_conf *limits,
				  int nlimits, u8 *bytes, int maxbytes)
{
	int i, nbytes;
	nbytes = 0;
	for (i = 0; i < nlimits; i++) {
		bytes[nbytes++] = limits[i].high;
		if (limits[i].low != NO_LIMIT)
			bytes[nbytes++] = limits[i].low;
		/* We may have overrun by one at this point, but this test
		 * should only trigger in development drivers as the sizes
		 * are not dynamic. */
		if (nbytes > maxbytes) {
			printk(KERN_ERR "%s: out of space!\n", __func__);
			break;
		}
	}
	return nbytes;
}

/*****************************************************************************
 * Support for the SFE4002
 *
 */
/* LM87 configuration data for the sensor on the SFE4002 board */
static const struct sensor_conf sfe4002_lm87_limits[] = {
	{"1.8V line", 0x91, 0x83},	/* 2.5V sensor, scaled for 1.8V */
	{"1.2V line", 0x5a, 0x51},	/* Vccp1 */
	{"3.3V line", 0xca, 0xb6},
	{"5V line", 0xc9, 0xb6},
	{"12V line", 0xe0, 0xb0},
	{"1V line", 0x4b, 0x44},	/* vccp2 */
	{"Ext. temp.", 0x46, 0x0a},	/* ASIC temp. */
	{"Int. temp.", 0x3c, 0x0a},	/* Board temp. */
	{"1.66V line", 0xb2, NO_LIMIT},	/* AIN1 only takes 1 value */
	{"1.5V line", 0xa1, NO_LIMIT}	/* AIN2 only takes 1 value */
};

static const int sfe4002_lm87_nlimits = ARRAY_SIZE(sfe4002_lm87_limits);

static u16 sfe4002_lm87_irq_mask = EFX_LM87_NO_INTS;

/* I2C ID of the onboard LM87 chip. This is board-specific as the bottom two
 * bits are set by strap pins */
#define SFE4002_LM87_I2C_ID (0x2e)

/****************************************************************************/
/* LED allocations. Note that on rev A0 boards the schematic and the reality
 * differ: red and green are swapped. Below is the fixed (A1) layout (there
 * are only 3 A0 boards in existence, so no real reason to make this
 * conditional).
 */
#define SFE4002_FAULT_LED (2)	/* Red */
#define SFE4002_RX_LED    (0)	/* Green */
#define SFE4002_TX_LED    (1)	/* Amber */

static int sfe4002_init_leds(struct efx_nic *efx)
{
	/* Set the TX and RX LEDs to reflect status and activity, and the
	 * fault LED off */
	xfp_set_led(efx, SFE4002_TX_LED,
		    QUAKE_LED_TXLINK | QUAKE_LED_LINK_ACTSTAT);
	xfp_set_led(efx, SFE4002_RX_LED,
		    QUAKE_LED_RXLINK | QUAKE_LED_LINK_ACTSTAT);
	xfp_set_led(efx, SFE4002_FAULT_LED, QUAKE_LED_OFF);
	efx->board_info.blinker.led_num = SFE4002_FAULT_LED;
	return 0;
}

static void sfe4002_fault_led(struct efx_nic *efx, int state)
{
	xfp_set_led(efx, SFE4002_FAULT_LED, state ? QUAKE_LED_ON :
			QUAKE_LED_OFF);
}

static int sfe4002_sensor_meaning(struct efx_nic *efx, int limit_num,
				  unsigned val)
{
	const struct sensor_conf *lim = &sfe4002_lm87_limits[limit_num];
	if (lim->low == NO_LIMIT)
		EFX_ERR(efx, "%10s  0x%02x (nominal value 0x%02x)\n", lim->name,
			val, lim->high);
	else
		EFX_ERR(efx, "%10s  0x%02x (nominal range 0x%02x - 0x%02x)\n",
			lim->name, val, lim->high, lim->low);
	return 1;
}

static int sfe4002_check_hw(struct efx_nic *efx)
{
	int rc;

	/* A0 board rev. 4002s  report a temperature fault the whole time
	 * (bad sensor) so we mask it out. */
	unsigned alarm_mask = (efx->board_info.minor > 0) ?
		0 : ~EFX_LM87_ETMP_INT;

	/* Check the sensor (NOP if not present). */
	rc = efx_check_lm87(efx, alarm_mask);

	/* We treat both lm87 interrupts and failure to talk to the lm87
	 * as problems (since failure will only be reported if we did
	 * find the sensor at probe time. */
	if (rc)
		EFX_ERR(efx, "sensor alert!\n");
	return rc;
}

static int sfe4002_init(struct efx_nic *efx)
{
	u8 lm87_bytes[LM87_SENSOR_BYTES];
	int nbytes;
	int rc;

	efx->board_info.monitor = sfe4002_check_hw;
	efx->board_info.interpret_sensor = sfe4002_sensor_meaning;
	efx->board_info.init_leds = sfe4002_init_leds;
	efx->board_info.set_fault_led = sfe4002_fault_led;
	efx->board_info.blink = board_blink;
	/* To clean up shut down the lm87 (NOP if not present) */
	efx->board_info.fini = efx_remove_lm87;

	nbytes = sensor_limits_to_bytes(sfe4002_lm87_limits,
					sfe4002_lm87_nlimits, lm87_bytes,
					LM87_SENSOR_BYTES);

	/* Activate the lm87 sensor if present (succeeds if nothing there) */
	rc = efx_probe_lm87(efx, SFE4002_LM87_I2C_ID,
			    lm87_bytes, nbytes, sfe4002_lm87_irq_mask);

	return rc;
}

/*****************************************************************************
 * Support for the SFE4003
 *
 */
/* LM87 configuration data for the sensor on the SFE4003 board */
static const struct sensor_conf sfe4003_lm87_limits[] = {
	{"1.5V line", 0x78, 0x6d},	/* 2.5V input, values scaled for 1.5V */
	{"1.2V line", 0x5a, 0x51},	/* Vccp1 */
	{"3.3V line", 0xca, 0xb6},
	{"5V line", 0xc0, 0x00},	/* Sensor not connected. */
	{"12V line", 0xe0, 0xb0},
	{"1V line", 0x4b, 0x44},	/* Vccp2 */
	{"Ext. temp.", 0x46, 0x0a},	/* ASIC temp. */
	{"Int. temp.", 0x3c, 0x0a},	/* Board temp. */
	{"", 0xff, NO_LIMIT},		/* FAN1/AIN1 unused */
	{"", 0xff, NO_LIMIT}		/* FAN2/AIN2 unused */
};

static const int sfe4003_lm87_nlimits = ARRAY_SIZE(sfe4003_lm87_limits);

static u16 sfe4003_lm87_irq_mask = EFX_LM87_NO_INTS;


static int sfe4003_sensor_meaning(struct efx_nic *efx, int limit_num,
				  unsigned val)
{
	const struct sensor_conf *lim = &sfe4003_lm87_limits[limit_num];
	if (lim->low == NO_LIMIT)
		return 0; /* Neither AIN1 nor AIN2 mean anything to us */
	else
		EFX_ERR(efx, "%10s  0x%02x (nominal range 0x%02x - 0x%02x)\n",
			lim->name, val, lim->high, lim->low);
	return 1;
}

/* I2C ID of the onboard LM87 chip. This is board-specific as the bottom two
 * bits are set by strap pins */
#define SFE4003_LM87_I2C_ID (0x2e)

/* Board-specific LED info. */
#define SFE4003_RED_LED_GPIO	(11)
#define SFE4003_LED_ON		(1)
#define SFE4003_LED_OFF		(0)

static void sfe4003_fault_led(struct efx_nic *efx, int state)
{
	/* The LEDs were not wired to GPIOs before A3 */
	if (efx->board_info.minor < 3 && efx->board_info.major == 0)
		return;

	txc_set_gpio_val(efx, SFE4003_RED_LED_GPIO,
			 state ? SFE4003_LED_ON : SFE4003_LED_OFF);
}

static int sfe4003_init_leds(struct efx_nic *efx)
{
	/* The LEDs were not wired to GPIOs before A3 */
	if (efx->board_info.minor < 3 && efx->board_info.major == 0)
		return 0;

	txc_set_gpio_dir(efx, SFE4003_RED_LED_GPIO, TXC_GPIO_DIR_OUTPUT);
	txc_set_gpio_val(efx, SFE4003_RED_LED_GPIO, SFE4003_LED_OFF);
	return 0;
}

static int sfe4003_check_hw(struct efx_nic *efx)
{
	int rc;
	/* A0/A1/A2 board rev. 4003s  report a temperature fault the whole time
	 * (bad sensor) so we mask it out. */
	unsigned alarm_mask =
		~(EFX_LM87_ETMP_INT | EFX_LM87_FAN1_INT | EFX_LM87_FAN2_INT);

	/* Check the sensor (NOP if not present). */

	rc = efx_check_lm87(efx, alarm_mask);
	/* We treat both lm87 interrupts and failure to talk to the lm87
	 * as problems (since failure will only be reported if we did
	 * find the sensor at probe time. */
	if (rc)
		EFX_ERR(efx, "sensor alert!\n");

	return rc;
}

static int sfe4003_init(struct efx_nic *efx)
{
	u8 lm87_bytes[LM87_SENSOR_BYTES];
	int nbytes;
	int rc;
	efx->board_info.monitor = sfe4003_check_hw;
	efx->board_info.interpret_sensor = sfe4003_sensor_meaning;
	efx->board_info.init_leds = sfe4003_init_leds;
	efx->board_info.set_fault_led = sfe4003_fault_led;
	efx->board_info.blink = board_blink;
	/* To clean up shut down the lm87 (NOP if not present) */
	efx->board_info.fini = efx_remove_lm87;

	nbytes = sensor_limits_to_bytes(sfe4003_lm87_limits,
					sfe4003_lm87_nlimits, lm87_bytes,
					LM87_SENSOR_BYTES);

	/* Activate the lm87 sensor if present (succeeds if nothing there) */
	rc = efx_probe_lm87(efx, SFE4003_LM87_I2C_ID,
			    lm87_bytes, nbytes, sfe4003_lm87_irq_mask);

	if (rc < 0)
		EFX_ERR(efx, "Temperature sensor probe failure: "
			"please check the jumper position\n");
	return rc;
}

/*****************************************************************************
 * Support for the SFE4005
 *
 */
/* LM87 configuration data for the sensor on the SFE4005 board */
static const u8 sfe4005_lm87_limits[] = {
	0x51, /* 2.5V high lim. (actually monitor 1.0V line, so 1050mV)  */
	0x49, /* 2.5V low lim. (950mV) */
	0xf6, /* Vccp1 high lim. (3.3V rail, 3465 mV) */
	0xde, /* Vcpp1 low lim. (3.3V rail, 3135 mV) */
	0xca, /* 3.3V AUX high lim. (3465 mV)  */
	0xb6, /* 3.3V AUX low lim. (3135mV) */
	0xc0, /* 5V high lim. not connected) */
	0x00, /* 5V low lim. (not connected) */
	0xd0, /* 12V high lim. (13000mV) */
	0xb0, /* 12V low lim. (11000mV) */
	0xc0, /* Vccp2 high lim. (unused) */
	0x00, /* Vccp2 low lim. (unused) */
	0x46, /* Ext temp 1 (ASIC) high lim. */
	0x0a, /* Ext temp 1 low lim. */
	0x3c, /* Int temp (board) high lim. */
	0x0a, /* Int temp 1 low lim. */
	0xff, /* Fan 1 high (unused) */
	0xff, /* Fan 2 high (unused) */
};

#define SFE4005_LM87_I2C_ID (0x2e)

/* Until the LM87 monitoring is interrupt driven. */
#define SFE4005_LM87_IRQMASK	EFX_LM87_NO_INTS

#define SFE4005_PCF8575_I2C_ID	(0x20)
/* Definitions for the I/O expander that controls the CX4 chip:
 * which PCF8575 pin maps to which function */
#define SFE4005_PORT0_EXTLOOP	(1 << 0)
#define SFE4005_PORT1_EXTLOOP	(1 << 1)
#define SFE4005_HOSTPROT_LOOP	(1 << 2)
#define SFE4005_BCAST		(1 << 3) /* TX on both ports */
#define SFE4005_PORT0_EQ	(1 << 4)
#define SFE4005_PORT1_EQ	(1 << 5)
#define SFE4005_HOSTPORT_EQ	(1 << 6)
#define	SFE4005_PORTSEL		(1 << 7) /* Which port (for RX in BCAST mode) */
#define SFE4005_PORT0_PRE_LBN	(8)      /* Preemphasis on port 0 (2 bits)*/
#define SFE4005_PORT1_PRE_LBN	(10)     /* Preemphasis on port 1 (2 bits)*/
#define SFE4005_HOSTPORT_PRE_LBN (12)    /* Preemphasis on host port (2 bits) */
#define SFE4005_UNUSED		(1 << 14)
#define SFE4005_CX4uC_nRESET	(1 << 15) /* Reset the controller on CX4 chip */


/* By default only turn on host port EQ. Can also OR in SFE4005_PORT0_EQ,
 * SFE4005_PORT1_EQ but this hasn't been seen to make a difference. */
#define SFE4005_CX4_DEFAULTS (SFE4005_CX4uC_nRESET | SFE4005_HOSTPORT_EQ)

static int sfe4005_write_ioexpander(struct efx_nic *efx)
{
	unsigned long iobits = (unsigned long)efx->phy_data;
	struct efx_i2c_interface *i2c = &efx->i2c;
	u8 send[2], check[2];
	int rc;
	/* Do not, EVER, deassert nRESET as that will reset Falcon too,
	 * and the driver won't know to repush the configuration, so
	 * nothing will work until the next power cycle. */
	BUG_ON(!(iobits & SFE4005_CX4uC_nRESET));
	send[0] = (iobits & 0xff);
	send[1] = ((iobits >> 8) & 0xff);
	rc = efx_i2c_send_bytes(i2c, SFE4005_PCF8575_I2C_ID, send, 2);
	if (rc) {
		EFX_ERR(efx, "failed to write to I/O expander: %d\n", rc);
		return rc;
	}
	/* Paranoia: just check what the I/O expander reads back */
	rc = efx_i2c_recv_bytes(i2c, SFE4005_PCF8575_I2C_ID, check, 2);
	if (rc)
		EFX_ERR(efx, "failed to read back from I/O expander: %d\n", rc);
	else if (check[0] != send[0] || check[1] != send[1])
		EFX_ERR(efx, "read back wrong value from I/O expander: "
			"wanted %.2x%.2x, got %.2x%.2x\n",
			send[1], send[0], check[1], check[0]);
	return rc;
}

static int sfe4005_init(struct efx_nic *efx)
{
	unsigned long iobits = SFE4005_CX4_DEFAULTS;
	int rc;

	/* There is no PHY as such on the SFE4005 so phy_data is ours. */
	efx->phy_data = (void *)iobits;

	/* Push the values */
	rc = sfe4005_write_ioexpander(efx);
	if (rc)
		return rc;

	/* Activate the lm87 sensor if present (succeeds if nothing there) */
	rc = efx_probe_lm87(efx, SFE4005_LM87_I2C_ID,
			    sfe4005_lm87_limits,
			    sizeof(sfe4005_lm87_limits), SFE4005_LM87_IRQMASK);

	/* To clean up shut down the lm87 (NOP if not present) */
	efx->board_info.fini = efx_remove_lm87;

	return rc;
}

/* This will get expanded as board-specific details get moved out of the
 * PHY drivers. */
struct efx_board_data {
	const char *ref_model;
	const char *gen_type;
	int (*init) (struct efx_nic *nic);
	unsigned mwatts;
};

static void dummy_fini(struct efx_nic *nic)
{
}

static int dummy_init(struct efx_nic *nic)
{
	nic->board_info.fini = dummy_fini;
	return 0;
}

/* Maximum board power (mW)
 * Falcon controller ASIC accounts for 2.2W
 * 10Xpress PHY accounts for 12W
 *
 */
#define SFE4001_POWER 18000
#define SFE4002_POWER 7500
#define SFE4003_POWER 4500
#define SFE4005_POWER 4500

static struct efx_board_data board_data[] = {
	[EFX_BOARD_INVALID] =
	{NULL,	    NULL,                  dummy_init,      0},
	[EFX_BOARD_SFE4001] =
	{"SFE4001", "10GBASE-T adapter",   sfe4001_poweron, SFE4001_POWER },
	[EFX_BOARD_SFE4002] =
	{"SFE4002", "XFP adapter",         sfe4002_init,    SFE4002_POWER },
	[EFX_BOARD_SFE4003] =
	{"SFE4003", "10GBASE-CX4 adapter", sfe4003_init,    SFE4003_POWER },
	[EFX_BOARD_SFE4005] =
	{"SFE4005", "10G blade adapter",   sfe4005_init,    SFE4005_POWER },
};

int efx_set_board_info(struct efx_nic *efx, u16 revision_info)
{
	int rc = 0;
	struct efx_board_data *data;

	if (BOARD_TYPE(revision_info) >= EFX_BOARD_MAX) {
		EFX_ERR(efx, "squashing unknown board type %d\n",
			BOARD_TYPE(revision_info));
		revision_info = 0;
	}

	if (BOARD_TYPE(revision_info) == 0) {
		efx->board_info.major = 0;
		efx->board_info.minor = 0;
		/* For early boards that don't have revision info. there is
		 * only 1 board for each PHY type, so we can work it out, with
		 * the exception of the PHY-less boards. */
		switch (efx->phy_type) {
		case PHY_TYPE_10XPRESS:
			efx->board_info.type = EFX_BOARD_SFE4001;
			break;
		case PHY_TYPE_XFP:
			efx->board_info.type = EFX_BOARD_SFE4002;
			break;
		case PHY_TYPE_CX4_RTMR:
			efx->board_info.type = EFX_BOARD_SFE4003;
			break;
		default:
			efx->board_info.type = 0;
			break;
		}
	} else {
		efx->board_info.type = BOARD_TYPE(revision_info);
		efx->board_info.major = BOARD_MAJOR(revision_info);
		efx->board_info.minor = BOARD_MINOR(revision_info);
	}

	data = &board_data[efx->board_info.type];

	/* Report the board model number or generic type for recognisable
	 * boards. */
	if (efx->board_info.type != 0)
		EFX_INFO(efx, "board is %s rev %c%d\n",
			 (efx->pci_dev->subsystem_vendor == EFX_VENDID_SFC)
			 ? data->ref_model : data->gen_type,
			 'A' + efx->board_info.major, efx->board_info.minor);

	efx->board_info.init = data->init;
	efx->board_info.mwatts = data->mwatts;

	return rc;
}
